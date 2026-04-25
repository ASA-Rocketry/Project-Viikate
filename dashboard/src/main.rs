use std::{
    collections::VecDeque,
    io::{Read, Write},
    time::Duration,
};

use bevy::{math::primitives::Cuboid, prelude::*};
use bevy_egui::{
    EguiContexts, EguiPlugin, EguiPrimaryContextPass,
    egui::{self, mutex::Mutex},
};
use egui_plot::{Legend, Line, LineStyle, Plot, PlotPoints};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;

const PACKET_MAGIC: u128 = 0x68d7ede87b264e66b6091a22a3325b10;
const HEADER_SIZE: usize = 16 + 4;
const MAX_PACKET_SIZE: usize = 64 * 1024;

// History LOD:
// - Recent data stays raw
// - Older data is bucketed
// - Very old data is bucketed more coarsely and kept indefinitely
const RECENT_WINDOW_S: f64 = 60.0;
const MEDIUM_WINDOW_S: f64 = 15.0 * 60.0;
const MEDIUM_BUCKET_S: f64 = 0.25;
const LONG_BUCKET_S: f64 = 2.0;

// 3D scene scaling.
// Since we only have altitude, the rocket stays at x=0, z=0 and altitude is
// scaled.
const WORLD_ALTITUDE_SCALE: f32 = 0.02;

// Set this to true if ori_x / ori_y / ori_z are degrees instead of radians.
const ORIENTATION_IS_DEGREES: bool = true;

// Fixed camera setup.
const CAMERA_POSITION: Vec3 = Vec3::new(-18.0, 10.0, 18.0);
const CAMERA_LOOK_AT: Vec3 = Vec3::new(0.0, 4.0, 0.0);

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FlightData {
    pub time_ms: u64,
    pub altitude: f32,
    pub vertical_velocity: f32,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
    pub rot_x: f32,
    pub rot_y: f32,
    pub rot_z: f32,
    pub ori_x: f32,
    pub ori_y: f32,
    pub ori_z: f32,
    pub mag_x: f64,
    pub mag_y: f64,
    pub mag_z: f64,
    pub heading: f64,
    pub accel_magnitude: f32,
    pub rbf_removed: bool,
}

#[derive(Resource, Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, rename_all = "camelCase")]
pub struct FlightConstants {
    // Optional fields allow partial updates
    pub k_proportional: f32,
    pub k_integrator: f32,
    pub k_derivative: f32,
    pub k_servo_trim1: f32,
    pub k_servo_trim2: f32,
    pub k_servo_trim3: f32,
    pub k_servo_trim4: f32,
}

impl Default for FlightConstants {
    fn default() -> Self {
        Self {
            k_proportional: 0.1745f32 * (180.0f32 / std::f32::consts::PI),
            k_integrator: 0.0f32,
            k_derivative: 0.09f32 * (180.0f32 / std::f32::consts::PI),
            k_servo_trim1: -10.0f32,
            k_servo_trim2: -10.0f32,
            k_servo_trim3: -10.0f32,
            k_servo_trim4: -10.0f32,
        }
    }
}

#[derive(Resource, Deref, DerefMut)]
struct SerialPortRes(Mutex<Box<dyn SerialPort>>);

#[derive(Resource, Default, Deref, DerefMut)]
struct SerialRxBuffer(Vec<u8>);

#[derive(Resource, Default)]
struct LatestFlightData(Option<FlightData>);

#[derive(Clone, Copy, Debug)]
struct Sample {
    t: f64,
    v: f64,
}

#[derive(Clone, Debug)]
struct BucketSummary {
    end_t: f64,
    first: Sample,
    min: Sample,
    max: Sample,
    last: Sample,
}

impl BucketSummary {
    fn representative_points(&self) -> Vec<[f64; 2]> {
        let mut pts = vec![
            [self.first.t, self.first.v],
            [self.min.t, self.min.v],
            [self.max.t, self.max.v],
            [self.last.t, self.last.v],
        ];

        pts.sort_by(|a, b| a[0].total_cmp(&b[0]));

        pts.dedup_by(|a, b| a[0].to_bits() == b[0].to_bits() && a[1].to_bits() == b[1].to_bits());

        pts
    }

    fn representative_samples(&self) -> Vec<Sample> {
        self.representative_points()
            .into_iter()
            .map(|[t, v]| Sample { t, v })
            .collect()
    }
}

#[derive(Clone, Debug)]
struct BucketAccumulator {
    bucket_index: i64,
    bucket_width: f64,
    first: Sample,
    min: Sample,
    max: Sample,
    last: Sample,
}

impl BucketAccumulator {
    fn new(bucket_index: i64, bucket_width: f64, sample: Sample) -> Self {
        Self {
            bucket_index,
            bucket_width,
            first: sample,
            min: sample,
            max: sample,
            last: sample,
        }
    }

    fn push(&mut self, sample: Sample) {
        if sample.v < self.min.v {
            self.min = sample;
        }
        if sample.v > self.max.v {
            self.max = sample;
        }
        self.last = sample;
    }

    fn start_t(&self) -> f64 {
        self.bucket_index as f64 * self.bucket_width
    }

    fn end_t(&self) -> f64 {
        self.start_t() + self.bucket_width
    }

    fn finalize(self) -> BucketSummary {
        BucketSummary {
            end_t: self.end_t(),
            first: self.first,
            min: self.min,
            max: self.max,
            last: self.last,
        }
    }

    fn preview(&self) -> BucketSummary {
        BucketSummary {
            end_t: self.end_t(),
            first: self.first,
            min: self.min,
            max: self.max,
            last: self.last,
        }
    }
}

#[derive(Debug, Default)]
struct LodSeries {
    recent: VecDeque<Sample>,
    medium: VecDeque<BucketSummary>,
    medium_work: Option<BucketAccumulator>,
    long: VecDeque<BucketSummary>,
    long_work: Option<BucketAccumulator>,
}

impl LodSeries {
    fn push(&mut self, t: f64, v: f64) {
        self.recent.push_back(Sample { t, v });
        self.compact(t);
    }

    fn compact(&mut self, now: f64) {
        self.compact_recent(now);
        self.compact_medium(now);
    }

    fn compact_recent(&mut self, now: f64) {
        let cutoff = now - RECENT_WINDOW_S;

        while matches!(self.recent.front(), Some(sample) if sample.t < cutoff) {
            let sample = self.recent.pop_front().unwrap();
            self.push_into_medium(sample);
        }
    }

    fn compact_medium(&mut self, now: f64) {
        let cutoff = now - RECENT_WINDOW_S - MEDIUM_WINDOW_S;

        while matches!(self.medium.front(), Some(bucket) if bucket.end_t < cutoff) {
            let bucket = self.medium.pop_front().unwrap();
            for sample in bucket.representative_samples() {
                self.push_into_long(sample);
            }
        }
    }

    fn push_into_medium(&mut self, sample: Sample) {
        push_into_bucket_level(
            &mut self.medium_work,
            &mut self.medium,
            MEDIUM_BUCKET_S,
            sample,
        );
    }

    fn push_into_long(&mut self, sample: Sample) {
        push_into_bucket_level(&mut self.long_work, &mut self.long, LONG_BUCKET_S, sample);
    }

    fn plot_points(&self) -> Vec<[f64; 2]> {
        let mut points = Vec::new();

        for bucket in &self.long {
            points.extend(bucket.representative_points());
        }
        if let Some(bucket) = &self.long_work {
            points.extend(bucket.preview().representative_points());
        }

        for bucket in &self.medium {
            points.extend(bucket.representative_points());
        }
        if let Some(bucket) = &self.medium_work {
            points.extend(bucket.preview().representative_points());
        }

        for sample in &self.recent {
            points.push([sample.t, sample.v]);
        }

        points
    }
}

fn push_into_bucket_level(
    work: &mut Option<BucketAccumulator>,
    finished: &mut VecDeque<BucketSummary>,
    bucket_width: f64,
    sample: Sample,
) {
    let bucket_index = (sample.t / bucket_width).floor() as i64;

    match work {
        Some(acc) if acc.bucket_index == bucket_index => {
            acc.push(sample);
        }
        Some(acc) => {
            let old = std::mem::replace(
                acc,
                BucketAccumulator::new(bucket_index, bucket_width, sample),
            );
            finished.push_back(old.finalize());
        }
        None => {
            *work = Some(BucketAccumulator::new(bucket_index, bucket_width, sample));
        }
    }
}

#[derive(Resource, Default)]
struct FlightHistory {
    altitude: LodSeries,
    vertical_velocity: LodSeries,
    acc_x: LodSeries,
    acc_y: LodSeries,
    acc_z: LodSeries,
    accel_magnitude: LodSeries,
    rot_x: LodSeries,
    rot_y: LodSeries,
    rot_z: LodSeries,
    ori_x: LodSeries,
    ori_y: LodSeries,
    ori_z: LodSeries,
    mag_x: LodSeries,
    mag_y: LodSeries,
    mag_z: LodSeries,
    heading: LodSeries,
}

impl FlightHistory {
    fn push(&mut self, data: &FlightData) {
        let t = data.time_ms as f64 / 1000.0;

        self.altitude.push(t, data.altitude as f64);
        self.vertical_velocity
            .push(t, data.vertical_velocity as f64);

        self.acc_x.push(t, data.acc_x as f64);
        self.acc_y.push(t, data.acc_y as f64);
        self.acc_z.push(t, data.acc_z as f64);
        self.accel_magnitude.push(t, data.accel_magnitude as f64);

        self.rot_x.push(t, data.rot_x as f64);
        self.rot_y.push(t, data.rot_y as f64);
        self.rot_z.push(t, data.rot_z as f64);

        self.ori_x.push(t, data.ori_x as f64);
        self.ori_y.push(t, data.ori_y as f64);
        self.ori_z.push(t, data.ori_z as f64);

        self.mag_x.push(t, data.mag_x);
        self.mag_y.push(t, data.mag_y);
        self.mag_z.push(t, data.mag_z);

        self.heading.push(t, data.heading);
    }
}

#[derive(Resource, Default)]
struct FlightReference {
    base_altitude_m: Option<f32>,
}

#[derive(Resource, Default)]
struct RocketTrail {
    points: Vec<Vec3>,
    last_time_ms: Option<u64>,
}

#[derive(Component)]
struct RocketVisualRoot;

fn main() {
    let sp = SerialPortRes(Mutex::new(
        serialport::new("/dev/tty1", 115200)
            .timeout(Duration::from_millis(10))
            .open()
            .unwrap(),
    ));

    App::new()
        .insert_resource(sp)
        .init_resource::<FlightConstants>()
        .insert_resource(SerialRxBuffer::default())
        .insert_resource(LatestFlightData::default())
        .insert_resource(FlightHistory::default())
        .insert_resource(FlightReference::default())
        .insert_resource(RocketTrail::default())
        .insert_resource(ClearColor(Color::srgb(0.03, 0.04, 0.06)))
        //.insert_resource(AmbientLight {
        //    color: Color::WHITE,
        //    brightness: 500.0,
        //    ..default()
        //})
        .add_plugins(DefaultPlugins)
        .add_plugins(EguiPlugin::default())
        .add_systems(Startup, setup_scene_system)
        .add_systems(
            Update,
            (
                serial_port_read,
                sync_rocket_visual_system,
                draw_scene_gizmos_system,
            ),
        )
        .add_systems(EguiPrimaryContextPass, ui_system)
        .run();
}

fn setup_scene_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(CAMERA_POSITION).looking_at(CAMERA_LOOK_AT, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 25_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(12.0, 20.0, 12.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let body_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.90, 0.92, 0.97),
        metallic: 0.15,
        perceptual_roughness: 0.35,
        ..default()
    });
    let nose_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.85, 0.12, 0.12),
        metallic: 0.05,
        perceptual_roughness: 0.5,
        ..default()
    });
    let fin_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.16, 0.16, 0.20),
        metallic: 0.25,
        perceptual_roughness: 0.55,
        ..default()
    });

    commands
        .spawn((
            Transform::default(),
            GlobalTransform::default(),
            RocketVisualRoot,
        ))
        .with_children(|parent| {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.35, 2.4, 0.35))),
                MeshMaterial3d(body_mat.clone()),
                Transform::from_xyz(0.0, 1.2, 0.0),
            ));

            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.22, 0.50, 0.22))),
                MeshMaterial3d(nose_mat.clone()),
                Transform::from_xyz(0.0, 2.65, 0.0),
            ));

            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.40, 0.14, 0.40))),
                MeshMaterial3d(fin_mat.clone()),
                Transform::from_xyz(0.0, 0.18, 0.0),
            ));

            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.08, 0.40, 0.32))),
                MeshMaterial3d(fin_mat.clone()),
                Transform::from_xyz(0.21, 0.25, 0.0),
            ));

            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.08, 0.40, 0.32))),
                MeshMaterial3d(fin_mat.clone()),
                Transform::from_xyz(-0.21, 0.25, 0.0),
            ));

            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.32, 0.40, 0.08))),
                MeshMaterial3d(fin_mat.clone()),
                Transform::from_xyz(0.0, 0.25, 0.21),
            ));

            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.32, 0.40, 0.08))),
                MeshMaterial3d(fin_mat.clone()),
                Transform::from_xyz(0.0, 0.25, -0.21),
            ));
        });
}

fn serial_port_read(
    port: Res<SerialPortRes>,
    mut rx: ResMut<SerialRxBuffer>,
    mut latest: ResMut<LatestFlightData>,
    mut history: ResMut<FlightHistory>,
) {
    let mut port = port.lock();

    let mut temp = [0u8; 1024];
    match port.read(&mut temp) {
        Ok(n) if n > 0 => {
            rx.extend_from_slice(&temp[..n]);
        }
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
        Err(e) => {
            warn!("Serial read error: {e}");
            return;
        }
    }

    let magic_bytes = PACKET_MAGIC.to_le_bytes();

    loop {
        if rx.len() < HEADER_SIZE {
            break;
        }

        let Some(start) = find_subslice(&rx, &magic_bytes) else {
            let keep = magic_bytes.len().saturating_sub(1);
            if rx.len() > keep {
                let drain_to = rx.len() - keep;
                rx.drain(..drain_to);
            }
            break;
        };

        if start > 0 {
            rx.drain(..start);
        }

        if rx.len() < HEADER_SIZE {
            break;
        }

        let magic = u128::from_le_bytes(rx[0..16].try_into().unwrap());
        let length = u32::from_le_bytes(rx[16..20].try_into().unwrap()) as usize;

        if magic != PACKET_MAGIC {
            rx.drain(..1);
            continue;
        }

        if length > MAX_PACKET_SIZE {
            warn!("Packet too large ({length} bytes), skipping");
            rx.drain(..1);
            continue;
        }

        let total_len = HEADER_SIZE + length;
        if rx.len() < total_len {
            break;
        }

        let body = &rx[HEADER_SIZE..total_len];
        match serde_json::from_slice::<FlightData>(body) {
            Ok(data) => {
                history.push(&data);
                latest.0 = Some(data);
            }
            Err(e) => {
                warn!("Failed to parse FlightData JSON: {e}");
            }
        }

        rx.drain(..total_len);
    }
}

fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

fn orientation_value(v: f32) -> f32 {
    if ORIENTATION_IS_DEGREES {
        v.to_radians()
    } else {
        v
    }
}

fn world_rotation(data: &FlightData) -> Quat {
    Quat::from_euler(
        EulerRot::XYZ,
        orientation_value(data.ori_x),
        orientation_value(data.ori_y),
        orientation_value(data.ori_z),
    )
}

fn world_position(data: &FlightData, reference: &mut FlightReference) -> Vec3 {
    let base_altitude = *reference.base_altitude_m.get_or_insert(data.altitude);
    let y = (data.altitude - base_altitude) * WORLD_ALTITUDE_SCALE;
    Vec3::new(0.0, y, 0.0)
}

fn sync_rocket_visual_system(
    latest: Res<LatestFlightData>,
    mut reference: ResMut<FlightReference>,
    mut trail: ResMut<RocketTrail>,
    mut rocket_query: Query<&mut Transform, With<RocketVisualRoot>>,
) {
    if !latest.is_changed() {
        return;
    }

    let Some(data) = latest.0.as_ref() else {
        return;
    };

    if trail.last_time_ms == Some(data.time_ms) {
        return;
    }

    let position = world_position(data, &mut reference);
    let rotation = world_rotation(data);

    for mut transform in &mut rocket_query {
        transform.translation = position;
        transform.rotation = rotation;
    }

    trail.points.push(position);
    trail.last_time_ms = Some(data.time_ms);
}

fn draw_scene_gizmos_system(mut gizmos: Gizmos, trail: Res<RocketTrail>) {
    let half_extent = 20.0;
    let grid_count = 20;

    for i in -grid_count..=grid_count {
        let p = i as f32;

        let x_color = if i == 0 {
            Color::srgb(0.65, 0.20, 0.20)
        } else if i % 5 == 0 {
            Color::srgb(0.28, 0.28, 0.34)
        } else {
            Color::srgb(0.17, 0.17, 0.22)
        };

        let z_color = if i == 0 {
            Color::srgb(0.20, 0.20, 0.65)
        } else if i % 5 == 0 {
            Color::srgb(0.28, 0.28, 0.34)
        } else {
            Color::srgb(0.17, 0.17, 0.22)
        };

        gizmos.line(
            Vec3::new(p, 0.0, -half_extent),
            Vec3::new(p, 0.0, half_extent),
            x_color,
        );

        gizmos.line(
            Vec3::new(-half_extent, 0.0, p),
            Vec3::new(half_extent, 0.0, p),
            z_color,
        );
    }

    gizmos.line(
        Vec3::ZERO,
        Vec3::new(0.0, 20.0, 0.0),
        Color::srgb(0.20, 0.85, 0.20),
    );

    if trail.points.len() >= 2 {
        gizmos.linestrip(trail.points.iter().copied(), Color::srgb(1.0, 0.75, 0.20));
    }
}

fn make_line(name: &str, series: &LodSeries, color: egui::Color32) -> Line<'static> {
    Line::new(name.to_owned(), PlotPoints::from_iter(series.plot_points()))
        .color(color)
        .style(LineStyle::Solid)
}

// Send any serializable value as a packet with the common header:
// [u128 PACKET_MAGIC little-endian][u32 length little-endian][JSON body]
fn send_packet_json<T: Serialize>(port_res: &SerialPortRes, value: &T) {
    let Ok(body) = serde_json::to_vec(value) else {
        warn!("Failed to serialize JSON for outbound packet");
        return;
    };
    if body.len() > MAX_PACKET_SIZE {
        warn!(
            "Outbound packet too large ({} bytes), not sending",
            body.len()
        );
        return;
    }

    let mut buf = Vec::with_capacity(HEADER_SIZE + body.len());
    buf.extend_from_slice(&PACKET_MAGIC.to_le_bytes());
    buf.extend_from_slice(&(body.len() as u32).to_le_bytes());
    buf.extend_from_slice(&body);

    let mut port = port_res.lock();
    if let Err(e) = port.write_all(&buf) {
        warn!("Serial write error: {e}");
    }
}

fn ui_system(
    mut contexts: EguiContexts,
    latest: Res<LatestFlightData>,
    history: Res<FlightHistory>,
    trail: Res<RocketTrail>,
    mut constants: ResMut<FlightConstants>,
    port: Res<SerialPortRes>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::Window::new("Controls")
        .default_width(420.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.heading("Flight constants");
            ui.label("Adjust constants below. Changes are sent immediately to the controller.");

            let mut changed = false;

            ui.separator();
            ui.collapsing("PID Gains", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Kp");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_proportional)
                            .speed(0.01)
                            .clamp_range(-1000.0..=1000.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Ki");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_integrator)
                            .speed(0.001)
                            .clamp_range(-1000.0..=1000.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Kd");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_derivative)
                            .speed(0.01)
                            .clamp_range(-1000.0..=1000.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });
            });

            ui.separator();
            ui.collapsing("Servo trims (degrees)", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Trim 1");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_servo_trim1)
                            .speed(0.25)
                            .clamp_range(-90.0..=90.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Trim 2");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_servo_trim2)
                            .speed(0.25)
                            .clamp_range(-90.0..=90.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Trim 3");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_servo_trim3)
                            .speed(0.25)
                            .clamp_range(-90.0..=90.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Trim 4");
                    let resp = ui.add(
                        egui::DragValue::new(&mut constants.k_servo_trim4)
                            .speed(0.25)
                            .clamp_range(-90.0..=90.0),
                    );
                    if resp.changed() {
                        changed = true;
                    }
                });
            });

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Reset to defaults").clicked() {
                    *constants = FlightConstants::default();
                    changed = true;
                }
                ui.label("Use with care during flight.");
            });

            if changed {
                send_packet_json(&port, &*constants);
                ui.label(egui::RichText::new("Constants updated and sent").italics());
            }
        });

    egui::Window::new("Flight Telemetry")
        .default_width(420.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.heading("3D view");
            ui.label("Fixed camera enabled");
            ui.label(
                "Note: trail is vertical because telemetry currently has altitude but not x/z position.",
            );

            ui.separator();

            ui.label(format!(
                "History policy: last {:.0}s raw, next {:.0}min at {:.2}s buckets, older at {:.1}s buckets",
                RECENT_WINDOW_S,
                MEDIUM_WINDOW_S / 60.0,
                MEDIUM_BUCKET_S,
                LONG_BUCKET_S,
            ));

            ui.label(format!("trail points kept: {}", trail.points.len()));

            if let Some(data) = &latest.0 {
                ui.separator();
                ui.label(format!("time: {} ms", data.time_ms));
                ui.label(format!("altitude: {:.2} m", data.altitude));
                ui.label(format!(
                    "vertical velocity: {:.2} m/s",
                    data.vertical_velocity
                ));
                ui.label(format!("heading: {:.4} rad", data.heading));
                ui.label(format!(
                    "orientation: ({:.3}, {:.3}, {:.3})",
                    data.ori_x, data.ori_y, data.ori_z
                ));
                ui.label(format!("rbf removed: {}", data.rbf_removed));
            } else {
                ui.separator();
                ui.label("No flight data yet");
            }

            ui.separator();

            Plot::new("altitude_plot")
                .height(180.0)
                .legend(Legend::default())
                .show_axes(true)
                .show_grid(true)
                .show(ui, |plot_ui| {
                    plot_ui.line(make_line(
                        "Altitude (m)",
                        &history.altitude,
                        egui::Color32::from_rgb(80, 200, 120),
                    ));
                    plot_ui.line(make_line(
                        "Vertical velocity (m/s)",
                        &history.vertical_velocity,
                        egui::Color32::from_rgb(80, 160, 255),
                    ));
                });

            Plot::new("acceleration_plot")
                .height(220.0)
                .legend(Legend::default())
                .show_axes(true)
                .show_grid(true)
                .show(ui, |plot_ui| {
                    plot_ui.line(make_line(
                        "Acc X",
                        &history.acc_x,
                        egui::Color32::from_rgb(255, 100, 100),
                    ));
                    plot_ui.line(make_line(
                        "Acc Y",
                        &history.acc_y,
                        egui::Color32::from_rgb(100, 255, 100),
                    ));
                    plot_ui.line(make_line(
                        "Acc Z",
                        &history.acc_z,
                        egui::Color32::from_rgb(100, 150, 255),
                    ));
                    plot_ui.line(make_line(
                        "Accel magnitude",
                        &history.accel_magnitude,
                        egui::Color32::from_rgb(255, 220, 100),
                    ));
                });

            Plot::new("rotation_plot")
                .height(220.0)
                .legend(Legend::default())
                .show_axes(true)
                .show_grid(true)
                .show(ui, |plot_ui| {
                    plot_ui.line(make_line(
                        "Rot X",
                        &history.rot_x,
                        egui::Color32::from_rgb(255, 120, 120),
                    ));
                    plot_ui.line(make_line(
                        "Rot Y",
                        &history.rot_y,
                        egui::Color32::from_rgb(120, 255, 120),
                    ));
                    plot_ui.line(make_line(
                        "Rot Z",
                        &history.rot_z,
                        egui::Color32::from_rgb(120, 170, 255),
                    ));
                });

            Plot::new("orientation_plot")
                .height(220.0)
                .legend(Legend::default())
                .show_axes(true)
                .show_grid(true)
                .show(ui, |plot_ui| {
                    plot_ui.line(make_line(
                        "Ori X",
                        &history.ori_x,
                        egui::Color32::from_rgb(240, 120, 120),
                    ));
                    plot_ui.line(make_line(
                        "Ori Y",
                        &history.ori_y,
                        egui::Color32::from_rgb(120, 240, 120),
                    ));
                    plot_ui.line(make_line(
                        "Ori Z",
                        &history.ori_z,
                        egui::Color32::from_rgb(120, 180, 240),
                    ));
                });

            Plot::new("magnetometer_plot")
                .height(220.0)
                .legend(Legend::default())
                .show_axes(true)
                .show_grid(true)
                .show(ui, |plot_ui| {
                    plot_ui.line(make_line(
                        "Mag X",
                        &history.mag_x,
                        egui::Color32::from_rgb(255, 140, 140),
                    ));
                    plot_ui.line(make_line(
                        "Mag Y",
                        &history.mag_y,
                        egui::Color32::from_rgb(140, 255, 140),
                    ));
                    plot_ui.line(make_line(
                        "Mag Z",
                        &history.mag_z,
                        egui::Color32::from_rgb(140, 180, 255),
                    ));
                });

            Plot::new("heading_plot")
                .height(180.0)
                .legend(Legend::default())
                .show_axes(true)
                .show_grid(true)
                .show(ui, |plot_ui| {
                    plot_ui.line(make_line(
                        "Heading (rad)",
                        &history.heading,
                        egui::Color32::from_rgb(255, 200, 80),
                    ));
                });
        });
}
