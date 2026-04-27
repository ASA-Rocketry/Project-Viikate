clear *

% Body fixed frame, x: roll axis, y: pitch axis, z: yaw axis
% Inertial frame, NED (x: North, y: East, z: Down)
mf = 0.957; % full mass kg
me = 0.894; % empty mass kg
bt = 1; % motor burnout time s
dmdt = -(mf - me)/bt; % rate of mass change (linear)
Ie = [0.0027 0 0; 0 0.1655 0; 0 0 0.1655];% Empty inertia matrix
If = [0.0027 0 0; 0 0.177 0; 0 0 0.177]; %Full inertia matrix

% Wind in inertial coords
wind = [0 10 7]';
%seed = randi(100);

%Forces
Thrust = [128 0 0]'; %Body fixed frame
gravity = [-9.819 0 0]'; %Inertial frame

Cd0 = 0.7; % coefficient of drag 
A = 0.00442; % frontal area m^2
rho = 1.225; % air density kg/m^3

% fin angles manually set
% a1 = deg2rad(0);
% a2 = deg2rad(1);
% a3 = deg2rad(0);
% a4 = deg2rad(1);
% 
% fin_angles = [a1 a2 a3 a4]';

%Cl = 0.191; %average fin lift coefficient at 5 deg AoA
Af = 0.0016875; %0.003689; % fin area
%Fl = 1/2*Cl*A*Vb^2
lr = 34.15*0.65; %34.15; % center of lift from root of fin
fr = (75/2 + lr)/1000; % Fin moment arm (m)

%Cg = 0.625;
Cp = -0.346;
dfins = 0.455;
%Cl = 0.0334462*a
fin_positions = [dfins 0 fr; dfins -fr 0; dfins 0 -fr; dfins fr 0]; 
r_cp = [Cp 0 0]';

N = [0 -1 0; 0 0 -1; 0 1 0; 0 0 1]; % Canard Lift directions
si = [0 0 1; 0 -1 0; 0 0 -1; 0 1 0]; % Canard span directions 

Cla = 0.633; %1.975*0.4; %1.975;
CNa = 20; % Normal force coefficient slope
Cmq = -40; % Pitch damping moment coefficient slope
Cnr = -40; %yaw damping moment slope
Clp = -0.3; % roll damping moment slope
L = 1.15; % Length of the rocket
b = 0.14;

% PID target values
ref_w = pi; %Target rolling rate (rad/s)
refz = 50;
refy = 50;
ref_phi = pi/2;

% Pitch and Yaw PID gains
Pgainz = deg2rad(20);
Pgainy = deg2rad(20);
Dgain = 0.2;
Igain = 0;

% Roll orientation PID gains
Pgainphi = deg2rad(0.05);
Dgainphi = 0.015;
Igainphi = 0;

tau = 0.05; % actuatuor time constant for fin servos
alim = deg2rad(600); % servo rate limiter

Vsafe = 0.1; %

out = sim("closedmodel.slx");
