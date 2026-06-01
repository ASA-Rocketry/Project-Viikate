# RFC-001: Avionics board v2

**Author(s):** Jozef Jabczun <jabczunjozef@gmail.com>
**Created:** 2025-06-01
**Status:** Draft
**Supersedes:** NONE
**Superseded by:** NONE

---

## 1. Summary

RFC-001 proposes version 2 of the Viikate avionics board. The new board
specification includes the following changes:

- a switch to an STM32H7 architecture,
- GPS,
- radio telemetry,
- separated power and guidance sections,
- current usage sensing,
- modular bus architecture based on CAN,
- removal of magnetometer.

This is to complement the already developed features from version 1 of the
Viikate avionics boards, including, but not limited to:

- 4 servo controls,
- 1 pyrotechnics control channel,
- battery power,
- integrated accelerometer, gyroscope, and barometer.

---

## 2. Motivation

The current Viikate avionics board (version 1) has served as a functional
prototype, but several limitations in its architecture and physical design have
become evident during testing and field use. Version 2 seeks to address these
issues to improve reliability, maintainability, and operational safety, as well
as to support future expansion.

### 1. Board size constraints

The existing avionics board occupies more physical space than desirable within
the airframe, leading to tight mechanical tolerances and increased complexity
during integration. A more compact board layout would make assembly easier,
improve internal wiring, and reduce the total system mass.

### 2. Design flaws in the original layout

Version 1 suffered from multiple small but critical hardware design issues,
including misrouted power traces, missing connections to certain
microcontroller pins, and inconsistent grounding. These errors required
workarounds during assembly and debugging, impacting reliability and
repeatability. A new iteration provides an opportunity to correct these flaws
comprehensively.

### 3. Streamlining pad operations and improving safety

Current assembly and launch preparations require multiple manual steps after
the rocket motor is installed. Some of these steps expose personnel to
unnecessary risk (e.g., connecting or testing live electrical systems near the
pad). Version 2 will simplify and consolidate these operations, reducing time
on the pad and minimizing safety hazards.

### 4. Improving recovery capability

At present, recovery operations can be difficult, especially once the rocket
lands far from the pad or beyond line of sight. Integrating GPS and radio
telemetry into the avionics board will make tracking and recovery significantly
easier, avoiding the need for separate tracking modules.

### 5. Connector reliability 

The connectors used in the current design may show mechanical weakness under
stress and vibration, leading to intermittent connections or partial
disconnections in flight. The updated design will incorporate more secure
connector types suited for the operational environment.

### 6. Data logging and analysis 

The existing logging procedure of logging directly to an SD card is fragile and
may lead to loss of data in higher performance rockets in the future. The new
design will aim to store data internally within the flight controller with a
secure soldered connection.

The new board will also include further sources of telemetry, such as GPS,
current sensing for servos (allows for stall detection and better ground
analysis), etc.

### 7. Magnetometer removal

In practice, the onboard magnetometer has proven unnecessary and may be
unreliable within the electromagnetic environment of the rocket, particularly
near high-current cabling and pyrotechnic circuits. Because its readings may be
noisy and currently contribute little to flight control, it will be removed to
simplify the design.

### 8. Transition away from the Teensy platform 

The Teensy (IMXRT1062) platform used in version 1 has become impractical for
further development due to the large size of the Teensy. The onboard IMXRT1062
microcontroller unit (MCU) is impractical for our uses as it is only packaged
in BGA (ball grid array), which complicates fabrication and rework. Migrating
to an STM32H7 microcontroller provides greater availability, better support,
and an improved development ecosystem while maintaining high performance for
real-time control tasks.

### 9. Long-term expandability 

Version 1’s layout does not lend itself well to modular upgrades or peripheral
expansion. By implementing a CAN-based modular bus, version 2 will allow easy
integration of future subsystems such as advanced telemetry links, additional
sensors, or actuator controllers. This extends the platform’s lifespan and
adaptability.

---

## 3. Goals


The main objectives of Viikate Avionics Board v2 are to improve system
reliability, safety, and maintainability while enabling continued development
of more capable rockets. This version will serve as a mature, flight-ready
platform that corrects shortcomings in version 1 and supports modular future
expansion.

### Primary Goals

1. Reduce physical footprint.
2. Rectify electrical design flaws.
3. Improve operational safety and ease of assembly.
4. Enhance recovery capabilities.
5. Increase connector reliability.
6. Implement reliable onboard data logging.
7. Remove non-essential or unreliable sensors.
8. Migrate to STM32H7 architecture.
9. Enable modular system expansion.

### Non-Goals
1. Complete software feature redesign (should be addressed in separate RFC).

---

## 4. Detailed Design

TBD at meeting

---

## 5. Alternatives Considered

TBD

---

## 6. Drawbacks

TBD

---

## 7. Impact

TBD

---

## 8. Rollout Plan

TBD

---

## 9. References

TBD
