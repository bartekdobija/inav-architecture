# 00 · Glossary

Plain-English definitions for programmers who are new to flight controllers or to INAV's implementation vocabulary.

**[← Back to index](../README.md)** · Next: [01 · Architecture Overview](01-architecture-overview.md)

---

## Aircraft and hardware

| Term | Definition |
|---|---|
| **Flight controller (FC)** | The computer and firmware that read sensors/pilot input and calculate motor/servo commands. In this documentation, FC can also mean the `fc/` application-composition module. |
| **UAV** | Unmanned aerial vehicle. A broad term for an aircraft without an onboard pilot. |
| **Multirotor** | An aircraft controlled mainly by varying thrust from several propellers, such as a quadcopter. |
| **Fixed wing** | An aircraft that generates lift from wings, such as an airplane or flying wing. It normally controls attitude with control surfaces and forward thrust separately. |
| **VTOL** | Vertical takeoff and landing. A vehicle that can take off/land vertically and may transition between multirotor and fixed-wing flight. |
| **Rover / boat** | Ground/surface vehicles supported by INAV's navigation and mixer logic. They share some fixed-wing control paths but are not airplanes. |
| **RC** | Radio control. The pilot's transmitter/receiver link and its channel values. |
| **RPY** | Roll, Pitch, Yaw: the three attitude-control axes. Arrays often add throttle as a fourth RC command. |
| **AUX** | Auxiliary RC channels or, in `TASK_AUX`, the 100 Hz task that updates controller coefficients and related auxiliary control state. Context distinguishes the meanings. |
| **A/E/T/R** | Aileron, Elevator, Throttle, Rudder: the conventional source-channel order remapped by INAV's `rcmap` setting. |
| **MC** | Multirotor/multicopter. Used as a shorthand in diagrams and navigation/controller filenames. |
| **Receiver (RX)** | Hardware or software that receives pilot channels. In code, RX also refers to the receiver-processing task/module. |
| **Transmitter (TX)** | In RC usage, the pilot's radio. In serial code, TX more generally means the transmit direction. |
| **ESC** | Electronic speed controller. Hardware that drives a motor using commands from the flight controller; many ESCs also report RPM, voltage, current, or temperature. |
| **Servo** | An actuator that moves a control surface or mechanism to a commanded position. |
| **Control surface** | A movable airplane surface such as an aileron, elevator, or rudder. |
| **Throttle** | A command related to motor power. It is not necessarily the same as actual thrust, RPM, or motor rotation. |
| **IMU tag / device tag** | An identifier that distinguishes multiple physical devices of the same type on a board. |

## Sensors and estimation

| Term | Definition |
|---|---|
| **IMU** | Inertial Measurement Unit. Usually a chip containing a gyroscope and accelerometer; sometimes also includes other sensors. In `flight/imu.c`, “IMU” also refers to attitude-estimation logic. |
| **Gyroscope / gyro** | A sensor that measures angular rotation rate. INAV's control gyro values are usually in degrees per second; the AHRS receives radians per second. |
| **Accelerometer / accel** | A sensor that measures specific force/acceleration. It helps estimate gravity direction and therefore roll/pitch, but vibration and maneuver acceleration can confuse it. |
| **Magnetometer / compass / mag** | A sensor that measures magnetic field direction, used as a heading reference after calibration and alignment. |
| **Barometer / baro** | A pressure sensor used to estimate altitude changes. Weather, prop wash, temperature, and ground effect can affect it. |
| **GPS** | Global Positioning System. In the code, often used generically for satellite positioning receivers and their normalized solution data. |
| **GNSS** | Global Navigation Satellite System. The generic term for GPS and comparable satellite navigation systems. |
| **Pitot tube / pitot** | A differential-pressure sensor arrangement used to estimate airspeed, especially for fixed-wing aircraft. |
| **Rangefinder** | A sensor such as lidar or sonar that measures distance to a surface. INAV can use tilt-compensated distance for above-ground estimation. |
| **Optical flow** | A camera-based motion estimate. Combined with height and attitude, it can estimate horizontal velocity without GPS. |
| **ADSB** | Automatic Dependent Surveillance–Broadcast. A source of nearby-aircraft reports used by INAV for traffic display/proximity logic. |
| **AHRS** | Attitude and Heading Reference System. Logic that estimates vehicle orientation. INAV uses a quaternion-based Mahony-style AHRS in `flight/imu.c`. |
| **Attitude** | Vehicle orientation: roll, pitch, and yaw. It is not position. |
| **Heading** | The horizontal direction associated with the vehicle or its course. Heading and course over ground can differ. |
| **Quaternion** | A four-number orientation representation that avoids some singularities of Euler angles. INAV stores orientation as a quaternion and derives other forms. |
| **Euler angles** | Roll/pitch/yaw angles. They are easy to display and use for control but need explicit units and sign conventions. |
| **Rotation matrix** | A 3×3 matrix that rotates vectors between coordinate frames. INAV caches `rMat` from its orientation. |
| **Mahony filter** | A complementary orientation estimator that integrates gyro and corrects drift using reference vectors such as gravity and magnetic north. |
| **EKF** | Extended Kalman Filter. A probabilistic state estimator using a covariance model. INAV's navigation position estimator in this snapshot is a constant-acceleration predictor/corrector, not a full covariance EKF. |
| **Complementary filter / predictor-corrector** | Estimation that combines a model prediction with corrections from measurements. INAV uses this approach for local position and velocity. |
| **Calibration** | The process of estimating offsets, gains, orientation, or reference values so measurements have useful zero/scale/alignment. |
| **Hard iron / soft iron** | Magnetic distortions. Hard-iron offset shifts the field; soft-iron effects distort scale/shape. Compass calibration estimates correction parameters. |
| **Alignment** | Rotation from a sensor's physical axes into board/vehicle axes. A correctly scaled sensor can still be wrong if alignment is wrong. |
| **Clipping** | Saturation of a sensor measurement at its measurable limit. Clipped acceleration loses information. |
| **Freshness** | Evidence that data arrived recently enough to use. A detected or “healthy” device can still have stale data. |
| **Health** | A module-specific statement about device availability or communication. It does not have one universal meaning across all INAV sensors. |
| **EPH / EPV** | Estimated horizontal/vertical position error, normally in centimetres in navigation code. Larger values mean less confidence. |
| **Dead reckoning** | Estimating motion by integrating previous state and measured movement when absolute position updates are unavailable. |

## Coordinates, units, and numeric representations

| Term | Definition |
|---|---|
| **Coordinate frame** | The convention for what X, Y, and Z mean. A vector is incomplete without knowing its frame. |
| **Body frame (BF)** | Axes attached to the vehicle after sensor/board alignment. |
| **Earth frame (EF)** | Axes attached to the local navigation environment rather than the rotating vehicle. |
| **NEU** | North, East, Up. INAV navigation's local position convention: X north, Y east, Z up. |
| **NED** | North, East, Down. GPS velocity is supplied in this convention; down speed must be negated for NEU Z. |
| **NWU** | North, West, Up. The raw AHRS earth-frame convention documented in `flight/imu.c`; conversion helpers apply sign changes for navigation-facing NEU. |
| **LLH** | Latitude, longitude, height/altitude. GPS waypoint and origin data use this geodetic representation. |
| **AGL** | Above Ground Level. Height above the surface rather than above home or sea level. |
| **AMSL / MSL** | Above Mean Sea Level / Mean Sea Level. An absolute altitude datum. |
| **COG** | Course Over Ground. Direction of actual movement, which can differ from the nose's heading in wind. |
| **Decidegree** | One tenth of a degree: 180° = 1800. Used by attitude and GPS course values. |
| **Centidegree** | One hundredth of a degree: 180° = 18000. Used by several navigation yaw/bearing fields. |
| **deg/s or dps** | Degrees per second. The main gyro/filter/rate-target unit. |
| **rad/s** | Radians per second. Used by the AHRS rotation input. |
| **cm/s²** | Centimetres per second squared. Acceleration unit used by IMU/navigation interfaces. |
| **cA / cW** | Centiamps/centiwatts: 0.01 A and 0.01 W in battery/power-limiter code. |
| **Q12** | A fixed-point scale where 4096 represents 1.0. Accelerometer calibration gains use this representation. |
| **Normalized command** | A dimensionless control range such as −500…+500. It becomes an angle, rate, or actuator correction only after applying the relevant scaling. |
| **PWM-like value** | A numeric command domain resembling servo pulse microseconds, often 1000–2000. It may later be converted to DShot or serial protocol values and is not always an actual pulse width. |
| **`dT`** | Delta time: elapsed time between control updates, normally in seconds in controller code. INAV's PID task derives it from the actual scheduler interval. |

## Control and stabilization

| Term | Definition |
|---|---|
| **Setpoint / target** | The desired value for a controller, such as a roll angle, yaw rate, climb rate, or position. |
| **Rate control** | Control of angular velocity, such as degrees per second of roll, using gyro feedback. |
| **Angle control** | Control of orientation. INAV commonly converts angle error into a rate target, then runs rate control—a cascade. |
| **Cascade controller** | An outer controller that produces the target for an inner controller, for example angle → rate → actuator. |
| **PID** | Proportional, Integral, Derivative control. P responds to current error, I accumulates error, and D responds to change. INAV's multirotor controller also has a control-derivative term. |
| **P term** | Output proportional to present error. |
| **I term / integrator** | Accumulated error used to remove persistent bias. It needs limits/reset behavior to avoid windup. |
| **D term** | Derivative/damping term. INAV primarily differentiates measured gyro rather than simple setpoint error. |
| **CD** | Control Derivative: a separately filtered derivative of the multirotor rate target. |
| **FF** | Feedforward. Output derived directly from the target rather than waiting for feedback error. |
| **PIFF** | INAV's fixed-wing controller family with proportional, integral, and feedforward terms; the implementation also includes D. |
| **Anti-windup** | Logic that prevents or corrects excessive integrator accumulation when actuator output is saturated or control is restricted. |
| **Back-calculation** | An anti-windup method that uses the difference between requested and limited output to adjust the integrator. |
| **I-term relax / I-term lock** | Fixed/multirotor mechanisms that reduce integration or related terms under large demands or saturation. |
| **TPA** | Throttle PID Attenuation. Gain scaling with throttle; fixed-wing configurations can also use airspeed-based attenuation. |
| **Saturation** | A controller or actuator reaching an output limit, leaving no additional authority in that direction. |
| **Mixer** | Logic that converts roll/pitch/yaw/throttle demands into individual motor or servo commands. |
| **Motor mixer** | Uses per-motor coefficients to combine attitude corrections and throttle while managing available output range. |
| **Servo mixer** | Weighted rules that map stabilized, RC, feature, or programming inputs to servo targets, then scale them to configured endpoints. |
| **Mixer profile** | A configurable motor/servo/platform setup. Multiple profiles support different configurations or VTOL transitions. |
| **Airmode** | Behavior that preserves attitude-control authority at low throttle on multirotors instead of simply stopping motors. |
| **ACRO / rate mode** | Pilot commands primarily request rotation rates rather than absolute angles. |
| **ANGLE mode** | Pilot roll/pitch commands request attitude angles; centered sticks request level flight. |
| **HORIZON mode** | A blend between angle stabilization near centered sticks and rate-like behavior at larger deflection. |
| **ANGLEHOLD** | Holds a captured absolute roll/pitch attitude and lets stick input adjust the held target. |
| **MANUAL mode** | Fixed-wing-family path where transformed pilot commands bypass stabilized `axisPID` as mixer input. It is not raw receiver data and does not stop PID execution. |
| **HEADFREE** | Rotates pilot roll/pitch commands relative to a captured heading so control is less dependent on current nose direction. |
| **Heading hold** | An outer controller that turns heading error into a yaw-rate target. |
| **Turn assistance / coordinated turn** | Fixed-wing logic that coordinates roll, pitch, and yaw rates for a banked turn using attitude and airspeed models. |
| **Hover throttle** | Approximate throttle needed to maintain multirotor hover; used as a reference for vertical navigation control. |
| **Servo autotrim** | Logic that adjusts servo midpoints from stable controller behavior and reduces corresponding integrator demand. |
| **Turtle mode** | A recovery feature for reversing selected motors to flip an overturned multirotor. It changes normal disarmed/mixer handling. |

## Navigation and autonomy

| Term | Definition |
|---|---|
| **Navigation FSM** | The finite-state machine that selects and executes navigation behavior such as holds, RTH, missions, launch, and landing. |
| **FSM** | Finite-State Machine: named states, events, and explicit transitions. INAV state descriptors also map controller/flight-mode flags. |
| **`onEntry` handler** | The function associated with a navigation state. Despite its name, self-timeouts can invoke it repeatedly as periodic state logic. |
| **AltHold** | Altitude Hold: automatic vertical control around a target altitude or climb-rate demand. |
| **PosHold** | Position Hold: automatic horizontal position/velocity control, often combined with AltHold. |
| **Course Hold** | Holds a course. Fixed-wing behavior follows COG; multirotor behavior follows heading. |
| **Cruise** | Course Hold combined with altitude control and platform-specific pilot/navigation behavior. |
| **RTH** | Return To Home. Autonomous return to a stored home or selected safehome with configurable climb, approach, loiter, and landing stages. |
| **Home** | The navigation return location and heading. It is distinct from the GPS coordinate origin. |
| **GPS origin** | The reference LLH used to convert GPS positions into local NEU centimetre coordinates. It can exist before home is valid. |
| **Safehome** | An alternative predefined home location that can be selected for RTH under configured conditions. |
| **Waypoint (WP)** | A mission item with position/altitude, action, and parameters. Not every item is necessarily a simple fly-to coordinate. |
| **WP mission** | A sequenced set of waypoint actions such as fly-to, hold, jump, set POI/heading, land, or RTH. |
| **POI** | Point of Interest. A location toward which heading can be controlled during a mission. |
| **JUMP** | A mission action that transfers execution to another item, with validated target and repeat behavior. |
| **Waypoint radius** | Distance threshold for ordinary waypoint completion. Turn and passage-plane rules can complete a fixed-wing waypoint without entering it. |
| **Fly-by / fly-over / fly-into** | Fixed-wing waypoint turn strategies that determine how the aircraft approaches, turns, and advances past a waypoint. |
| **Carrot / virtual target** | A moving point ahead on the desired fixed-wing track used to calculate guidance error. |
| **Loiter** | Hold around a location. Fixed-wing loiter normally follows a circle; multirotor loiter holds position. |
| **Trackback** | RTH behavior that follows a bounded history of earlier positions before continuing normal return. |
| **Geozone** | A configured circular or polygonal region with altitude limits and actions such as avoid, PosHold, or RTH. |
| **SendTo** | A geozone-driven navigation action that sends the vehicle to a selected target. |
| **Terrain following** | Altitude behavior relative to the surface. Rangefinder-based surface following and SD-map fixed-wing AGL hold are separate implementations. |
| **Auto Speed** | Fixed-wing automatic speed control using ground speed or airspeed to adjust throttle. |
| **Autolaunch** | Fixed-wing launch state machine covering throttle wait, detection, motor spinup, climb, and transition to normal control. |
| **Autoland** | Fixed-wing approach, glide, flare, and landing sequence available under its feature gate. |
| **Emergency landing** | Navigation's controlled fallback when normal autonomous requirements fail or failsafe requests landing. |
| **EST_NONE / EST_USABLE / EST_TRUSTED** | Navigation estimate quality levels. Autonomous functions generally need stronger validity than basic hold functions. |

## Arming, modes, and failsafe

| Term |Definition|
|---|---|
| **ARMED** | A runtime flag allowing normal flight-control policy. It does not prove that motors are physically rotating. |
| **Arming blocker** | A condition represented by `ARMING_DISABLED_*` flags that prevents normal arming, such as calibration, unsafe throttle, invalid navigation, or hardware failure. |
| **Emergency arming** | A limited override of explicitly permitted arming blockers. It is not a universal safety bypass. |
| **Motor-stop** | Policy that can stop or idle motors under low-throttle/navigation/failsafe conditions even while armed. |
| **`motorControlEnable`** | A late-initialization output gate that allows the PID task to call motor-output code. It is separate from `ARMED`. |
| **Flight-mode flags** | Global bits representing active modes such as ANGLE, MANUAL, navigation modes, and failsafe-related behavior. |
| **State flags** | Global bits for platform and runtime conditions such as AIRPLANE, MULTIROTOR, ALTITUDE_CONTROL, and other derived state. |
| **Failsafe** | The policy and state machine that responds to RC-link loss or related unsafe conditions. Procedures include RTH, landing, drop, or holding commands. |
| **RX link / RSSI / LQ** | Measures of receiver connection. RSSI is signal strength; LQ is link quality. Valid RC data also requires protocol/frame and flight-channel checks. |
| **Rearm lockout** | A failsafe state that blocks rearming until the link is valid long enough and the ARM switch is off. |

## Scheduler and embedded architecture

| Term | Definition |
|---|---|
| **Scheduler** | INAV's cooperative mechanism that chooses which registered task runs next. It is not an RTOS with preemptive flight threads. |
| **Cooperative scheduling** | Tasks run to completion and must yield by returning. A long task delays others. |
| **Task** | A `cfTask_t` entry with work function, optional event check, desired period, static priority, and runtime statistics. |
| **Desired period** | Target time between task runs. It is not a guaranteed rate or deadline. |
| **Actual delta** | Measured time since the task's previous execution, available through scheduler timing APIs. |
| **Static priority** | The task's configured priority and aging multiplier. |
| **Dynamic priority** | The current selection priority computed from static priority and waiting age. |
| **Aging** | Increasing a waiting task's dynamic priority as desired periods pass, reducing starvation. |
| **`TASK_PRIORITY_REALTIME`** | A scheduler class that forces overdue GYRO/PID selection preference. It does not preempt running code or enforce deadlines. |
| **`checkFunc`** | Optional task callback that checks whether event-driven work is available. It is polled during scheduler queue scans. |
| **Realtime callbacks** | Short completion/polling functions run after idle scheduler passes and after forced realtime tasks, not independent task-table entries. |
| **ISR** | Interrupt Service Routine. Hardware-triggered code that should do bounded low-level work and defer heavy processing. |
| **DMA** | Direct Memory Access. Hardware that transfers data between memory and peripherals without CPU copying every byte/word. |
| **RTOS** | Real-Time Operating System. INAV's main flight loop does not depend on a general RTOS task per subsystem. |
| **Protothread** | A stackless continuation driven by its containing code. It provides wait/resume structure without an independent stack or scheduler task. |
| **Singleton** | One statically owned instance of a device/service. Many INAV frontends are singletons even when they use replaceable function pointers. |
| **Vtable** | A set of function pointers in a C struct that provides runtime-selectable operations, similar in purpose to C++ virtual methods. |
| **Frontend / backend** | A frontend provides common policy/state; a backend implements a particular chip, protocol, or platform. For RX, byte reception and scheduled protocol work are also split into frontend/backend roles. |
| **Registry** | A collection discovered at runtime, often from linker sections rather than a handwritten central list. |
| **Linker section** | A named memory area in the executable. INAV uses special sections for PG registries/defaults, bus devices, fast code/RAM, and DMA buffers. |
| **`FASTRAM` / `FAST_CODE`** | Placement attributes for performance-sensitive data/code. Their meaning depends on MCU memory and linker layout. |
| **`DMA_RAM`** | Placement attribute for memory intended to be accessible by DMA. Correct placement is functional, not just an optimization. |
| **Static allocation** | Memory reserved at link time rather than allocated dynamically during flight. It improves predictability but requires careful size and ownership review. |

## Protocols and I/O

| Term | Definition |
|---|---|
| **UART** | Universal Asynchronous Receiver/Transmitter. A common serial peripheral used for RC, GPS, telemetry, and other devices. |
| **USB VCP** | USB Virtual COM Port. A USB serial-style interface used for MSP/CLI. |
| **MSP** | MultiWii Serial Protocol. INAV's main configurator and general control/status protocol, with v1/v2 framing and FC command handlers. |
| **CLI** | Command-Line Interface. Text configuration and service commands accessed over serial/MSP infrastructure. |
| **GCS** | Ground Control Station. External software that configures, monitors, or controls the aircraft. |
| **Configurator** | The INAV desktop application used to configure and flash the firmware. It communicates heavily through MSP. |
| **Telemetry** | Flight data sent from the aircraft/controller to a radio, ground station, or display. Some protocols also carry requests or MSP. |
| **SBUS / SBUS2** | Serial RC protocols. SBUS2 adds slot-based telemetry behavior. |
| **CRSF** | Crossfire protocol for RC channels, link data, telemetry, and MSP-related traffic. |
| **FPort / FPort2** | FrSky protocols combining RC and telemetry-related traffic, with FPort2 adding expanded transport behavior. |
| **SmartPort** | FrSky polled sensor/telemetry protocol. INAV can act as a sensor and, with SmartPort master, poll external sensors. |
| **SRXL2** | Spektrum serial protocol that can carry RC, telemetry, bus negotiation, and motor output behavior. |
| **GHST** | ImmersionRC Ghost RC/telemetry protocol. |
| **LTM** | Lightweight Telemetry protocol, a one-way scheduled telemetry format. |
| **MAVLink** | A message-based vehicle communication protocol. INAV supports routing, streams, commands, missions, guided input, RC override, ADSB and MSP tunneling. |
| **DroneCAN** | A CAN-based device protocol built with libcanard in INAV. It supports node management, GNSS, battery, parameters, and MSP-facing services. |
| **CAN / bxCAN / FDCAN** | Controller Area Network and STM32 peripheral variants. The H7 backend in this snapshot configures classic CAN rather than CAN-FD. |
| **DNA** | Dynamic Node Allocation in DroneCAN: assigning node IDs to devices, including persistent UID mappings. |
| **PWM** | Pulse-Width Modulation. A timer-based output commonly used for analog ESC/servo commands. “PWM-like” configuration values are not always physical PWM. |
| **OneShot / Multishot** | Faster analog ESC pulse protocols with shorter pulse ranges than standard PWM. |
| **DShot** | Digital ESC command protocol using packetized values and DMA-capable output. Mixer values are converted into DShot values at the output layer. |
| **OSD** | On-Screen Display. Flight data rendered into a video or external display. |
| **Displayport** | INAV's display abstraction (`displayPort_t`) used by OSD, CMS, and dashboard backends. |
| **CMS** | Configuration Menu System. Declarative menus/editors that can grab a displayport for ground or in-flight use. |
| **Blackbox** | INAV's flight logging subsystem and log format. It writes complete/delta snapshots and events to serial, flash, SD, or SITL file. |
| **I frame / P frame** | Blackbox complete snapshot versus predicted/delta sample. These are logging concepts, not video frames. |
| **FlashFS** | INAV's append-oriented filesystem for onboard NOR/NAND flash, commonly used for Blackbox. |
| **AsyncFatFS** | Cooperative asynchronous FAT filesystem layer used for SD-card logs and terrain data. |
| **FAT** | File Allocation Table filesystem family used on SD cards and emulated for onboard-flash USB export. |
| **USB MSC** | USB Mass Storage Class. INAV enters a dedicated boot service mode to expose SD or flash-backed logs to a host. |
| **PINIO / PINIOBOX** | User-controllable GPIO/timer outputs and their RC-box/programming control paths. |
| **VTX** | Video transmitter. INAV can control band, channel, power, and pit mode through several device protocols. |
| **Pit mode** | A VTX low-power/disabled-transmission state used to reduce interference. |

## Configuration, build, and testing

| Term | Definition |
|---|---|
| **PG / Parameter Group** | A registered persistent C structure, array, or profile with a stable PGN, version, active/copy RAM, and reset behavior. |
| **PGN** | Parameter Group Number. The stable 12-bit persistent identity used in EEPROM records and registry lookup. |
| **PG version** | A four-bit per-group layout version. A mismatch causes that stored group instance to fall back to defaults. |
| **True profile** | A `PG_REGISTER_PROFILE*` object serialized once per profile and selected by rebinding current pointers. |
| **System array profile** | An array stored in one system PG record, with profile selection implemented by the owning module. It is not a true profile PG. |
| **Reset template / reset function** | Static default data or code used to initialize a PG during reset/load fallback. |
| **Active PG RAM** | Authoritative mutable runtime configuration serialized during save. |
| **Copy PG RAM** | Backup storage used by CLI dump/diff, not a transactional save buffer. |
| **`settings.yaml`** | Source metadata for selected CLI/MSP settings. It does not define PG registration or persistence by itself. |
| **Generated setting index** | Build-specific index into generated metadata. It is not a stable persistent identity like PGN. |
| **`USE_*`** | Compile-time feature/hardware macro. Controls source inclusion and target composition. |
| **`FEATURE_*`** | Runtime-configured feature bit. Separate from whether code was compiled and whether hardware was detected. |
| **Feature latch** | A snapshot of enabled features used by `feature()` consumers. Changing the stored mask does not necessarily update it immediately. |
| **CMake** | The build-system generator used to compose firmware, SITL, and native tests. |
| **Target / board target** | A buildable firmware configuration for a board or board variant, defined by a target factory plus hardware files. |
| **Variant** | One executable configuration produced from a board description. One directory can define several variants. |
| **`SKIP_RELEASES`** | Target metadata excluding an image from the release set, not necessarily making it unbuildable. |
| **Linker script** | Memory layout definition for code, data, stack, configuration, registries, and special RAM regions. |
| **Startup code** | Reset/vector code that establishes the C runtime and enters `main()`. |
| **LTO** | Link-Time Optimization. Cross-module optimization enabled for relevant release builds; registry sections still need correct retention. |
| **HEX / BIN / ELF** | Firmware artifact forms. ELF contains symbols/debug/map-related information; HEX/BIN are flashable image formats. |
| **UF2** | USB Flashing Format commonly used for RP2040/RP2350-style bootloaders. |
| **OpenOCD** | Open On-Chip Debugger, used for flashing/debugging supported MCUs. |
| **SVD** | System View Description. XML device register descriptions used by debuggers. |
| **SITL** | Software In The Loop. INAV compiled as a native host process with fake/host hardware and simulator connections; not an ARM instruction emulator. |
| **HITL** | Hardware In The Loop. Simulation involving real flight-controller hardware. Behavior differs from normal SITL output/sensor paths. |
| **Native unit test** | GoogleTest executable compiled for the development host with selected production C files and test definitions. |
| **Integration harness** | Higher-level test that runs multiple modules/processes, such as SITL with MAVLink mission/routing scripts. |
| **Map file** | Linker output showing addresses and sizes. Essential for checking memory fit, sections, stack, and DMA placement. |

## Navigation

**[← Back to index](../README.md)** · Next: [01 · Architecture Overview](01-architecture-overview.md)
