# 11 · Extending INAV

Source-oriented contributor recipes for adding tasks, drivers, settings, protocols, navigation behavior, boards, and MCU ports without breaking timing, persistence, or hardware assumptions.

**[← Back to index](../README.md)** · Previous: [10 · Build, Targets & Testing](10-build-targets-and-testing.md) · Glossary: [00 · Glossary](00-glossary.md)

---

## Start with the execution and ownership model

Before adding code, answer five questions:

1. **When will it run?** Interrupt, GYRO/PID, ordinary scheduled task, realtime callback, special service mode, or host-only tooling?
2. **Who owns the state?** Existing global frontend, a new static module object, a parameter group, or transient protocol storage?
3. **What are the units and frame?** ADC counts, deg/s, rad/s, cm/s², cm, decidegrees, centidegrees, normalized command, or PWM-like output?
4. **How is it composed?** `USE_*` compile gate, target descriptor, runtime `FEATURE_*`, detection state, and task enablement?
5. **How will it be verified?** Native unit test, target build, SITL/integration, bench hardware, and—where flight-critical—controlled flight validation?

Use the smallest appropriate interface. Do not introduce a new global scheduler task, persistent setting, or hardware abstraction when an existing module hook already fits.

## Where to change what

| Goal | Primary locations | Read first |
|---|---|---|
| Add periodic work | `fc/fc_tasks.c`, `scheduler/scheduler.h`, owning module | [Boot & Scheduler](03-boot-and-scheduler.md) |
| Add IMU/sensor hardware | `drivers/<family>/`, `sensors/`, target bus descriptors | [Platforms & Drivers](02-platforms-and-drivers.md), [Sensors](04-sensors-and-estimation.md) |
| Change stabilization | `flight/pid.c`, `flight/mixer.c`, `flight/servos.c`, `fc/fc_core.c` | [Flight Control](05-flight-control.md) |
| Change autonomy | `navigation/navigation.c`, estimator and platform-controller files | [Navigation](06-navigation-and-missions.md) |
| Add RC/serial protocol | `rx/`, `io/serial.*`, `drivers/serial.*` | [Communications](07-communications.md) |
| Add MSP command | `msp/msp_protocol*`, `fc/fc_msp.c`, transport-specific size checks | [MSP architecture](07-communications.md#msp-protocol-architecture) |
| Add telemetry/MAVLink data | `telemetry/`, `mavlink/` | [Communications](07-communications.md) |
| Add OSD/Blackbox field | `io/osd.*`, `blackbox/` | [OSD, Logging & Programming](08-osd-logging-and-programming.md) |
| Add persistent setting | Config struct/registration, `parameter_group_ids.h`, `fc/settings.yaml`, reset code | [Configuration](09-configuration-and-persistence.md) |
| Add a board | `src/main/target/BOARD/`, target factory | [Build & Targets](10-build-targets-and-testing.md), [board recipe](#add-a-board-on-an-existing-mcu-family) |
| Add an MCU family | `cmake/`, `platform.h`, drivers, startup, linker, target factory | [MCU-port recipe](#port-a-new-mcu-family) |

## Add a scheduled task

Use a scheduled task for recurring cooperative work that does not belong inside an existing task.

### Recipe

1. **Choose the owner and entry function.** Keep algorithm/state in the owning module; `fc_tasks.c` should contain wiring, not the whole implementation.
2. **Add a task ID under appropriate compile guards** in `scheduler/scheduler.h`.
3. **Add a `cfTasks[]` entry** in `fc/fc_tasks.c` with name, optional `checkFunc`, `taskFunc`, desired period, and static priority.
4. **Enable it explicitly** in `fcTasksInit()` only when compilation, configuration, detection, and resource preconditions are satisfied.
5. **Keep function-level guards** for state that can change after queue enablement.
6. **Use actual elapsed time** for integration, differentiation, expiry, or filters. `getTaskDeltaTime(TASK_SELF)` is available in the task function.
7. **Bound every invocation.** Do not wait for hardware, flush unlimited data, or run an unbounded loop.
8. **Add tests and inspect timing on target.** A desired period is not a deadline guarantee.

### Priority selection

- Do not use `TASK_PRIORITY_REALTIME` for ordinary work. It is reserved for the flight-critical GYRO/PID model and changes scheduler selection semantics.
- Use the lowest priority that meets functional needs.
- Use an event `checkFunc` when external availability should signal work; remember that checks are polled during queue scans.
- Use realtime callbacks only for short self-scheduling completion work that belongs with the existing callback model.

### Pitfalls

- A long task delays all later tasks.
- A high desired rate does not imply high priority.
- Missed periods do not create catch-up calls.
- Queue enablement does not make the task run concurrently with initialization.
- Do not use `TASK_SELF` in a check function or arbitrary callback without a current task.

## Add a sensor or device driver

The normal pattern is **detect → bind callbacks → initialize → normalize in frontend**.

### Recipe

1. **Copy the closest driver with the same bus and device family.** Match register model and conversion behavior, not just the chip name.
2. **Add the correct compile-time feature macro.** Confirm the macro in the driver and CMake/common composition rather than guessing from the filename.
3. **Describe hardware in the target.** Use common hardware descriptors where sufficient; otherwise emit explicit `BUSDEV_REGISTER_*` entries and disable the corresponding common set.
4. **Implement detection.** Open the `busDevice_t`, verify identity, handle failure without leaving partial state, and distinguish multiple tags where supported.
5. **Fill the device structure.** Supply init/read/update callbacks, scale, alignment and conversion delays required by that frontend.
6. **Integrate detection into the sensor frontend** under its feature guards.
7. **Preserve frontend policy.** Calibration, board alignment, common units, filtering, health, and scheduler cadence should remain consistent with sibling drivers.
8. **Validate units at the public boundary.** For example, gyro is deg/s after scaling, accel crossing into IMU is cm/s², pressure is Pa, and navigation distances are cm.
9. **Test absent, wrong-ID, bus-error, timeout and valid-device cases.** Then validate actual orientation, scale and conversion timing on hardware.

### Do not assume multi-instance support

Static scratchpads, file-static calibration, filters and driver state can make a nominally vtable-based driver singleton. Adding a second device requires an explicit ownership review.

### Health is a separate design decision

Detection does not imply freshness or plausibility. Decide whether a driver should update:

- communication timestamp;
- last valid sample;
- no-new-data/out-of-range status;
- hardware-health state;
- measurement-quality/validity flags.

Do not hide all of them behind one Boolean.

## Add or change a persistent setting

First read upstream's setting guidance and decide whether a user setting is warranted. Derived aircraft state and internal algorithm details often should not become settings.

### Recipe

1. **Define or modify the C configuration structure** in the owning module. Use fixed-width storage types and avoid pointers/runtime-only objects.
2. **Choose the ownership model:** scalar system, system array, or true profile.
3. **Allocate a unique stable PGN** in `config/parameter_group_ids.h`.
4. **Register exactly once** with the appropriate `PG_REGISTER*` macro.
5. **Provide reset defaults** through a supported reset template or reset function. Arrays must initialize every element.
6. **Increment the PG version for structure-layout changes.** This includes field addition/removal/reorder, type or array-size changes, packing/alignment changes, and condition-dependent layout changes.
7. **Expose selected fields in `fc/settings.yaml`** only if generic CLI/MSP access is needed. Supply field, default, range/table, section and compile conditions.
8. **Consume generated `SETTING_*_DEFAULT` macros manually** in reset code where appropriate.
9. **Add validation** for cross-field constraints not covered by scalar min/max checks.
10. **Test persistence behavior:** missing record, old version, reset defaults, target overlays, profile selection, CLI dump/diff, MSP set versus explicit save, and reboot.
11. **Regenerate upstream user-facing CLI documentation** with the project's documented generator when contributing the change upstream.

### Compatibility warning

The serializer writes raw C bytes. Equal-version/different-size loading copies a prefix, so a casual append can expose old padding as a new field value. Versioned reset is usually safer than accidental compatibility.

## Add an MSP command

1. Allocate the command in the correct MSP protocol header/namespace and document direction/payload/version.
2. Add the handler in `fc/fc_msp.c` under the appropriate feature guard.
3. Read payloads with safe stream operations and accept intentional prefix compatibility only where appropriate.
4. Calculate the exact maximum response size. MSP output buffers are bounded and not every write is capacity-checked.
5. Mutate the same authoritative state used by runtime/CLI rather than creating a parallel model.
6. Add armed-state, post-process and persistence guards for dangerous operations.
7. If the action is reboot/passthrough/reinitialization, preserve the after-TX-drain post-process boundary.
8. Test v1/v2 transports as applicable, telemetry chunking if supported, malformed/truncated input, maximum response, and armed rejection.

Do not assume that adding a handler automatically makes every transport safe for large responses or post-process actions; the MAVLink tunnel has explicit restrictions.

## Add receiver or telemetry support

### Receiver protocol

- Implement byte/frame assembly in the protocol frontend with bounded buffers and CRC/length validation.
- Fill `rxRuntimeConfig_t` with frame-status, raw-read and optional deferred-processing callbacks.
- Return accurate frame flags for complete, failsafe, dropped and deferred states.
- Normalize into the common RC domain and let `rx.c` own channel mapping, validity holds and failsafe evidence.
- Configure serial sharing/inversion/half-duplex behavior through the existing serial-function model.
- Test timeout, malformed frames, link recovery, channel count limits, failsafe bits, and deferred-work timing.

### Telemetry protocol

- Use the common telemetry state check/handler pattern unless protocol timing requires a dedicated task or RX-owned transmit window.
- Check TX room and respect poll/response deadlines.
- Read shared state using its documented units and validity.
- Avoid unbounded schedule tables or sending unavailable values without defined fallback.
- Remember that rescheduling `TASK_TELEMETRY` changes the shared dispatcher for all protocols.

### MAVLink

- Preserve per-port parser/TX sequence and active-port context.
- Route before local dispatch where required and avoid ingress echo.
- Use nonblocking/drop semantics under TX pressure.
- Keep mission transfer single-owner, bounded-retry and armed guards.
- Set the correct active port/send mask before stream helpers; the API is not reentrant.

## Add OSD, Blackbox, or programming behavior

### OSD element

- Read public state and validity rather than duplicating estimation.
- Add layout/configuration metadata under the appropriate feature guards.
- Keep formatting bounded by display dimensions and buffer sizes.
- Respect display transfer/CMS ownership and incremental refresh scheduling.

### Blackbox field

- Add the field definition and encoder change together.
- Preserve exact declaration/write order, predictor, signedness, unit and conditional inclusion.
- Cache inclusion decisions at log start.
- Verify with the actual Blackbox decoder, not only by inspecting bytes.

### Logic-condition operation

- Define operand types, units, activator behavior and latch semantics.
- Use the shared operand resolver and bounded global-variable APIs.
- Document index-order behavior and 10 Hz update latency.
- Route overrides through existing query points rather than creating a hidden second control path.
- Test forward/backward condition references, activation/removal delay, arming/disarming and failsafe interactions.

## Change navigation or mission behavior

Navigation changes have wide safety impact because the same FSM handles pilot modes, failsafe, RTH, missions, launch, landing and VTOL.

1. Identify whether the change belongs in sensor topic processing, estimator, FSM selection, state handler, or platform controller.
2. Preserve NEU/NED, absolute/AGL, decidegree/centidegree and pitch-sign conversions.
3. Use `EST_USABLE` versus `EST_TRUSTED` deliberately.
4. If adding a state, assign a stable persistent Blackbox ID separately from the enum.
5. Remember that self-timeouts can call `onEntry` repeatedly; implement both initialization and periodic behavior intentionally.
6. Keep mode activation requirements separate from active-state failure/recovery policy.
7. Preserve `rcCommand` as the normal controller output unless deliberately redesigning the downstream interface.
8. Test position/heading/altitude loss, GPS recovery, home substitution, disarm/rearm, failsafe, long update gaps and platform differences.

For mission actions, validate upload parameters and arming constraints, define reached semantics, and test interruption/restart behavior. Do not assume waypoint reached always means radius entry.

## Add a board on an existing MCU family

### 1. Select the closest reference

Match MCU, flash size, sensor count/bus layout, storage, output resources, and ideally manufacturer. Do not copy only pin names without checking timer/DMA and memory differences.

### 2. Add a discoverable target directory

Create `src/main/target/BOARD/` with:

- `CMakeLists.txt` invoking the exact target factory;
- `target.h` with the complete hardware/feature contract;
- `target.c` with timer mappings and optional bus descriptors;
- optional `config.c` and hardware setup/revision files.

A `target.h`/`target.c` pair without `CMakeLists.txt` is not discovered.

### 3. Define the hardware contract

Check at minimum:

- unique board identifier and product string;
- MCU/flash/HSE selection through the correct factory arguments;
- SPI/I²C instances, chip selects, addresses, tags and sensor alignment;
- UARTs, VCP, softserial and serial-port count;
- ADC channels and function assignments;
- `TARGET_IO_PORTx` masks for every referenced pin;
- storage, LED, beeper, PINIO, DShot and output-count limits;
- USB and bootloader requirements.

### 4. Build timer/DMA mappings deliberately

- Use `DEF_TIM` and appropriate usage flags.
- Cross-check every timer/channel/DMA option against the MCU reference manual.
- Check conflicts with UART, LED, beeper, ADC, softserial and output timer groups.
- Ensure output allocation covers every mixer profile.

Ownership metadata does not replace this review.

### 5. Choose bus descriptors and board hooks

Use common descriptors for standard layouts. Use explicit descriptors only when necessary and prevent duplicate common registrations. Add target defaults/validation for serial functions, looptime, PINIO, preboot power/mux and revision handling as needed.

### 6. Verify incrementally

1. Configure/build only the intended artifact after rerunning CMake discovery.
2. Inspect ELF/map size, stack, FASTRAM, DMA and registry sections.
3. Validate USB/MSP/CLI.
4. Validate sensor identity, scale and alignment.
5. Validate every UART, output and timer/DMA path.
6. Validate storage persistence across reboot, Blackbox, reset and bootloader paths.
7. Test release/LTO as well as debug composition.
8. Follow upstream hardware-sample/new-target policy before expecting official release inclusion.

Native tests and SITL do not replace these checks.

## Port a new MCU family

A new MCU port is a platform architecture project, not a board-definition task.

### Required build/platform work

1. Add a CMake platform module with toolchain gating, CPU flags, source selection/exclusions, target factory, startup, linker, artifacts and debug metadata.
2. Extend `platform.h` while preserving common-header → board-header → sanity → post-header order.
3. Select or exclude vendor SDK sources deliberately; do not force incompatible register types into shared backends.
4. Add at least one reference target and a reproducible verification configuration.

### Required runtime contracts

Implement and test:

- clocks, system init, reset/bootloader and persistent reset state;
- microsecond/millisecond time and delays;
- GPIO/RCC and resource metadata;
- DMA, timers, interrupt indices and output support;
- SPI/I²C and bus-device behavior;
- UART/VCP and serial vtables;
- ADC and required EXTI paths;
- PWM/DShot/LED and storage/config streamer;
- DSP operations, cache/MPU and memory coherence;
- startup/vector code and all linker-created registries/config symbols.

Do not use silent stubs for safety-critical behavior on a real flight target. Host/SITL stubs are appropriate only for simulation.

### Linker and memory review

Preserve `KEEP()` and start/end symbols for PG/bus registries, configuration addresses, data initializers, stack and fast/DMA regions. Validate that each DMA engine can access its buffers and that cache maintenance matches the platform.

### Verification progression

- native tests for HAL-free logic;
- host/SITL integration where meaningful;
- target compilation in debug and release/LTO;
- bench validation of clocks, buses, interrupts, USB, storage and outputs;
- timing/load measurement;
- controlled flight validation only after the underlying platform is proven.

## Contributor review checklist

### Architecture

- [ ] The change uses an existing ownership boundary or documents a new one.
- [ ] Execution context and worst-case blocking are understood.
- [ ] Compile-time, runtime-feature, detection and task-enablement guards are distinct.
- [ ] State units, coordinate frames and validity/freshness are explicit.
- [ ] Static allocation, stack use, fast/DMA memory placement and reentrancy are reviewed.

### Persistence and protocols

- [ ] PG structure/version/PGN/reset behavior is correct.
- [ ] YAML, reset defaults, validation and generated metadata are synchronized.
- [ ] Protocol input is bounded and malformed data is handled.
- [ ] Output size/backpressure and post-process timing are handled.
- [ ] Setter versus save/reinitialization semantics are clear.

### Safety and timing

- [ ] No unbounded work was added to flight tasks or realtime callbacks.
- [ ] Actual delta time is used where required.
- [ ] Arming, motor-stop, output-enable and failsafe behavior remains distinct.
- [ ] Sensor loss, stale data and recovery paths are defined.
- [ ] Changes to command call order were reviewed against every writer/consumer.

### Verification

- [ ] Relevant native tests were added/updated.
- [ ] Required target combinations compile, including release/LTO where relevant.
- [ ] SITL/integration coverage is used for cross-module behavior.
- [ ] Hardware-specific claims have hardware evidence.
- [ ] Documentation and generated user docs are updated through the upstream workflow.

## High-value pitfalls to avoid

1. Do not treat scheduler “realtime” as preemption or a deadline guarantee.
2. Do not put long flash, serial, SD or polling operations into the PID/gyro path.
3. Do not add a setting without deciding PG version, reset and migration behavior.
4. Do not assume `featureSet()` immediately changes latched `feature()` consumers.
5. Do not switch an initialized driver/output backend by editing only its configuration field.
6. Do not interpret sensor presence as data validity.
7. Do not mix NEU/NED or angle/time units at module boundaries.
8. Do not bypass the shared `rcCommand` pipeline without reviewing navigation, failsafe, power limiting and mixers.
9. Do not change Blackbox declarations without matching encoders and decoder verification.
10. Do not rely on host success for DMA, cache, electrical, timing or flight correctness.

## Navigation

**[← Back to index](../README.md)** · Previous: [10 · Build, Targets & Testing](10-build-targets-and-testing.md) · Glossary: [00 · Glossary](00-glossary.md)
