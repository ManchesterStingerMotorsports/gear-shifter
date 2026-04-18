# Gear Shifter

Firmware for the Manchester Stinger Motorsports electronic gear shifter controller.

This project runs on ESP-IDF and controls a motor-driven shift mechanism through a copperhead 10 ESC from castle creations. The firmware listens for shift inputs, checks the requested shift against ECU/CAN gear state, drives the actuator, and confirms completion using an AMT20 absolute encoder from Samesky.

## Design intent

The main design goal is that a shift request should either complete in a controlled way or stop cleanly.

Shift button interrupts do not perform the shift directly. They only latch the first request, disable further shift input interrupts, and wake the high-priority gear safety task. The gear safety task then owns the actuator until either:

- the encoder reaches the expected shifted position
- the shift times out

Additional shift inputs are ignored during this window.

## Runtime tasks

`app_main()` starts two application tasks:

- `can_task`: lower-priority task that keeps the latest ECU gear snapshot fresh from CAN.
- `gear_safety_task`: highest-priority task that owns shift validation, ESC actuation, timeout handling, and encoder confirmation.

The control-path tasks are pinned to ESP32-S3 core 1. This is deliberate: during the active shift window, the gear safety task should win by priority instead of allowing lower-priority CAN work to run on the other core.

If either task fails to start, the firmware disables interrupts and parks.

## Core model

Core 1 is treated as the control core:

- `gear_safety_task`
- `can_task`
- `MSM_CAN_RX`
- `MSM_CAN_TX`

`gear_safety_task` runs at the highest application priority. The CAN-facing tasks run at low priority on the same core. This means that while the ESC is actively being commanded during a shift, task-level CAN processing should not run until the gear safety task exits the shift window.

This does not disable hardware interrupts. TWAI/CAN ISR callbacks may still fire and queue received frames, but the RX/TX worker tasks are pinned to the control core and remain lower priority than the gear safety task.

## Shift safety checks

Before commanding the ESC, the gear safety task checks:

- the requested shift is allowed from the current internal gear count
- the ECU gear packet is fresh
- the ECU gear agrees with the internal gear count
- the AMT20 encoder can be read successfully

If any check fails, the ESC is commanded to zero torque and the shift is rejected.


## Encoder

The shifter uses an AMT20 absolute encoder over SPI. The encoder driver is in:

- `main/encoder/AMT20.hpp`
- `main/encoder/AMT20.cpp`

The AMT20 driver implements the datasheet read-position sequence:

- send `0x10` to request position
- poll with `0x00` while the encoder returns `0xA5`
- wait for the encoder to echo `0x10`
- read MSB and LSB
- combine the lower 12 bits into a position from `0` to `4095`


## Calibration still required

Final encoder stop positions need to be measured on the assembled mechanism.

These values live in `main/tasks/gear_safety_task/gear_safety_task.cpp`:

```cpp
static const uint16_t SHIFT_UP_STOP_POSITION = 0;
static const uint16_t SHIFT_DOWN_STOP_POSITION = 0;
static const uint16_t SHIFT_NEUTRAL_FROM_1_STOP_POSITION = 0;
static const uint16_t SHIFT_NEUTRAL_FROM_2_STOP_POSITION = 0;
static const uint16_t SHIFT_POSITION_TOLERANCE = 20;
```

The torque values and timeout are also currently calibration values:

```cpp
static const int64_t SHIFT_TIMEOUT_US = 200000;

static const float SHIFT_UP_TORQUE = 0.25f;
static const float SHIFT_DOWN_TORQUE = -0.25f;
static const float SHIFT_NEUTRAL_FROM_1_TORQUE = 0.25f;
static const float SHIFT_NEUTRAL_FROM_2_TORQUE = -0.25f;
```

## Current pin map

The firmware currently uses:

| Function | ESP32-S3 GPIO |
| --- | ---: |
| ESC PWM | GPIO 9 |
| Encoder CS | GPIO 10 |
| Encoder MOSI | GPIO 11 |
| Encoder SCLK | GPIO 12 |
| Encoder MISO | GPIO 13 |
| CAN TX | GPIO 2 |
| CAN RX | GPIO 1 |
| Shift up input | GPIO 45 |
| Shift down input | GPIO 47 |
| Shift neutral input | GPIO 48 |

## CAN inputs

This project is currently using a slightly modified version of `MSM_CAN`.

The wrapper owns the ESP-IDF TWAI node and creates two internal worker tasks:

- `MSM_CAN_RX`: receives frames from the TWAI ISR queue and updates subscription state.
- `MSM_CAN_TX`: handles one-shot and scheduled transmit requests.

In this firmware, those worker tasks are pinned to the same control core as the gear safety task so that they cannot run as task-level work during the active shift loop.

`can_task` currently subscribes to:

- `0x470`: ECU gear packet. The decoded gear is read from byte 7.
- `0x360`: RPM packet, reserved for future downshift protection.

Valid ECU gear values are expected to be `0` to `5`, matching:

```cpp
GEAR_N = 0,
GEAR_1,
GEAR_2,
GEAR_3,
GEAR_4,
GEAR_5
```

## Repository layout

```text
main/
  encoder/                  AMT20 absolute encoder driver
  esc/                      ESC PWM wrapper
  MSM_CAN/                  CAN helper layer
  tasks/can_task/           ECU gear snapshot update task
  tasks/gear_safety_task/   shift request handling and safety logic
Hardware/                   KiCad hardware files
Archive Python Code/        earlier prototype scripts
ESC Settings/               ESC configuration files
```

