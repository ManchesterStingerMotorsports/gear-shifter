# Gear Shifter

ESP-IDF firmware and bench-test projects for the Manchester Stinger Motorsports electronic gear shifter controller.

The repository now contains four standalone ESP32-S3 projects:

```text
gear-shifter/
  production_firmware/             Production shifter firmware
  can_test_firmware/                Bench test: CAN/TWAI test scaffold
  button_test_firmware/            Bench test: buttons, LEDs, and low ESC torque
  encoder_position_test_firmware/  Bench test: print AMT20 encoder position
  Hardware/                        KiCad hardware files
  ESC Settings/                    ESC configuration files
  Archive Python Code/             Earlier prototype scripts
```

There is intentionally no ESP-IDF project at the repository root. Open/build one of the firmware folders directly.

## Production Firmware

Project folder:

```text
production_firmware/
```

Build:

```powershell
cd C:\Users\james\FS\gear-shifter\production_firmware
idf.py build
```

Flash:

```powershell
idf.py -p COMx flash monitor
```

Current behavior:

- starts the shifter task and a placeholder CAN task
- reads shift inputs from GPIO
- rejects shifts that would leave the valid gear range
- uses the latest CAN task output during pre-check: neutral position sensor first, then ECU gear when the car is moving
- checks the encoder before commanding the ESC
- commands the ESC for the requested shift after safety checks pass
- stops the ESC when the encoder reaches the calibrated target, an encoder read fails, or the shift timeout expires
- tracks the internal gear state through neutral, 1st, 2nd, 3rd, 4th, and 5th
- prints encoder position continuously while idle and during shift movement

Production pin, timing, and calibration constants live in:

```text
production_firmware/main/config.hpp
```

Production source layout:

```text
production_firmware/main/
  config.hpp
  main.cpp
  encoder/             AMT20 encoder driver
  esc/                 ESC PWM driver
  interrupts/          GPIO ISR/input latch code
  tasks/
    can_task/          CAN output placeholder/status
    shifter_task/      Main shifter control logic
```

Current values:

```cpp
static constexpr bool STANDALONE_TESTING = true;
static constexpr int8_t STANDALONE_INITIAL_GEAR = 2;
static constexpr float SHIFT_UP_TORQUE = 0.4f;
static constexpr float SHIFT_DOWN_TORQUE = -0.4f;
static constexpr float SHIFT_1_TO_NEUTRAL_TORQUE = SHIFT_UP_TORQUE;
static constexpr float SHIFT_NEUTRAL_TO_1_TORQUE = SHIFT_DOWN_TORQUE;
static constexpr uint16_t BASE_POSITION = 2840;
static constexpr uint16_t SHIFT_UP_STOP_POSITION = 2639;
static constexpr uint16_t SHIFT_DOWN_STOP_POSITION = 3039;
static constexpr uint16_t SHIFT_1_TO_NEUTRAL_STOP_POSITION = BASE_POSITION;
static constexpr uint16_t SHIFT_NEUTRAL_TO_1_STOP_POSITION = SHIFT_DOWN_STOP_POSITION;
static constexpr uint16_t SHIFT_POSITION_TOLERANCE = 20;
```

## Button And ESC Test

Project folder:

```text
button_test_firmware/
```

Behavior:

- reads UP, DOWN, and NEUTRAL as active-low inputs
- high impedance reads as released using the internal pull-up
- GND reads as pressed
- prints the button state to serial whenever it changes
- mirrors the debounced button states on the UP, DOWN, and NEUTRAL LEDs
- commands low positive ESC torque while UP is pressed
- commands low negative ESC torque while DOWN is pressed
- sends neutral ESC torque when neither or both shift directions are pressed

Constants live in:

```text
button_test_firmware/main/main.cpp
```

## Encoder Position Test

Project folder:

```text
encoder_position_test_firmware/
```

Behavior:

- repeatedly reads the AMT20 encoder over SPI
- prints the raw encoder position from `0` to `4095`

Constants live in:

```text
encoder_position_test_firmware/main/main.cpp
```

## CAN Test

Project folder:

```text
can_test_firmware/
```

Behavior:

- intentionally blank scaffold for CAN/TWAI bench testing
- builds with the same reusable `MSM_CAN` helper used by production firmware

Constants and test logic can be added in:

```text
can_test_firmware/main/main.cpp
```

## Pin Map

| Function | ESP32-S3 GPIO |
| --- | ---: |
| ESC PWM | GPIO 9 |
| Encoder CS | GPIO 10 |
| Encoder MOSI | GPIO 11 |
| Encoder SCLK | GPIO 12 |
| Encoder MISO | GPIO 13 |
| Shift up input | GPIO 48 |
| Shift down input | GPIO 45 |
| Shift neutral input | GPIO 47 |
| Shift up LED | GPIO 7 |
| Shift down LED | GPIO 6 |
| Shift neutral LED | GPIO 5 |
| CAN RX | GPIO 2 |
| CAN TX | GPIO 1 |
