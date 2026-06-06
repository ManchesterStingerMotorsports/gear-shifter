# Gear Shifter

ESP-IDF firmware and bench-test projects for the Manchester Stinger Motorsports electronic gear shifter controller.

The repository now contains three standalone ESP32-S3 projects:

```text
production_firmware/             Production shifter firmware
esc_button_test_firmware/        Bench test: buttons command ESC torque
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
cd C:\Users\james\gear-shifter\production_firmware
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
- checks the encoder before commanding the ESC
- commands the ESC for the requested shift after safety checks pass
- stops the ESC when the encoder reaches the calibrated target, an encoder read fails, or the shift timeout expires
- tracks the internal gear state through neutral, 1st, 2nd, 3rd, 4th, and 5th
- prints encoder position continuously while idle and during shift movement
- has CAN/ECU safety checks removed for standalone testing

Production pin, timing, and calibration constants live in:

```text
production_firmware/main/config.hpp
```

Current values:

```cpp
static constexpr float SHIFT_UP_TORQUE = 0.4f;
static constexpr float SHIFT_DOWN_TORQUE = -0.4f;
static constexpr float SHIFT_1_TO_NEUTRAL_TORQUE = SHIFT_UP_TORQUE;
static constexpr float SHIFT_NEUTRAL_TO_1_TORQUE = SHIFT_DOWN_TORQUE;
static constexpr uint16_t BASE_POSITION = 2502;
static constexpr uint16_t SHIFT_UP_STOP_POSITION = 2290;
static constexpr uint16_t SHIFT_DOWN_STOP_POSITION = 2670;
static constexpr uint16_t SHIFT_1_TO_NEUTRAL_STOP_POSITION = BASE_POSITION;
static constexpr uint16_t SHIFT_NEUTRAL_TO_1_STOP_POSITION = SHIFT_DOWN_STOP_POSITION;
static constexpr uint16_t SHIFT_POSITION_TOLERANCE = 20;
```

## ESC Button Test

Project folder:

```text
esc_button_test_firmware/
```

Behavior:

- hold UP to command positive test torque
- hold DOWN to command negative test torque
- release both buttons to send neutral torque

Constants live in:

```text
esc_button_test_firmware/main/main.cpp
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

## Pin Map

| Function | ESP32-S3 GPIO |
| --- | ---: |
| ESC PWM | GPIO 9 |
| Encoder CS | GPIO 10 |
| Encoder MOSI | GPIO 11 |
| Encoder SCLK | GPIO 12 |
| Encoder MISO | GPIO 13 |
| Shift up input | GPIO 45 |
| Shift down input | GPIO 47 |
| Shift neutral input | GPIO 48 |
