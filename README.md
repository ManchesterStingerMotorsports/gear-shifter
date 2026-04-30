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

- starts the gear safety task only
- reads shift inputs from GPIO
- commands the ESC immediately for the requested shift
- stops the ESC when the encoder reaches the calibrated target, an encoder read fails, or the shift timeout expires
- prints encoder position continuously while idle and during shift movement
- has CAN/ECU safety checks removed for standalone testing

Calibration constants live in:

```text
production_firmware/main/tasks/gear_safety_task/gear_safety_task.cpp
```

Current values:

```cpp
static const float SHIFT_UP_TORQUE = 0.4f;
static const float SHIFT_DOWN_TORQUE = -0.4f;
static const uint16_t BASE_POSITION = 2502;
static const uint16_t SHIFT_UP_STOP_POSITION = 2290;
static const uint16_t SHIFT_DOWN_STOP_POSITION = 2670;
static const uint16_t SHIFT_POSITION_TOLERANCE = 20;
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

## Notes

Generated ESP-IDF outputs such as `build/`, `sdkconfig`, `.bin`, `.elf`, and `.map` are ignored. Each project sets the ESP32-S3 target in its own `CMakeLists.txt`.
