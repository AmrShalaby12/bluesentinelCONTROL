# BlueSentinel

BlueSentinel is an `ESP32`-based control system for a small surface vehicle that uses four thrusters mounted at `45 degrees` in an `X-drive` layout.

It includes:

- `ESP32` main controller
- `LCD I2C` display
- `MPU6050` IMU
- `DS18B20` water temperature sensor
- `Analog pH` sensor
- Browser dashboard over Wi-Fi
- Laptop joystick control over the same network

## Overview

The project is built around a four-thruster `X-drive` mixer:

- Forward/backward: all thrusters contribute together
- Left/right strafing: opposite thrust components are mixed across the vehicle
- Rotation: left and right sides counter each other

Motor mixing used in the firmware:

```text
FL = surge + strafe + yaw
FR = surge - strafe - yaw
RL = surge - strafe + yaw
RR = surge + strafe - yaw
```

If any motor direction is reversed, do not change the mixer math. Just change the corresponding `INVERT_*` flag in [include/config.h](include/config.h).

## Important Assumptions

This firmware assumes each motor is connected to one of the following:

- A `reversible ESC` for `BLDC` thrusters
- A motor driver that accepts `RC PWM` style control for both forward and reverse

This is important because a `45 degree` four-thruster surface vehicle needs positive and negative thrust to move correctly in all directions.

## Hardware Wiring

### ESP32 DevKit 33-pin connections

#### I2C devices

| Device | ESP32 Pin | Notes |
|---|---:|---|
| `LCD SDA` | `GPIO21` | I2C bus |
| `LCD SCL` | `GPIO22` | I2C bus |
| `MPU6050 SDA` | `GPIO21` | Shared I2C bus |
| `MPU6050 SCL` | `GPIO22` | Shared I2C bus |

#### Sensors

| Device | ESP32 Pin | Notes |
|---|---:|---|
| `DS18B20 DATA` | `GPIO13` | Use a `4.7k` pull-up resistor to `3.3V` |
| `pH analog OUT` | `GPIO34` | `ADC1` input, safe to use while Wi-Fi is enabled |

#### Motor control outputs

| Thruster | ESP32 Pin |
|---|---:|
| Front Left | `GPIO25` |
| Front Right | `GPIO26` |
| Rear Left | `GPIO27` |
| Rear Right | `GPIO14` |

### Power notes

- Do not power the motors from the `ESP32`
- Use a common `GND` between the `ESP32` and all `ESCs` or motor drivers
- Verify your sensor supply voltage requirements before wiring
- Some pH boards use `5V` supply, but the signal going to the `ESP32` must still be safe for `3.3V` logic

## Thruster Layout

Top view:

```text
        Front

   FL             FR
    \             /
     \           /
      \         /
      /         \
     /           \
    /             \
   RL             RR

        Rear
```

Each thruster is mounted at `45 degrees` relative to the vehicle body.

## Project Files

- Main firmware: [src/main.cpp](src/main.cpp)
- Pin map and tuning: [include/config.h](include/config.h)
- Example Wi-Fi config: [include/secrets.example.h](include/secrets.example.h)
- Web dashboard: [data/index.html](data/index.html)
- PlatformIO config: [platformio.ini](platformio.ini)

## Wi-Fi Setup

Copy:

```text
include/secrets.example.h -> include/secrets.h
```

Then edit `include/secrets.h` and set your actual Wi-Fi credentials.

If no Wi-Fi credentials are provided, or station mode fails, the `ESP32` starts its own access point:

- SSID: `BlueSentinel-Setup`
- Password: `BlueWater2026`

In AP mode, open:

```text
http://192.168.4.1
```

If the board connects successfully to your router:

- Open the IP address shown on the LCD
- Or try `http://BlueSentinel.local`

## Joystick Control From Laptop

1. Connect the laptop to the same Wi-Fi network as the `ESP32`
2. Connect the joystick/gamepad to the laptop
3. Open the dashboard in a browser using the board IP
4. Press any gamepad button once if the browser does not detect it immediately
5. Press `Enable Motion` on the page

Default joystick mapping:

- `Left Stick X`: strafe left/right
- `Left Stick Y`: forward/backward
- `Right Stick X`: yaw / rotate

The browser reads the joystick locally and sends commands to the `ESP32` over `WebSocket`.

## Safety Features

- If joystick commands stop for more than `600 ms`, all motors stop automatically
- If the web page closes or the socket disconnects, motion is disabled
- On startup, the firmware sends `neutral` to all ESCs for `3 seconds` to allow arming

## LCD Display

The LCD rotates between two screens:

- Temperature, pH, roll, and pitch
- Current IP address and control state

## Calibration

### pH sensor

Tune these values in [include/config.h](include/config.h):

- `PH_NEUTRAL_VOLTAGE`
- `PH_VOLTS_PER_PH`

Start with the default values, then calibrate using real buffer solutions such as `pH 7`, `pH 4`, or `pH 10`.

### Motor direction

If forward motion causes rotation, or left/right movement is reversed, only change:

- `INVERT_FRONT_LEFT`
- `INVERT_FRONT_RIGHT`
- `INVERT_REAR_LEFT`
- `INVERT_REAR_RIGHT`

## Uploading the Firmware

### Using VS Code + PlatformIO

1. Install the `PlatformIO` extension
2. Open this project folder
3. Run `Build`
4. Run `Upload`
5. Run `Upload Filesystem Image`

### Using terminal

```bash
pio run
pio run --target upload
pio run --target uploadfs
```

The `uploadfs` step is required because the control dashboard is stored in `data/`.

## Libraries Used

Configured in [platformio.ini](platformio.ini):

- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `ArduinoJson`
- `WebSockets`
- `ESP32Servo`
- `DallasTemperature`
- `OneWire`
- `LiquidCrystal_I2C`

## Practical Notes

- `GPIO34` is input-only, which makes it a good pH analog input
- Avoid using `ADC2` for analog sensors while Wi-Fi is active on `ESP32`
- If the LCD does not respond, try changing the I2C address from `0x27` to `0x3F`
- If your joystick uses a different axis layout, update `AXIS_YAW`, `AXIS_STRAFE`, or `AXIS_SURGE` in [include/config.h](include/config.h)
- If your IMU is not `MPU6050`, the driver section in the firmware must be changed

## First Bench Test

1. Power the board without testing in water yet
2. Confirm the dashboard shows temperature, pH, roll, and pitch data
3. Enable motion and move the joystick slightly
4. Watch the per-motor output values on the dashboard
5. Verify motor directions before moving to a safe water test
