# ESP32 Flight Controller (RTH)

This project is a basic flight controller for RC planes using an ESP32. It can return to home (RTH) when a switch is flipped.

## Hardware Used
- ESP32
- NEO-M8 GPS
- 3x servo outputs (throttle, rudder, elevator)
- 1x RC channel input for RTH switch (PWM/analog)

## Features
- Locks GPS position at startup as home
- Manual mode: flies forward to a set waypoint
- RTH mode: flies back to launch point
- Controls rudder using compass heading
- Fixed throttle and elevator for simplicity

## Wiring (example)
| Component      | ESP32 Pin |
|----------------|-----------|
| GPS TX         | GPIO 16   |
| GPS RX         | GPIO 17   |
| Compass SDA    | GPIO 21   |
| Compass SCL    | GPIO 22   |
| RC RTH Input   | GPIO 34   |

## How It Works
1. Power on → waits for GPS fix.
2. Saves current position as home.
3. Flies toward a fixed waypoint.
4. When RTH switch is activated, turns around and flies back home.
5. Uses compass to steer toward target heading.

## Notes
- Compass must be calibrated manually.
- No altitude hold yet.
- No aileron control, just rudder.
- For stable planes only.

## To Do
- Add PID for pitch control
- Add GPS-based landing logic


