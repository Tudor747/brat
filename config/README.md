# Wiring and serial protocol

## Selected board and pin map

Classic ESP32-WROOM-32 DevKit, PlatformIO `esp32dev`. Do not use this map on a
PSRAM/WROVER board (GPIO16/17 may be reserved), C3, or S3.

| Actuator | STEP | DIR | Limit input |
|---|---:|---:|---:|
| Base NEMA17 | 13 | 14 | 32 |
| Shoulder NEMA23 | 16 | 17 | 33 |
| Elbow NEMA17 | 18 | 19 | 34 |
| Wrist A NEMA17 | 21 | 22 | 35 |
| Wrist B NEMA17 | 25 | 26 | 36 |

Shared active-low driver ENABLE: GPIO27, with external 10k pullup to 3.3V.
Servo signal: GPIO23. Emergency-stop monitoring input: GPIO39.
All five limit inputs and the E-stop input require external 10k pullups to
3.3V and normally-closed contacts to GND. Open/broken wiring triggers a fault.
GPIO34/35/36/39 have no internal pullups. Wrist inputs may monitor mechanism
travel limits; firmware does not seek or assign a zero to these switches.

Use suitable external motor supplies/drivers and a separate rated servo supply;
share signal ground where required by your driver interface. Never connect motor
windings to the ESP or put 5V on GPIO. Verify driver 3.3V compatibility, enable
polarity, current setting, STEP pulse duration and DIR setup time against its
manual. The current implementation uses 5us STEP high and at least 20us DIR
setup. Microstep DIP/jumper settings must match calibration.

Wire a physical emergency-stop circuit appropriate for the mechanism, with a
brake/support for gravity-loaded axes; the GPIO input alone is a software stop.
Firmware STOP/fault retains stepper torque and cancels servo PWM. DISABLE removes
stepper torque. Power loss and ESP reset can remove torque regardless of software.

## Build

From the root: `python -m platformio run`. The pinned platform uses Arduino-ESP32
2.x and its channel-based LEDC API. Do not switch to Arduino core 3.x without
adapting the servo API. No third-party motor/servo libraries are needed.

References: [PlatformIO board](https://docs.platformio.org/en/stable/boards/espressif32/esp32dev.html),
[ESP32 core migration guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html).

## Protocol v1

115200 baud, 8N1; ASCII lines ending LF (CRLF accepted), max 191 bytes excluding
newline. Joint order: base, shoulder, elbow, wrist pitch, wrist roll, in degrees.
Only one outstanding command; MOVE is not queued.

| Command | Response / meaning |
|---|---|
| PING | OK,PING; refresh movement watchdog |
| STATUS | STATUS,SIM-or-HW,IDLE-or-MOVING,referenced,enabled,faulted,b,s,e,p,r |
| ENABLE | OK,ENABLE; apply holding torque |
| REF,b,s,e,p,r | OK,REF; declare measured pose, requires enabled and idle |
| MOVE,b,s,e,p,r | OK,MOVE then DONE,MOVE on completion |
| GRIP,fraction | OK,GRIP; 0..1 interpolates calibrated pulse endpoints |
| STOP | FAULT,STOP then OK,STOP; invalidate reference, retain holding torque |
| DISABLE | FAULT,DISABLED then OK,DISABLE; invalidate reference, remove torque |
| CLEAR | OK,CLEAR if all interlocks closed; reference still required |

Boot emits READY,1,SIM or READY,1,HW. Rejections emit ERROR,reason. Asynchronous
FAULT,reason cancels motion and invalidates referencing. Status angles are the
last completed/declared target, NOT measured position, and stale during motion
or after a fault. The open-loop controller cannot detect lost steps.

While MOVE runs, the host sends PING every 0.5 seconds; 2 seconds without contact
latches LINK_TIMEOUT. Other motion commands return ERROR,BUSY. STOP, DISABLE,
PING and STATUS remain available. The Python client waits for DONE, not just OK.
On timeout/interruption it sends STOP best-effort. Quit stops and closes serial,
but leaves holding torque enabled. To release: support the arm, then `disable`.

No flash persistence: reboot invalidates reference and starts disabled. To recover
from a limit, support/disable the mechanism, manually release the switch, then
clear, enable and remeasure/ref. There is no powered limit-escape command.
