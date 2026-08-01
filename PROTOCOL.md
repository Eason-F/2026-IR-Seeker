# IR seeker UART protocol

The ESP32 uses UART1 at 460800 baud, 8-N-1:

- ESP32 GPIO 18 (TX) connects to Teensy RX.
- ESP32 GPIO 17 (RX) connects to Teensy TX.
- The boards must share ground. Both use 3.3 V logic.

Each decoded frame contains the following little-endian fields:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Protocol version (`1`) |
| 1 | 1 | Packet type |
| 2 | 2 | Payload length |
| 4 | 2 | Sequence number |
| 6 | 4 | ESP32 capture timestamp in microseconds |
| 10 | Variable | Payload |
| Last 2 | 2 | CRC-16/CCITT-FALSE |

The complete decoded frame is COBS encoded and followed by `0x00`. CRC uses
polynomial `0x1021`, initial value `0xFFFF`, no reflection, and no final XOR.

## Packet types

### `0x01` Ball measurement

Sent at 250 Hz by default. Its 12-byte payload is:

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 2 | `int16` | Bearing in centidegrees; clockwise is positive |
| 2 | 2 | `uint16` | Signal strength, 0 to 65535 |
| 4 | 1 | `uint8` | Confidence, 0 to 255 |
| 5 | 1 | `uint8` | Strongest sensor, 0 to 17; `255` means none |
| 6 | 4 | `uint32` | Active receiver mask; lower 18 bits are used |
| 10 | 2 | `uint16` | Measurement flags |

Flag bits are: valid (`0`), saturated (`1`), interference (`2`), weak (`3`),
multiple clusters (`4`), calibration valid (`5`), capture overrun (`6`),
sensor fault (`7`), and power warning (`8`). Ignore the bearing unless the
valid flag is set.

### `0x02` Raw sensors

Disabled by default. The 42-byte payload contains capture duration (`uint16`),
sample count (`uint16`), 18 calibrated sensor values (`uint16[18]`), and flags
(`uint16`). It can be enabled using the raw-rate command.

### `0x03` Status

Sent once per second. The 20-byte payload contains uptime seconds, supply mV
(`0` means unavailable), measured update rate, UART error count, capture
overrun count, unexpected-reset indicator, faulty-sensor mask, and status
flags.

### `0x04` Device information

Sent at startup or on request. It contains hardware version, the three firmware
version numbers, sensor count, capability bits, build identifier, and the lower
32 bits of the ESP32 device ID.

### `0x10` Command

The Teensy sends a command ID, request ID, and optional arguments. Supported
commands request device information (`1`), request status (`2`), set the
measurement rate (`3`), set the raw rate (`4`), enter diagnostics (`5`), exit
diagnostics (`6`), reload calibration (`7`, currently unavailable), and ping
(`8`). Rates are little-endian `uint16` values. Measurement rate can be 100,
250, or 500 Hz; raw rate can be 0 through 100 Hz.

### `0x11` Command response

The payload contains the original command ID, request ID, result, and optional
data. Result values are success (`0`), unknown command (`1`), invalid argument
(`2`), unavailable (`3`), failed (`4`), and malformed (`5`).

## Receiver pin order

The direct GPIO pin list is in `include/config.h`. Sensor 0 must face forward,
and sensor numbers increase clockwise in 20-degree steps. Update that list to
match the PCB before powering the complete sensor ring.
