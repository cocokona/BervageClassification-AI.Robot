# SendSerial (EIE3360) — Usage & Debug Guide

This folder provides simple transmitter and receiver examples for sending
control frames from a Linux/Windows computer to an STM32 over a USART link.

Files
- `transmitter.py` — builds and sends fixed-length control frames. Contains
  `send_control_command(...)` and `send_control_loop(...)` plus port config at
  the top of the file.
- `receiver.py` — continuous reader that resynchronizes the incoming stream,
  verifies header/footer and prints parsed values.
- `lib3360.py` — convenience library exposing `motor(...)` and `servo(...)` wrappers
  (replaces the older single-call `send_control(...)` API). Backwards compatibility
  to `send_control` is retained.
- `3360lib.py` — compatibility wrapper that loads `lib3360.py` by path.
- `example.py` — small demo showing how to get the hex frame or send frames.

Protocol (frame layout)
- Total frame length: 11 bytes
  - Header: 1 byte = `0x0D`
  - Payload: 9 bytes:
    - motor1 PWM: 2 bytes unsigned (big-endian)
    - motor2 PWM: 2 bytes unsigned (big-endian)
    - servo1 PWM: 2 bytes unsigned (big-endian)
    - servo2 PWM: 2 bytes unsigned (big-endian)
    - direction byte: 1 byte (bit flags)
      - bit0 (LSB): motor1 direction (0 = backward, 1 = forward)
      - bit1: motor2 direction (0 = backward, 1 = forward)
      - bits 2..7: unused
  - Footer: 1 byte = `0x20`

Example (values used in examples)
- m1=1000 -> hex `03e8`
- m2=1500 -> hex `05dc`
- s1=2000 -> hex `07d0`
- s2=2500 -> hex `09c4`
- dir byte: `0x02` (motor2 forward)
- Payload hex: `03e805dc07d009c402`
- Full frame (header+payload+footer): `0d03e805dc07d009c40220`

Requirements
- Python 3.8+ recommended
- `pyserial` library

Install dependencies
```bash
pip install pyserial
```

Configuration
- Edit serial port names at the top of `transmitter.py` and `receiver.py`:
  - Windows: change `TX_PORT_WIN` / `RX_PORT_WIN` (e.g., `COM14`, `COM15`)
  - Linux: change `TX_PORT_LIN` / `RX_PORT_LIN` (e.g., `/dev/ttyUSB0`, `/dev/ttyUSB1`)
- Adjust `BAUDRATE` and `TIMEOUT` if needed (default `115200`).

How to run (quick)
- On Windows (cmd.exe):
```bat
python transmitter.py    # transmitter sends a continuous loop by default
python receiver.py       # receiver listens and prints parsed frames
```
- On Linux:
```sh
python3 transmitter.py
python3 receiver.py
```

Using the library API in other Python code
- Import `motor` and `servo` from `lib3360` and call them. The `transmitter.py`
  module now keeps a small in-memory state for the four PWM values and two
  direction flags. Passing `None` for a parameter leaves the previous value
  unchanged (e.g. `motor(None, 50000)` updates only `m2`).

```py
from lib3360 import motor, servo

# Set servo values (no send) and request the full frame hex without transmitting
servo(1000, 1400, mode='hex')
hexstr = motor(1000, 1500, dir1=0, dir2=1, mode='hex')
print('Frame hex:', hexstr)

# Update only motor2 and send once (motor1 remains unchanged)
motor(None, 50000, mode='once')

# Start a continuous send loop using current state (Ctrl-C to stop)
motor(1200, None, mode='loop', interval=1.0)
```

**Stateful behaviour**
- **In-process memory:** `motor()` and `servo()` keep a small in-memory state for `m1, m2, s1, s2, d1, d2` while your Python process runs. Calling `motor(None, 50000)` updates only `m2` in that same process; it does not persist across separate program runs.
- **Pass `None` to keep previous value:** If you call `motor(40000, 20000, dir1=1, dir2=1)` and then later call `motor(40000, None, dir1=1, dir2=1)`, the second call will keep `m2=20000` unchanged and only update `m1` (and directions). This is supported and recommended when you only want to change one motor.
- **Troubleshooting:** If you observe that a value unexpectedly becomes zero or stops after calling with `None`, ensure both calls occur in the same Python process (same script or interactive session). Re-running a separate script resets the module state to defaults.


Expected behavior & outputs
- `transmitter.py` (default, continuous loop) will print lines like:
  `Opening serial port: COM14 @ 115200 — sending every 1.0s. Ctrl-C to stop`
  `Sent 11 bytes -> 0d03e805dc07d009c40220`
- `receiver.py` will print the raw frame hex and decoded values:
  `Received raw: 0d03e805dc07d009c40220`
  `m1=1000, m2=1500, s1=2000, s2=2500, dir_m1=False, dir_m2=True`

Debug guide

1) Confirm `pyserial` installed
```bash
python -c "import serial; print(serial.__version__)"
```

2) Confirm port names and access
- Windows: check Device Manager or run `python -m serial.tools.list_ports` to list COM ports.
- Linux: `ls /dev/ttyUSB*` or `dmesg | tail` after plugging in the USB adapter. Ensure your user has permission (add to `dialout` group or use `sudo`).

3) Test frame creation locally (no hardware)
- Run `example.py` which prints the hex for the example frame. That verifies the same bytes are produced by the library.

4) Loopback test (if you don't have an STM32)
- On Windows you can create a virtual pair (com0com) and point `transmitter.py` to one end and `receiver.py` to the other.
- On Linux you can use `socat` to create a linked pair:
```sh
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# use the two printed /dev/pts/X ports as TX and RX
```

5) Capture serial bytes with a monitor
- Use `screen` (Linux) or PuTTY (Windows) to open the RX port and visually inspect bytes if needed.

6) Common problems & fixes
- Permission denied on Linux: add your user to `dialout` or run with `sudo`.
- Port already open: ensure no other app (IDE, terminal) has the port open.
- Garbage frames: ensure both transmitter and receiver use the same header/footer/baudrate/endianness.
- If frames sometimes fail to parse, increase `TIMEOUT` or use the loopback method to debug.

STM32 decoding tips (summary)
- Search for header byte `0x0D` in the incoming stream.
- When header found, ensure 11 bytes available; validate footer byte at offset 10 is `0x20`.
- Extract four big-endian `uint16_t` values and one `uint8_t` dir byte.
- Map PWM values to timers (CCR) and set motor direction GPIOs from bits 0..1.
- Implement a safety timeout to stop motors if no valid frame is received for N ms.

Extending the protocol
- If you later need integrity checks, add a checksum or CRC within the payload and update both transmitter/receiver.
- To support multiple message types, add a small `msg_type` byte in the payload and a length field.

Further work (suggested)
- Add unit tests that verify `build_packet()` output; example test asserts frame hex equals expected.
- Add a small script that simulates the STM32 side and exercises the real serial link.

Contact / support
- If you want, I can generate a ready-to-drop-in C file using STM32 HAL with DMA-based UART reception and the parsing code shown in the course notes.

---
File locations (in this workspace)
- `transmitter.py` — transmitter implementation
- `receiver.py` — receiver implementation
- `lib3360.py` / `3360lib.py` — library wrappers
- `example.py` — example/demo

**Example.py — Detailed usage**

This project already contains `example.py` which demonstrates three modes:
- `demo_hex()` — returns/prints the full frame hex string (header+payload+footer)
- `demo_once()` — sends a single frame (opens and closes the serial port)
- `demo_loop()` — opens the serial port and sends the same frame repeatedly until interrupted

Run the provided `example.py` from the SendSerial folder to see the hex output quickly:
```bash
python example.py
```
Expected immediate output (demo_hex prints):
```
Hex frame (no header/footer shown in payload): 0d03e805dc07d009c40220
```
Notes:
- The printed hex includes header `0x0D` at the start and footer `0x20` at the end.
- `example.py` has `demo_once()` and `demo_loop()` commented. Uncomment them to actually send frames.

If you want to create a new Python script from scratch and use the library, start with an empty file and copy one of these minimal examples.

1) Get the raw frame hex (no serial I/O)
```py
# file: send_hex.py
from lib3360 import motor, servo

# update servo state (no transmit)
servo(2000, 2500, mode='hex')
# update motor state and get hex
hexstr = motor(1000, 1500, dir1=0, dir2=1, mode='hex')
print('Frame hex:', hexstr)  # -> 0d03e805dc07d009c40220
```

2) Send one frame (opens and closes the port)
```py
# file: send_once.py
from lib3360 import motor, servo

# first update servo state (no transmit), then send once via motor()
servo(2000, 2500, mode='hex')
# If the module can't find the default port, you can pass `port='COM14'` or '/dev/ttyUSB0'
motor(1000, 1500, dir1=0, dir2=1, mode='once', port=None)
```
Expected behavior: the script prints the port-opening message and `Sent 11 bytes -> 0d03...20`.

3) Start an indefinite send loop (stop with Ctrl-C)
```py
# file: send_loop.py
from lib3360 import motor, servo

# update servo state (no transmit) then start motor loop which repeatedly sends
servo(2000, 2500, mode='hex')
motor(1000, 1500, dir1=0, dir2=1, mode='loop', interval=1.0)
```
Expected behavior: opens the port once and prints one `Sent ...` line per `interval` second until you press Ctrl-C.

Overriding the port or other parameters
- To override which serial port is used, pass `port='COM14'` (Windows) or `port='/dev/ttyUSB0'` (Linux) to `motor()` or `servo()` (whichever you call to perform the send).
- To change the send frequency in loop mode, set `interval=` to the number of seconds between sends.

Important: where to change the configured port
- The default serial port values live in `transmitter.py` and `receiver.py` as `TX_PORT_WIN`, `TX_PORT_LIN`, `RX_PORT_WIN`, `RX_PORT_LIN` near the top of each file. If you edit those files to change the port, the library and examples will pick up the new values when imported.
- If you run `example.py` (or your own script) from a different working directory, the library now reads `transmitter.py` by file path relative to `lib3360.py`, so changes to `transmitter.py` will be used regardless of cwd.
- Alternatively you can override the port at runtime using the `SENDSERIAL_PORT` environment variable or by passing a port string as the first command-line argument to `example.py`.

Environment variable example (Windows cmd.exe):
```bat
set SENDSERIAL_PORT=COM14
python example.py
```
Linux/macOS:
```sh
export SENDSERIAL_PORT=/dev/ttyUSB0
python3 example.py
```

Command-line override example:
```sh
python example.py COM14
```

Importing when module names might conflict or are not on `PYTHONPATH`

The library `lib3360.py` is designed to be imported normally with `from lib3360 import motor, servo` when the file is in the same folder as your script. If you named the file `3360lib.py` (starts with a digit) or your script lives in a different folder, use one of the following approaches.

Option A — plain import (recommended when files are colocated)
```py
# send_once.py (in same directory)
from lib3360 import motor, servo
# update servo state (no transmit) then send once via motor()
servo(2000, 2500, mode='hex')
motor(1000,1500, dir1=0, dir2=1, mode='once')
```

Option B — import by file path with importlib (works with any filename)
```py
# send_once_dynamic.py
import importlib.util
from pathlib import Path

lib_path = Path(__file__).resolve().parent / '3360lib.py'  # or lib3360.py
spec = importlib.util.spec_from_file_location('mylib', str(lib_path))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

# `3360lib.py` forwards motor/servo, so call them directly
motor = mod.motor
servo = mod.servo
servo(2000,2500, mode='hex')
motor(1000,1500, dir1=0, dir2=1, mode='once')
```

Option C — add the library folder to `PYTHONPATH` at runtime
```py
import sys
sys.path.append(r'C:\path\to\SendSerial')  # or '/home/user/SendSerial'
from lib3360 import motor, servo
```

Debugging example runs
- If `send_control(..., mode='once')` raises `serial.SerialException`, verify the port and that no other program has it open.
- If `mode='hex'` returns a value, check it matches the expected example hex: `0d03e805dc07d009c40220`.
- If the receiver is not parsing frames, confirm `receiver.py` uses the same `FRAME_HEADER` and `FRAME_FOOTER` values.

Quick checklist to run example from empty script:
1. Create a new file `send_once.py` with the `Option A` snippet above.
2. From the `SendSerial` folder run:
```bash
pip install pyserial
python send_once.py
```
3. Expected output when successful:
```
Opening serial port: COM14 @ 115200
Sent 11 bytes -> 0d03e805dc07d009c40220
```

