"""
transmitter.py

Sends a fixed-length serial packet to the STM32.

Packet format (default, to match example):
- 4 x uint16 (big-endian) : motor1, motor2, servo1, servo2 (each 2 bytes)
- 1 x uint8 : direction flags (bit0 = motor1 dir, bit1 = motor2 dir)

Total payload length: 9 bytes. Optional header/footer bytes are provided
as constants at the top; they default to empty to match the expected
string like "03e805dc07d009c402".

Usage:
 - Configure ports at the top of this file.
 - Run on Windows or Linux; other OS will error out.

Requires: pyserial
"""
import platform
import struct
import sys
import time
import os

try:
	import serial
except ModuleNotFoundError:
	raise SystemExit("pyserial is required. Install with: pip install pyserial")

# ----------------------- Port / Serial Config (top of file) -----------------------
# Windows and Linux port names. Change as needed.
# Windows and Linux port names. Change as needed.
TX_PORT_WIN = 'COM15'
# TX_PORT_LIN = '/dev/ttyUSB0'
TX_PORT_LIN = '/dev/ttyTHS1'
BAUDRATE = 115200
TIMEOUT = 1  # seconds

# Frame header/footer (updated per request): 0x0D (CR) header, 0x20 (space) footer
# These are included in the transmitted frame and the receiver must use the same.
FRAME_HEADER = b"\x0D"
FRAME_FOOTER = b"\x20"
# -------------------------------------------------------------------------------

# Module-level persistent state. Callers can update only the values they want
# by passing `None` for unchanged parameters (e.g. `motor(None, 50000)`).
# Defaults chosen to match example values used previously.
STATE = {
	'm1': 1000,
	'm2': 1500,
	's1': 1000,
	's2': 1400,
	'd1': 0,
	'd2': 1,
}


def detect_port():
	os_name = platform.system().lower()
	if os_name.startswith('win'):
		return TX_PORT_WIN
	if os_name.startswith('linux'):
		return TX_PORT_LIN
	raise SystemExit(f"Unsupported OS: {platform.system()}. Only Windows and Linux supported.")


def build_packet(m1, m2, s1, s2, dir_m1_flag=0, dir_m2_flag=0):
	"""Builds the packet bytes.

	- m1,m2,s1,s2: integers 0..65535
	- dir_m1_flag, dir_m2_flag: integers 0 or 1 where 0=backward, 1=forward

	Returns full frame (header + payload + footer) as bytes.
	"""
	if not all(isinstance(v, int) and 0 <= v <= 0xFFFF for v in (m1, m2, s1, s2)):
		raise ValueError('PWM values must be integers 0..65535')

	# Ensure direction flags are either 0 or 1
	d1 = 1 if int(dir_m1_flag) & 0x1 else 0
	d2 = 1 if int(dir_m2_flag) & 0x1 else 0

	# bit0 = motor1 direction flag, bit1 = motor2 direction flag
	dir_byte = (d1) | (d2 << 1)
	payload = struct.pack('>HHHHB', int(m1), int(m2), int(s1), int(s2), dir_byte)
	return FRAME_HEADER + payload + FRAME_FOOTER


def bytes_to_hex(b: bytes) -> str:
	return b.hex()


def _update_state(m1=None, m2=None, s1=None, s2=None, d1=None, d2=None):
	"""Update the persistent `STATE` with any non-None parameters."""
	if m1 is not None:
		STATE['m1'] = int(m1)
	if m2 is not None:
		STATE['m2'] = int(m2)
	if s1 is not None:
		STATE['s1'] = int(s1)
	if s2 is not None:
		STATE['s2'] = int(s2)
	if d1 is not None:
		STATE['d1'] = 1 if int(d1) & 0x1 else 0
	if d2 is not None:
		STATE['d2'] = 1 if int(d2) & 0x1 else 0


def motor(m1=None, m2=None, dir1=None, dir2=None, mode='once', interval=1.0, port=None):
	"""Update motor values/directions and send according to `mode`.

	Parameters may be `None` to keep previous value (state memory).
	- `mode`: 'once' | 'loop' | 'hex'
	- When `mode=='hex'` returns the full frame hex string (header+payload+footer)
	- When `mode=='once'` sends a single frame (opens/closes port)
	- When `mode=='loop'` opens port once and sends repeatedly until Ctrl-C
	"""
	if mode not in ('once', 'loop', 'hex'):
		raise ValueError("mode must be 'once', 'loop' or 'hex'")

	# Update only the motor-related parts of the state
	_update_state(m1=m1, m2=m2, d1=dir1, d2=dir2)

	if mode == 'hex':
		frame = build_packet(STATE['m1'], STATE['m2'], STATE['s1'], STATE['s2'], STATE['d1'], STATE['d2'])
		return frame.hex()

	if mode == 'once':
		env_port = os.getenv('SENDSERIAL_PORT')
		if port is None and env_port:
			port = env_port
		return send_control_command(STATE['m1'], STATE['m2'], STATE['s1'], STATE['s2'], STATE['d1'], STATE['d2'], port=port)

	# loop
	env_port = os.getenv('SENDSERIAL_PORT')
	if port is None and env_port:
		port = env_port
	return send_control_loop(STATE['m1'], STATE['m2'], STATE['s1'], STATE['s2'], STATE['d1'], STATE['d2'], interval=interval, port=port)


def servo(s1=None, s2=None, mode='once', interval=1.0, port=None):
	"""Update servo values and send according to `mode`.

	Parameters may be `None` to keep previous value (state memory).
	"""
	if mode not in ('once', 'loop', 'hex'):
		raise ValueError("mode must be 'once', 'loop' or 'hex'")

	# Update only the servo-related parts of the state
	_update_state(s1=s1, s2=s2)

	if mode == 'hex':
		frame = build_packet(STATE['m1'], STATE['m2'], STATE['s1'], STATE['s2'], STATE['d1'], STATE['d2'])
		return frame.hex()

	if mode == 'once':
		env_port = os.getenv('SENDSERIAL_PORT')
		if port is None and env_port:
			port = env_port
		return send_control_command(STATE['m1'], STATE['m2'], STATE['s1'], STATE['s2'], STATE['d1'], STATE['d2'], port=port)

	# loop
	env_port = os.getenv('SENDSERIAL_PORT')
	if port is None and env_port:
		port = env_port
	return send_control_loop(STATE['m1'], STATE['m2'], STATE['s1'], STATE['s2'], STATE['d1'], STATE['d2'], interval=interval, port=port)


def send_control_command(m1, m2, s1, s2, dir1, dir2, port=None):
	"""Send one control command line.

	- dir1, dir2: 0 = backward, 1 = forward (integers)
	- port: optional override port string; if None, detect by OS.
	"""
	if port is None:
		port = detect_port()

	frame = build_packet(m1, m2, s1, s2, dir1, dir2)
	print(f"Opening serial port: {port} @ {BAUDRATE}")
	with serial.Serial(port, BAUDRATE, timeout=TIMEOUT) as ser:
		# small pause for some USB-serial adapters
		time.sleep(0.05)
		sent = ser.write(frame)
		ser.flush()
	print(f"Sent {sent} bytes -> {bytes_to_hex(frame)}")


def send_control_loop(m1, m2, s1, s2, dir1, dir2, interval=1.0, port=None):
	"""Open the serial port once and continuously send the same control frame.

	- interval: seconds between sends
	- stops on KeyboardInterrupt
	"""
	if port is None:
		port = detect_port()

	frame = build_packet(m1, m2, s1, s2, dir1, dir2)
	print(f"Opening serial port: {port} @ {BAUDRATE} — sending every {interval}s. Ctrl-C to stop")
	with serial.Serial(port, BAUDRATE, timeout=TIMEOUT) as ser:
		# small pause for some USB-serial adapters
		time.sleep(0.05)
		try:
			while True:
				sent = ser.write(frame)
				ser.flush()
				print(f"Sent {sent} bytes -> {bytes_to_hex(frame)}")
				time.sleep(interval)
		except KeyboardInterrupt:
			print('\nSend loop stopped by user')


if __name__ == '__main__':
	# Continuous send loop (single-line): dir flags: 0=backward, 1=forward
	send_control_loop(10000, 20000, 1000, 1400, 1, 1)
