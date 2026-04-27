"""
receiver.py

Reads fixed-length serial packets from the STM32 and parses them.

Packet format expected (matching transmitter defaults):
- 4 x uint16 (big-endian) : motor1, motor2, servo1, servo2
- 1 x uint8 : direction flags (bit0 = motor1 dir, bit1 = motor2 dir)

Example payload (hex): "03e805dc07d009c402" -> m1=1000,m2=1500,s1=2000,s2=2500,dir=0x02

Requires: pyserial
"""
import platform
import struct
import sys
import time

try:
	import serial
except ModuleNotFoundError:
	raise SystemExit("pyserial is required. Install with: pip install pyserial")

# ----------------------- Port / Serial Config (top of file) -----------------------
# Windows and Linux port names. Change as needed.
RX_PORT_WIN = 'COM15'
# RX_PORT_LIN = '/dev/ttyUSB1'
RX_PORT_LIN = '/dev/ttyTHS1'
BAUDRATE = 115200
TIMEOUT = 1  # seconds

# Frame header/footer (must match transmitter): 0x0D (CR) header, 0x20 (space) footer
FRAME_HEADER = b"\x0D"
FRAME_FOOTER = b"\x20"
# -------------------------------------------------------------------------------


def detect_port():
	os_name = platform.system().lower()
	if os_name.startswith('win'):
		return RX_PORT_WIN
	if os_name.startswith('linux'):
		return RX_PORT_LIN
	raise SystemExit(f"Unsupported OS: {platform.system()}. Only Windows and Linux supported.")


def parse_frame(frame: bytes):
	"""Parse a full frame (header+payload+footer) and return values.

	Returns tuple: (m1,m2,s1,s2, dir_m1_bool, dir_m2_bool)
	"""
	header_len = len(FRAME_HEADER)
	footer_len = len(FRAME_FOOTER)
	payload = frame[header_len: len(frame) - footer_len if footer_len else None]
	if len(payload) != 9:
		raise ValueError(f'Unexpected payload length: {len(payload)}')

	m1, m2, s1, s2, dir_byte = struct.unpack('>HHHHB', payload)
	dir_m1 = bool(dir_byte & 0x01)
	dir_m2 = bool(dir_byte & 0x02)
	return m1, m2, s1, s2, dir_m1, dir_m2


def main_listen_loop():
	port = detect_port()
	frame_len = len(FRAME_HEADER) + 9 + len(FRAME_FOOTER)
	print(f"Listening on {port} @ {BAUDRATE}, expecting {frame_len} byte frames")

	buf = bytearray()
	with serial.Serial(port, BAUDRATE, timeout=TIMEOUT) as ser:
		try:
			while True:
				# Read up to 64 bytes at a time (non-blocking up to timeout)
				chunk = ser.read(64)
				if chunk:
					buf.extend(chunk)
				else:
					# timeout occurred, loop back to allow KeyboardInterrupt
					continue

				# Find header position
				idx = buf.find(FRAME_HEADER)
				if idx == -1:
					# No header yet; trim buffer if it grows too large
					if len(buf) > 2048:
						buf.clear()
					continue

				# Wait until we have a full frame after the header
				if len(buf) - idx < frame_len:
					continue

				candidate = bytes(buf[idx: idx + frame_len])
				# Verify footer
				if not candidate.endswith(FRAME_FOOTER):
					# Discard this header and search again
					del buf[: idx + 1]
					continue

				# Parse and report
				try:
					m1, m2, s1, s2, dir_m1, dir_m2 = parse_frame(candidate)
					print(f"Received raw: {candidate.hex()}")
					print(f"m1={m1}, m2={m2}, s1={s1}, s2={s2}, dir_m1={dir_m1}, dir_m2={dir_m2}")
				except Exception as e:
					print(f"Parse error: {e}")

				# Remove consumed bytes from buffer
				del buf[: idx + frame_len]
		except KeyboardInterrupt:
			print('\nReceiver stopped by user')


if __name__ == '__main__':
	main_listen_loop()

