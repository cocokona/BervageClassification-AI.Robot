"""example.py

Demonstrates using `lib3360.send_control`.

This example shows how to:
- get the raw hex frame without sending
- send one frame
- (optionally) start the continuous send loop

Run in the same directory as `transmitter.py` and `lib3360.py`.
"""
import time
import sys

try:
    # Preferred import: use the new motor/servo API
    from lib3360 import motor, servo
except Exception:
    # Fallback: load by filename using importlib (works even if you named it 3360lib.py)
    import importlib.util
    import pathlib
    import os

    lib_path = pathlib.Path(__file__).resolve().parent / 'lib3360.py'
    spec = importlib.util.spec_from_file_location('lib3360', str(lib_path))
    lib = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lib)
    motor = getattr(lib, 'motor')
    servo = getattr(lib, 'servo')

# Optionally allow overriding serial port via environment variable or CLI
PORT = None
import os
if os.getenv('SENDSERIAL_PORT'):
    PORT = os.getenv('SENDSERIAL_PORT')
elif len(sys.argv) > 1:
    PORT = sys.argv[1]


def demo_hex():
    # Update servo and motor values without sending by using mode='hex'
    servo(1000, 1400, mode='hex')
    hexstr = motor(1000, 1500, dir1=0, dir2=1, mode='hex')
    print('Hex frame (header+payload+footer):', hexstr)


def demo_once():
    print('Sending one frame...')
    # update servo state (no send) then send once with motor() which uses current state
    servo(2000, 1400, mode='hex')
    motor(0, 0, dir1=0, dir2=1, mode='once', port=PORT)


def demo_loop():
    print('Starting send loop (press Ctrl-C to stop)')
    # update servo state (no send) then start motor loop which repeatedly sends current full frame
    servo(1000, 1400, mode='hex')
    motor(1000, 1500, dir1=0, dir2=1, mode='loop', interval=1.0, port=PORT)


if __name__ == '__main__':
    time.sleep(0.2)
    # demo_hex()
    # Uncomment to actually send once (requires transmitter port available):
    demo_once()
    # Uncomment to start continuous send loop:
    # demo_loop()
