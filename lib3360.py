"""lib3360.py

Small convenience library wrapping the existing `transmitter.py` API so callers
can send a control frame with one function call.

Functions:
- send_control(m1,m2,s1,s2,dir1,dir2, mode='once', interval=1.0, port=None)
    mode: 'once' | 'loop' | 'hex' ('hex' returns hex string without sending)

This file expects `transmitter.py` to be in the same directory.
"""
from typing import Optional
from pathlib import Path
import importlib.util
import os

# Dynamically load transmitter.py from the same directory as this file. This
# ensures callers can run example scripts from any working directory and still
# pick up the port configuration present in `transmitter.py`.
here = Path(__file__).resolve().parent
transmitter_path = here / 'transmitter.py'
if not transmitter_path.exists():
    raise ImportError(f"transmitter.py not found next to {__file__}")

spec = importlib.util.spec_from_file_location('transmitter', str(transmitter_path))
trans_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trans_mod)

# Pull required functions from the loaded module
send_control_command = trans_mod.send_control_command
send_control_loop = trans_mod.send_control_loop
build_packet = trans_mod.build_packet
# New stateful convenience functions
motor = getattr(trans_mod, 'motor', None)
servo = getattr(trans_mod, 'servo', None)


def send_control(m1: int, m2: int, s1: int, s2: int,
                 dir1: int, dir2: int,
                 mode: str = 'once',
                 interval: float = 1.0,
                 port: Optional[str] = None):
    """High-level single-call API.

    - dir1, dir2: 0 = backward, 1 = forward
    - mode: 'once' (default) sends a single frame and returns None
            'loop' opens the port and sends repeatedly until Ctrl-C
            'hex' returns the hex string for the frame without sending
    - interval: seconds between sends when mode=='loop'
    - port: optional serial port override
    """
    if mode not in ('once', 'loop', 'hex'):
        raise ValueError("mode must be 'once', 'loop' or 'hex'")

    if mode == 'hex':
        frame = build_packet(int(m1), int(m2), int(s1), int(s2), int(dir1), int(dir2))
        return frame.hex()

    if mode == 'once':
        # call transmitter's function which opens/closes port for us
        # allow environment variable override if port not explicitly passed
        env_port = os.getenv('SENDSERIAL_PORT')
        if port is None and env_port:
            port = env_port
        return send_control_command(int(m1), int(m2), int(s1), int(s2), int(dir1), int(dir2), port=port)

    # mode == 'loop'
    env_port = os.getenv('SENDSERIAL_PORT')
    if port is None and env_port:
        port = env_port
    return send_control_loop(int(m1), int(m2), int(s1), int(s2), int(dir1), int(dir2), interval=interval, port=port)


__all__ = ['send_control']
if motor is not None and servo is not None:
    __all__.extend(['motor', 'servo'])
