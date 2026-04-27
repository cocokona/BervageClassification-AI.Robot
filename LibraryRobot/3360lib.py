"""3360lib.py

Compatibility file named as requested. Note: importing a module whose name
starts with a digit is not supported with the normal `import` statement. This
file exists so users can load it by filename using importlib if they prefer.

It simply forwards to `lib3360.py` implementation.
"""
from importlib import util
from pathlib import Path
import sys

# Load lib3360.py from the same directory as this file
here = Path(__file__).resolve().parent
spec = util.spec_from_file_location('lib3360', str(here / 'lib3360.py'))
module = util.module_from_spec(spec)
spec.loader.exec_module(module)

# Re-export send_control at module level
send_control = module.send_control

__all__ = ['send_control']
