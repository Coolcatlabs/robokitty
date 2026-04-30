"""
ax12 – AX-12A Dynamixel servo driver.

Typical usage::

    from ax12 import AX12Interface

    with AX12Interface('/dev/ttyUSB0') as ax:
        ax.torque_enable(1)
        ax.move_degrees(1, 150.0, speed=512)
        ax.wait_for_stop(1)
        print(ax.get_status(1))
"""

from .ax12 import AX12Interface
from .bus import SerialBus
from .status import StatusPacket, ErrorFlag
from .instruction import InstructionPacketBuilder
from .registers import Register
from .utils import degrees_to_position, to_le_bytes, clamp_position

__all__ = [
    "AX12Interface",
    "SerialBus",
    "StatusPacket",
    "ErrorFlag",
    "InstructionPacketBuilder",
    "Register",
    "degrees_to_position",
    "to_le_bytes",
    "clamp_position",
]
