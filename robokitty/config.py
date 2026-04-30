# config.py
import platform

DEFAULT_PORT = "COM3" if platform.system() == "Windows" else "/dev/ttyUSB0"
DEFAULT_BAUD = 1_000_000  # 1 Mbps factory default for AX12
