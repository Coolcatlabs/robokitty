"""
tools/_utils.py – shared formatting helpers for AX-12A diagnostic tools.
"""

W = 55  # line width


def header(title: str) -> None:
    print(f"\n  {'─' * (W - 4)}")
    print(f"  {title}")
    print(f"  {'─' * (W - 4)}")


def row(label: str, value: str, warn: str = "") -> None:
    marker = "!!" if warn else "  "
    print(f"  {marker}  {label:<26} {value}")
    if warn:
        print(f"       └─ {warn}")


def safe_read(read_fn, *args, label: str = "?"):
    """
    Call read_fn(*args). On failure, print an inline error and return None
    so the caller can continue past the failed read.
    """
    try:
        return read_fn(*args)
    except Exception as e:
        print(f"  !!  {'READ ERROR':<26} {label} — {e}")
        return None


def safe_write(write_fn, *args, label: str = "?") -> bool:
    """
    Call write_fn(*args). On failure, print an inline error and return False
    so the caller can decide whether to abort or continue.
    """
    try:
        write_fn(*args)
        return True
    except Exception as e:
        print(f"  !!  {'WRITE ERROR':<26} {label} — {e}")
        return False
