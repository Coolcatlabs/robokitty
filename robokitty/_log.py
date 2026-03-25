import logging
from datetime import datetime
from pathlib import Path

from rich.logging import RichHandler


def _ensure_log_directory(directory_name: str = ".logs") -> Path:
    """Creates the log directory if it doesn't exist."""
    log_path = Path(directory_name)
    log_path.mkdir(exist_ok=True)
    return log_path


def _get_log_filename(directory: Path) -> Path:
    """Generates a timestamped path for the log file."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return directory / f"{__package__}_{timestamp}.log"


def _get_console_handler() -> RichHandler:
    """Returns a RichHandler for colored terminal output."""
    handler = RichHandler(rich_tracebacks=True, markup=True, show_path=False)
    handler.setLevel(logging.INFO)
    return handler


def _get_file_handler(file_path: Path) -> logging.FileHandler:
    """Returns a formatted FileHandler for persistent logs."""
    handler = logging.FileHandler(file_path)
    handler.setLevel(logging.DEBUG)

    formatter = logging.Formatter(
        "%(asctime)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    handler.setFormatter(formatter)
    return handler


def get_logger() -> logging.Logger:
    """Assembles the logger with both console and file handlers."""
    logger = logging.getLogger(__package__)
    logger.setLevel(logging.DEBUG)

    log_dir = _ensure_log_directory()
    log_file = _get_log_filename(log_dir)

    logger.addHandler(_get_console_handler())
    logger.addHandler(_get_file_handler(log_file))

    return logger
