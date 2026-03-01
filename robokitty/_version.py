import importlib.metadata

_DEFAULT_VERSION = "0.0.1"
try:
    __version__ = importlib.metadata.version(__package__)
except importlib.metadata.PackageNotFoundError:
    __version__ = _DEFAULT_VERSION