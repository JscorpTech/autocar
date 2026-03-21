"""
Centralized logging for the Autonomous Car project.

All serial communication between Raspberry Pi and ESP32 is written to:
  - console  (INFO level and above, human-readable)
  - raspberry_pi/logs/comm.log  (DEBUG level and above, rotating, 1 MB × 5 files)

Usage:
    from logger import setup_logging, get_logger

    setup_logging()                 # call once at program start
    log = get_logger("comm")        # per-module logger
    log.info("TX → CMD:150,0")
"""

import logging
import logging.handlers
import os
from config import LOG_DIR, LOG_LEVEL


def setup_logging():
    """Configure root logger. Call exactly once at startup."""
    os.makedirs(LOG_DIR, exist_ok=True)
    log_file = os.path.join(LOG_DIR, "comm.log")

    fmt = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d [%(name)s] %(levelname)-5s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # rotating file handler — keeps last 5 × 1 MB
    fh = logging.handlers.RotatingFileHandler(
        log_file, maxBytes=1_000_000, backupCount=5, encoding="utf-8"
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)

    # console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(fmt)

    root = logging.getLogger()
    root.setLevel(LOG_LEVEL)
    # avoid duplicate handlers if setup_logging() is called more than once
    if not root.handlers:
        root.addHandler(fh)
        root.addHandler(ch)


def get_logger(name: str) -> logging.Logger:
    """Return a child logger under the given name."""
    return logging.getLogger(name)
