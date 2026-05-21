from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path


def get_logger(name: str = "waver_patrol", log_dir: str | Path = "logs") -> logging.Logger:
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)
    path = Path(log_dir).expanduser()
    try:
        path.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(
            path / f"{datetime.now().strftime('%Y%m%d')}_{name}.log",
            encoding="utf-8",
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    except OSError as exc:
        logger.warning("Could not open log file in %s: %s", path, exc)
    return logger
