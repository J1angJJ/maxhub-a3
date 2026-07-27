import argparse

import numpy as np


def parse_target_position(raw):
    if raw is None:
        return None
    parts = [item.strip() for item in raw.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--target-position must be formatted as x,y,z")
    try:
        return np.array([float(item) for item in parts], dtype=np.float32)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--target-position must contain numeric values") from exc


def parse_target_positions(raw):
    if raw is None:
        return None
    targets = []
    for chunk in raw.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        targets.append(parse_target_position(chunk))
    if not targets:
        raise argparse.ArgumentTypeError("--hard-target-positions must contain at least one x,y,z target")
    return np.stack(targets).astype(np.float32)
