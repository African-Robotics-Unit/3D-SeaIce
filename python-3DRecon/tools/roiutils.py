import numpy as np


def decode_bound(value):
    """Convert a JSON-safe bound string (or number) to float, handling ±inf."""
    if isinstance(value, str):
        value = value.strip().lower()
        if value == "-inf":
            return -np.inf
        if value == "inf":
            return np.inf
    return float(value)


def encode_bound(value):
    """Convert a float bound (potentially ±inf) to a JSON-safe string."""
    if np.isneginf(value):
        return "-inf"
    if np.isposinf(value):
        return "inf"
    return str(value)


def decode_roi(roi_cfg):
    """
    Convert a config ROI dict to a list of [min, max] bound pairs.

    Parameters
    ----------
    roi_cfg : dict with keys "x", "y", "z", each a [min_str, max_str] list

    Returns
    -------
    list of three [float, float] pairs: [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
    """
    return [
        [decode_bound(roi_cfg["x"][0]), decode_bound(roi_cfg["x"][1])],
        [decode_bound(roi_cfg["y"][0]), decode_bound(roi_cfg["y"][1])],
        [decode_bound(roi_cfg["z"][0]), decode_bound(roi_cfg["z"][1])],
    ]


def encode_roi(x_roi, y_roi, z_roi):
    """Convert three [min, max] float pairs to the config dict format."""
    return {
        "x": [encode_bound(x_roi[0]), encode_bound(x_roi[1])],
        "y": [encode_bound(y_roi[0]), encode_bound(y_roi[1])],
        "z": [encode_bound(z_roi[0]), encode_bound(z_roi[1])],
    }
