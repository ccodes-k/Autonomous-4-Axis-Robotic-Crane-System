import json
import math
import urllib.request
import numpy as np
import pyzed.sl as sl

BOX_NAME_MAP = {
    "Small_Box": "BOX_SMALL",
    "Mid_Box": "BOX_MEDIUM",
    "Big_Box": "BOX_LARGE",
    "box_small": "BOX_SMALL",
    "box_medium": "BOX_MEDIUM",
    "box_large": "BOX_LARGE",
    "BOX_SMALL": "BOX_SMALL",
    "BOX_MEDIUM": "BOX_MEDIUM",
    "BOX_LARGE": "BOX_LARGE",
}


def valid_xyz(v):
    if v is None or len(v) < 3:
        return False
    x, y, z = v[:3]
    return np.isfinite(x) and np.isfinite(y) and np.isfinite(z)


def median_xyz(point_cloud, px, py, radius=4):
    pts = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            err, val = point_cloud.get_value(px + dx, py + dy)
            if err == sl.ERROR_CODE.SUCCESS and valid_xyz(val):
                pts.append(val[:3])

    if not pts:
        return None

    arr = np.array(pts, dtype=np.float32)
    return np.median(arr, axis=0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def estimate_distance_m(bbox_width_px, real_width_m, focal_px):
    if bbox_width_px <= 0:
        return None
    return (real_width_m * focal_px) / bbox_width_px


def moonraker_headers(api_key=None):
    headers = {"Content-Type": "application/json"}
    if api_key:
        headers["X-Api-Key"] = api_key
    return headers


def send_safety_pause(base_url, api_key=None, timeout=2.0):
    """Call SAFETY_PAUSE macro via Moonraker gcode script endpoint."""
    url = f"{base_url.rstrip('/')}/printer/gcode/script"
    payload = {"script": "SAFETY_PAUSE"}
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=moonraker_headers(api_key),
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = resp.read().decode("utf-8", errors="ignore")
        return resp.status, body

def query_safety_state(base_url, api_key=None, timeout=2.0):
    """Returns 'paused' if SAFETY_STATE.paused is True, else 'resumed'."""
    query_url = f"{base_url.rstrip('/')}/printer/objects/query"
    payload = {
        "objects": {
            "gcode_macro SAFETY_STATE": None
        }
    }
    req = urllib.request.Request(
        query_url,
        data=json.dumps(payload).encode("utf-8"),
        headers=moonraker_headers(api_key),
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="ignore")
        data = json.loads(raw)
    paused = data["result"]["status"].get("gcode_macro SAFETY_STATE", {}).get("paused", False)
    return "paused" if paused else "resumed"


def query_box_state(base_url, api_key=None, timeout=2.0):
    query_url = f"{base_url.rstrip('/')}/printer/objects/query"

    payload = {
        "objects": {
            "gcode_macro BOX_STATE": None
        }
    }

    req = urllib.request.Request(
        query_url,
        data=json.dumps(payload).encode("utf-8"),
        headers=moonraker_headers(api_key),
        method="POST",
    )

    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8", errors="ignore")
        data = json.loads(raw)

    status = data["result"]["status"].get("gcode_macro BOX_STATE", {})

    # read current Klipper fields
    box_code = status.get("box_code") or "NONE"
    box_id = int(status.get("box_id") or 0)

    return {
        "name": str(box_code),
        "selected": 1 if str(box_code) != "NONE" and box_id > 0 else 0,
        # ignored for now
        "length": 0.0,
        "width": 0.0,
        "height": 0.0,
    }

def normalize_detected_box_name(det_name: str):
    return BOX_NAME_MAP.get(det_name, det_name)
