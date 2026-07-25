import json
import urllib.request


def moonraker_headers(api_key=None):
    headers = {"Content-Type": "application/json"}
    if api_key:
        headers["X-Api-Key"] = api_key
    return headers


def send_gcode(base_url, script: str, api_key=None, timeout=3.0):
    url = f"{base_url.rstrip('/')}/printer/gcode/script"
    payload = {"script": script}
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=moonraker_headers(api_key),
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read().decode("utf-8", errors="ignore")


def cmd_home_all(base_url, api_key=None):
    return send_gcode(base_url, "HOME_ALL_CUSTOM", api_key)


def cmd_init_all(base_url, api_key=None):
    return send_gcode(base_url, "INIT_ALL", api_key)


def cmd_enable_all(base_url, api_key=None):
    return send_gcode(base_url, "ENABLE_ALL", api_key)


def cmd_disable_all(base_url, api_key=None):
    return send_gcode(base_url, "DISABLE_ALL", api_key)


def cmd_forward(base_url, dist_mm=20.0, speed=10.0, accel=80.0, api_key=None):
    return send_gcode(
        base_url,
        f"Y_FORWARD DIST={dist_mm:.2f} SPEED={speed:.2f} ACCEL={accel:.2f}",
        api_key,
    )


def cmd_backward(base_url, dist_mm=20.0, speed=10.0, accel=80.0, api_key=None):
    return send_gcode(
        base_url,
        f"Y_BACKWARD DIST={dist_mm:.2f} SPEED={speed:.2f} ACCEL={accel:.2f}",
        api_key,
    )


def cmd_right(base_url, dist_mm=50.0, speed=30.0, accel=80.0, api_key=None):
    return send_gcode(
        base_url,
        f"X_FORWARD DIST={dist_mm:.2f} SPEED={speed:.2f} ACCEL={accel:.2f}",
        api_key,
    )


def cmd_left(base_url, dist_mm=50.0, speed=30.0, accel=80.0, api_key=None):
    return send_gcode(
        base_url,
        f"X_BACKWARD DIST={dist_mm:.2f} SPEED={speed:.2f} ACCEL={accel:.2f}",
        api_key,
    )


def cmd_turn_cw(base_url, deg=90.0, speed=40.0, accel=100.0, api_key=None):
    return send_gcode(
        base_url,
        f"DISK_CW DIST={deg:.2f} SPEED={speed:.2f} ACCEL={accel:.2f}",
        api_key,
    )


def cmd_turn_ccw(base_url, deg=90.0, speed=40.0, accel=100.0, api_key=None):
    return send_gcode(
        base_url,
        f"DISK_CCW DIST={deg:.2f} SPEED={speed:.2f} ACCEL={accel:.2f}",
        api_key,
    )
