"""Fake `vex` module: runs src/main.py on a laptop against a simulated field.

Only what main.py uses is modelled. Time is simulated: wait() advances the
clock and moves the motors, so a 10 minute mission runs in seconds.
Use it through tools/sim/run.py.
"""
import random

# ---- simulated world (run.py may overwrite these before main.py runs) -------
SIM = {
    "k_x": 0.10,
    "k_z": 0.10,
    "x_min": -1250.0,      # physical positions, independent of encoder zero
    "x_max": 0.0,
    "x_start": -200.0,
    "z_max": 420.0,
    "dist_dx": 0.0,        # beam X minus gripper X; the same sign as DIST_DX_MM
    "grab_reading": 10.0,  # true distance reading in the grab pose
    "hold_reading": 8.0,   # reading while a cube is held
    "beam_half": 0.0,      # ideal narrow beam; test wider beams separately
    "noise": 1.0,          # distance noise sigma, mm
    "wall_z": 350.0,       # mining area back wall, grab-Z frame
    "cube": 75.0,
    "cubes": [],           # dicts: x, z, colour, held, placed
    "port_x": 5, "port_z": 1, "port_grip": 9,
    "trace": False,
    "home_mode": "normal",  # normal, missing, stuck, never
    "grip_inner": 89.0,
    "grip_outer": 100.0,
    "grip_depth": 80.0,
}

_t = [0]           # ms
_motors = {}


def _advance(ms):
    steps = max(1, int(ms // 5))
    for _ in range(steps):
        _t[0] += ms / float(steps)
        for m in _motors.values():
            m._step(ms / float(steps))
        _grip_logic()


def _mm(port):
    m = _motors.get(port)
    if m is None:
        return 0.0
    k = SIM["k_x"] if port == SIM["port_x"] else SIM["k_z"]
    return m._pos * k


def _held():
    for c in SIM["cubes"]:
        if c.get("held"):
            return c
    return None


_grip_state = ["closed"]


def _grip_logic():
    g = _motors.get(SIM["port_grip"])
    if g is None:
        return
    x, z = _mm(SIM["port_x"]), _mm(SIM["port_z"])
    held = _held()
    if held is not None:
        held["x"], held["z"] = x, z
    if _grip_state[0] == "open" and g._pos > -60:
        _grip_state[0] = "closed"
        for c in SIM["cubes"]:
            centred = abs(c["x"] - x) <= (SIM["grip_inner"] - SIM["cube"]) / 2
            blocked = any(other is not c and not other.get("held")
                          and abs(other["x"] - x) < (SIM["grip_outer"] + SIM["cube"]) / 2
                          and other["z"] < z + SIM["grip_depth"]
                          for other in SIM["cubes"])
            if not c.get("held") and centred and not blocked and abs(c["z"] - z) < 15:
                c["held"] = True
                break
    elif _grip_state[0] == "closed" and g._pos < -300:
        _grip_state[0] = "open"
        if held is not None:
            held["held"] = False
            held["placed"] = True


# ---- constants ---------------------------------------------------------------
class Ports:
    pass


for _i in range(1, 13):
    setattr(Ports, "PORT%d" % _i, _i)

DEGREES, TURNS = "deg", "turns"
PERCENT, RPM, DPS = "pct", "rpm", "dps"
MSEC, SECONDS = "ms", "s"
MM, INCHES = "mm", "in"
FORWARD, REVERSE = 1, -1
HOLD, BRAKE, COAST = "hold", "brake", "coast"


class Color:
    RED, GREEN, BLUE, YELLOW, PURPLE, WHITE, BLACK, ORANGE = (
        "red", "green", "blue", "yellow", "purple", "white", "black", "orange")


class LedStateType:
    ON, OFF = "on", "off"


def wait(t, units=MSEC):
    _advance(t * 1000 if units == SECONDS else t)


# ---- devices -----------------------------------------------------------------
class Motor:
    MAX_DPS = 720.0

    def __init__(self, port, *args):
        self.port = port
        self._pos = SIM["x_start"] / SIM["k_x"] if port == SIM["port_x"] else 0.0
        self._zero = 0.0
        self._vel = 0.0         # actual deg/s
        self._mode = "idle"
        self._target = 0.0
        self._speed = 50.0
        self._timeout = 10000
        self._t0 = 0
        self._done = True
        _motors[port] = self

    def _limits(self):
        if self.port == SIM["port_x"]:
            return (SIM["x_min"] / SIM["k_x"], SIM["x_max"] / SIM["k_x"])
        if self.port == SIM["port_z"]:
            return (-5.0 / SIM["k_z"], SIM["z_max"] / SIM["k_z"])
        return (-600.0, 20.0)

    def _step(self, ms):
        dps = self._speed / 100.0 * self.MAX_DPS
        old = self._pos
        if self._mode == "pos":
            err = self._target - self._pos
            stepd = dps * ms / 1000.0
            if abs(err) <= stepd:
                self._pos = self._target
                self._done = True
                self._mode = "idle"
            else:
                self._pos += stepd if err > 0 else -stepd
            if _t[0] - self._t0 > self._timeout:
                self._done = True
                self._mode = "idle"
        elif self._mode == "spin":
            self._pos += self._dir * dps * ms / 1000.0
        lo, hi = self._limits()
        self._pos = min(max(self._pos, lo), hi)
        if self._mode == "pos" and self._pos in (lo, hi) and self._pos != self._target:
            self._done = True       # stalled on a hard stop
            self._mode = "idle"
        self._vel = (self._pos - old) * 1000.0 / ms if ms > 0 else 0.0

    def installed(self):
        return True

    def set_stopping(self, mode):
        pass

    def set_timeout(self, value, units=MSEC):
        self._timeout = value

    def set_position(self, value, units=DEGREES):
        self._zero = self._pos - float(value)

    def position(self, units=DEGREES):
        return self._pos - self._zero

    def velocity(self, units=PERCENT):
        return self._vel / self.MAX_DPS * 100.0

    def spin(self, direction, speed=50, units=PERCENT):
        self._mode = "spin"
        self._dir = direction * (1 if speed >= 0 else -1)
        self._speed = abs(speed)

    def spin_to_position(self, rotation, units=DEGREES, velocity=50, units_v=PERCENT, wait=True):
        self._mode = "pos"
        self._target = float(rotation) + self._zero
        self._speed = abs(velocity)
        self._t0 = _t[0]
        self._done = False
        while wait and not self._done:
            _advance(10)

    def is_done(self):
        return self._done

    def stop(self, mode=None):
        self._mode = "idle"
        self._done = True


class Distance:
    def __init__(self, port):
        pass

    def installed(self):
        return True

    def object_distance(self, units=MM):
        if _held() is not None:
            return SIM["hold_reading"] + random.gauss(0, SIM["noise"] * 0.3)
        x = _mm(SIM["port_x"]) + SIM["dist_dx"]
        z = _mm(SIM["port_z"])
        near = SIM["wall_z"]
        for c in SIM["cubes"]:
            if abs(c["x"] - x) <= SIM["cube"] / 2 + SIM["beam_half"] and c["z"] >= z - 5:
                near = min(near, c["z"])
        return max(0.0, near - z + SIM["grab_reading"] + random.gauss(0, SIM["noise"]))


class Optical:
    HUES = {"red": 8.0, "green": 125.0, "blue": 225.0}

    def __init__(self, port):
        pass

    def installed(self):
        return True

    def set_light(self, *args):
        pass

    def is_near_object(self):
        return _held() is not None

    def hue(self):
        c = _held()
        if c is None:
            return random.uniform(0, 360)
        return (self.HUES[c["colour"]] + random.gauss(0, 6)) % 360

    def brightness(self, readraw=False):
        return 40.0 if _held() is not None else 2.0


class Bumper:
    def __init__(self, port):
        pass

    def installed(self):
        return SIM["home_mode"] != "missing"

    def pressing(self):
        if SIM["home_mode"] != "normal":
            return SIM["home_mode"] == "stuck"
        return _mm(SIM["port_x"]) >= SIM["x_max"] - 0.2


class Touchled:
    """Pressed for 200 ms every 400 ms, so 'press to start' passes."""

    def __init__(self, port):
        pass

    def installed(self):
        return True

    def set_color(self, c):
        pass

    def off(self):
        pass

    def pressing(self):
        return _t[0] > 2500 and _t[0] % 400 < 200     # not held at start-up


class _Button:
    def pressing(self):
        return False


class _Screen:
    def __getattr__(self, name):
        return lambda *a, **k: None


class _Timer:
    def time(self, units=MSEC):
        return _t[0] if units == MSEC else _t[0] / 1000.0

    def clear(self):
        _t[0] = 0


class _Battery:
    def capacity(self, *a):
        return 90


class Brain:
    def __init__(self):
        self.screen = _Screen()
        self.timer = _Timer()
        self.battery = _Battery()
        self.buttonLeft = _Button()
        self.buttonRight = _Button()
        self.buttonCheck = _Button()
