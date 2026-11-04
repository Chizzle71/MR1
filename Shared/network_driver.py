# network_driver.py
# Receiver-side program runner for high-level MOVE commands.
#
# Usage on the rover/sim:
#   runner = ProgramRunner(max_speed_mps=1.5)   # or your real max linear m/s
#   ...
#   # when a packet arrives with obj["type"] == "program":
#   runner.ingest(obj["program"])
#   ...
#   # each tick:
#   active, dir_code, speed = runner.sample()
#   if active:
#       # dir_code is compact: f,b,l,r,pl,pr,fl,fr,bl,br,fpl,fpr,bpl,bpr
#       # speed is 0..1 (normalized throttle)
#       # For your 1:1 sim: virt.compute_for_sim(speed, dir_code) -> apply
#       pass

import math
import time


# Map console DIR tokens to motor_code4 compact direction labels
# (console uses f,b,l,r,fl,fr,bl,br; we also keep pivots pl/pr and diagonals)
DIR_MAP = {
    "f": "f", "b": "b",
    "l": "pl", "r": "pr",           # pivot left/right
    "pl": "pl", "pr": "pr",
    "fl": "fl", "fr": "fr", "bl": "bl", "br": "br",
    "fpl": "fpl", "fpr": "fpr", "bpl": "bpl", "bpr": "bpr",
}

class ProgramRunner:
    """
    Turns high-level MOVE programs into timed low-level commands.

    Interprets program dict:
      {
        "move": "crab" | "point",
        "radius_m": float or None,
        "mode": "time" | "distance",
        "time_ms": int or None,
        "distance_m": float or None,
        "dir": "f"|"b"|"l"|"r"|"fl"|"fr"|"bl"|"br",
        "speed": float in [0..1]
      }
    """
    def __init__(self, max_speed_mps=1.0):
        self.max_speed_mps = float(max_speed_mps)
        self._active = False
        self._start = 0.0
        self._end = 0.0
        self._dir = None
        self._speed = 0.0
        self._meta = {}

    def ingest(self, program: dict):
        """Load a new program and compute end time if needed."""
        move = program.get("move")
        mode = program.get("mode")
        d    = program.get("dir")
        v    = float(program.get("speed", 0.0))
        radius = program.get("radius_m", None)
        time_ms = program.get("time_ms")
        dist_m  = program.get("distance_m")

        # normalize direction to motor_code4 labels
        dir_code = DIR_MAP.get(d)
        if not dir_code:
            return False, f"Unknown DIR '{d}'"

        # compute duration
        if mode == "time":
            if time_ms is None:
                return False, "MODE time missing time_ms"
            duration_s = max(0.0, float(time_ms) / 1000.0)
        elif mode == "distance":
            if dist_m is None:
                return False, "MODE distance missing distance_m"
            # path length is straight for crab; arc length for point
            if move == "point":
                if radius is None or radius <= 0:
                    return False, "MOVE point missing valid radius"
                # distance_m is *path length along arc*; time = s / linear_speed
                path_m = float(dist_m)
            else:  # crab
                path_m = float(dist_m)
            # Convert normalized speed to world m/s using this node's max speed
            lin_mps = max(0.0, min(1.0, v)) * self.max_speed_mps
            if lin_mps <= 1e-6:
                return False, "SPEED too small for distance mode"
            duration_s = path_m / lin_mps
        else:
            return False, f"Unknown MODE '{mode}'"

        now = time.time()
        self._active = True
        self._start = now
        self._end = now + duration_s
        self._dir = dir_code
        self._speed = float(v)
        self._meta = {
            "move": move,
            "mode": mode,
            "radius_m": radius,
            "duration_s": duration_s,
        }
        return True, None

    def sample(self):
        """
        Returns (active, dir_code, speed) each tick.
        When inactive, returns (False, None, 0.0).
        """
        if not self._active:
            return False, None, 0.0
        if time.time() >= self._end:
            self._active = False
            return False, None, 0.0
        return True, self._dir, self._speed

    def cancel(self):
        self._active = False

    def status(self):
        if not self._active:
            return {"active": False}
        remain = max(0.0, self._end - time.time())
        s = {"active": True, "dir": self._dir, "speed": self._speed,
             "remaining_s": remain}
        s.update(self._meta)
        return s
