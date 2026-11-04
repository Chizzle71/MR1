# gamepad_driver uses the inputs library to connect with a wired or bluetooth Xbox One controller and read stick inputs
# deadzone set by parent function (how far stick has to be moved before registering input)
# currently just returns up/down/left/right dir on left stick
# runs as daemon on user laptop -> UDP connection to rover, but can also run natively on pi using bluetooth
# supported by simulator

# imports
from inputs import get_gamepad
import time
import threading

class ControllerDriver:
    # start() initialises a thread on which _loop runs continually scanning for events via get_gamepad()
    # This currently is just the left stick X/Y pos (after a set deadband value for error elimination)
    # _compute_dir finds the absolute direction -> sets forward/backward/left/right or None for neutral stick

    def __init__(self, deadzone: int = 8000):
        self.deadzone = int(deadzone)  
        self._running = False
        self._thread = None
        self._dir = None 
        self._x_raw = 0
        self._y_raw = 0

    def _compute_dir(self, x_raw: int, y_raw: int):
        # deadzone is radius of circle where inputs will be ignored until
        ax, ay = abs(x_raw), abs(y_raw)
        if ax < self.deadzone and ay < self.deadzone:
            return None
        if ax >= ay:
            return "right" if x_raw > 0 else "left"
        # most controllers report +Y as down
        return "backward" if y_raw > 0 else "forward"

    def _loop(self):
        while self._running:
            try:
                events = get_gamepad()  # blocking until events arrive
                for e in events:
                    if e.code == "ABS_X":
                        self._x_raw = int(e.state)
                    elif e.code == "ABS_Y":
                        self._y_raw = int(e.state)
                self._dir = self._compute_dir(self._x_raw, self._y_raw)
            except Exception:
                time.sleep(0.01)  

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        # daemon -> background thread so controller can be used with any window open
        self._thread.start()

    def read_dir(self):
        # Return direction value, can be changed to accomodate diagonals -> last direction
        """Return 'forward','backward','left','right' or None."""
        return self._dir

    def stop(self):
        self._running = False
