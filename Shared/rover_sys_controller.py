# Rover system/network controller script
# UDP-controlled rover controller that works with the UI / simulator protocol.

# imports
import json
import socket
import time
from typing import Optional

from drivers.motor_code import MotorController  # your existing motor driver

# variables
LISTEN_PORT = 9999          # change to dynamic later, default val
LOOP_HZ = 100               # motor update loop rate
DEADMAN_S = 0.5             # stop if no commands for this many seconds
SPEED_INPUT = 0.5           # base throttle speed is 0.5, maybe add acceleration feature?
STATUS_PRINT_HZ = 1         # how often to print connection details

# main class
class UdpCmdServer:
    """
    Minimal UDP server:
      receives 'cmd' packets with dir and optional ping_ms
      replies to 'ping' with 'pong'
      stores last direction and last-heard time
    """
    def __init__(self, port: int):
        # set net details
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)
        self.last_dir: Optional[str] = None
        self.last_rx_ts: float = 0.0
        self.latest_ping_ms: Optional[float] = None
        print(f"[UDP] Listening on :{self.port}")

    def poll(self):
        # finish queue/process as string
        """Process any pending UDP packets (timeout 100ms)"""
        try:
            data, addr = self.sock.recvfrom(2048)
        except socket.timeout:
            return

        try:
            obj = json.loads(data.decode("utf-8"))
        except Exception:
            return

        typ = obj.get("type")
        if typ == "cmd":
            d = obj.get("dir")
            # accept forward/backward/left/right/None
            if d in (None, "forward", "backward", "left", "right"):
                self.last_dir = d
                self.last_rx_ts = time.time()
                # pass-through of UI's last measured ping (optional)
                pm = obj.get("ping_ms")
                if isinstance(pm, (int, float)):
                    self.latest_ping_ms = float(pm)

        elif typ == "ping":
            # echo back a pong so UI can measure round-trip time
            pong = {"type": "pong", "seq": obj.get("seq"), "ts": obj.get("ts")}
            try:
                self.sock.sendto(json.dumps(pong).encode("utf-8"), addr)
            except Exception:
                pass

    def age(self) -> float:
        """Seconds since last valid command was received"""
        if self.last_rx_ts == 0.0:
            return float("inf")
        return time.time() - self.last_rx_ts


def netdir_to_motor_dir(net_dir: Optional[str]) -> str:
    """
    Map network directions to MotorCode strings
    Network uses: forward/backward/left/right/None
    MotorController expects: fwd/bwd/l/r/stop
    """
    # maybe update motor_code to use the same terminology?
    if net_dir == "forward":
        return "fwd"
    if net_dir == "backward":
        return "bwd"
    if net_dir == "left":
        return "l"
    if net_dir == "right":
        return "r"
    return "stop"


def main():
    # Instantiate motor controller, start in stop
    mc = MotorController(motor_speed=SPEED_INPUT, direction="stop")
    current_dir = "stop"

    # UDP server
    udp = UdpCmdServer(LISTEN_PORT)

    dt = 1.0 / LOOP_HZ
    next_status = time.time()

    try:
        while True:
            # 1) handle incoming UDP (cmds/pings)
            udp.poll()

            # 2) deadman check -> check if sender has stopped transmitting -> STOP motors
            connected = udp.age() <= DEADMAN_S
            desired_net_dir = udp.last_dir if connected else None
            desired_motor_dir = netdir_to_motor_dir(desired_net_dir)

            # 3) update motors if direction changed or to keep them refreshed
            if desired_motor_dir != current_dir:
                current_dir = desired_motor_dir
                s = mc.calc_motor_speed(SPEED_INPUT, current_dir)
                mc.set_speed(*s)
            else:
                pass

            # 4) print status
            now = time.time()
            if now >= next_status:
                if connected and udp.latest_ping_ms is not None:
                    print(f"[status] connected - ping = {udp.latest_ping_ms:.1f} ms   dir={desired_motor_dir}")
                else:
                    print("[status] disconnected   dir=stop")
                next_status = now + (1.0 / max(1, STATUS_PRINT_HZ))

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        # eg. run every interrupt
        # stop motors and cleanup GPIO on exit
        try:
            mc.set_speed(*mc.calc_motor_speed(SPEED_INPUT, "stop"))
        except Exception:
            pass
        mc.cleanup()

# Startup program
if __name__ == "__main__":
    main()
