# ui_cli.py — Ground-station CLI for sending 1:1 rover commands over UDP.
#
# Now supports the expanded direction set:
#   f, b, l, r,
#   pl, pr,
#   fpl, fpr, bpl, bpr,
#   fl, fr, bl, br
#
# Keyboard bindings (from kb_driver.py):
#   W / UpArrow  -> f      |  S / DownArrow -> b
#   LeftArrow    -> l      |  RightArrow    -> r
#   A -> pl      |  D -> pr
#   Q -> fpl     |  E -> fpr
#   Z -> bpl     |  X -> bpr
#   U -> fl      |  I -> fr
#   J -> bl      |  K -> br

import json
import socket
import time
import subprocess
import sys
from pathlib import Path

from drivers.gamepad_driver import ControllerDriver  # unchanged
from drivers.kb_driver import KeyboardDriverPynput   # updated file

SEND_HZ = 50.0
PING_HZ = 2.0
DEFAULT_SIM_ADDR = ("127.0.0.1", 9999)


def main():
    print("==== Rover UI ====")
    print("1) Launch Simulator (local)")
    print("2) Connect Rover")
    menu_choice = input("Select [1/2]: ").strip()

    print("\nInput source: Keyboard (k) or Controller (c)? ")
    mode = input("> ").strip().lower()
    driver = KeyboardDriverPynput() if mode.startswith("k") else ControllerDriver(deadzone=8000)

    # Network target
    sim_proc = None
    if menu_choice == "1":
        target = DEFAULT_SIM_ADDR
        # try to launch simulator alongside this script
        project_root = Path(__file__).resolve().parent
        sim_script = project_root / "kinematic_sim.py"
        if not sim_script.exists():
            print(f"ERROR: cannot find {sim_script}")
            return
        sim_proc = subprocess.Popen([sys.executable, str(sim_script)], cwd=str(project_root))
        print(f"Simulator launched (PID {sim_proc.pid})")
        time.sleep(0.5)
    else:
        host = input("Enter rover/sim IP address: ").strip()
        port = int(input("Enter UDP port [9999]: ") or 9999)
        target = (host, port)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.0)

    driver.start()

    # Help banner
    print("\nControls:")
    print("  W/Up -> f   |  S/Down -> b")
    print("  Left -> l   |  Right  -> r")
    print("  A -> pl     |  D -> pr")
    print("  Q -> fpl    |  E -> fpr")
    print("  Z -> bpl    |  X -> bpr")
    print("  U -> fl     |  I -> fr")
    print("  J -> bl     |  K -> br")
    print(f"\nSending to {target[0]}:{target[1]}   Input: {'Keyboard' if isinstance(driver, KeyboardDriverPynput) else 'Controller'}")

    seq = 0
    dt_send = 1.0 / SEND_HZ
    dt_ping = 1.0 / PING_HZ
    next_ping = time.time()

    last_ping_seq = None
    last_ping_ts = 0.0
    last_pong_ts = 0.0
    last_ping_ms = None
    last_status = 0.0

    try:
        while True:
            now = time.time()

            # get current direction (driver returns compact string or None)
            d = driver.read_dir()
            msg = {"type": "cmd", "seq": seq, "ts": now, "dir": d, "ping_ms": last_ping_ms}
            sock.sendto(json.dumps(msg).encode(), target)
            seq += 1

            # ping
            if now >= next_ping:
                last_ping_seq = seq
                last_ping_ts = now
                ping = {"type": "ping", "seq": last_ping_seq, "ts": last_ping_ts}
                sock.sendto(json.dumps(ping).encode(), target)
                next_ping = now + dt_ping

            # read pongs
            try:
                while True:
                    data, _ = sock.recvfrom(2048)
                    obj = json.loads(data.decode())
                    if obj.get("type") == "pong" and obj.get("seq") == last_ping_seq:
                        rtt_ms = (time.time() - obj.get("ts", last_ping_ts)) * 1000.0
                        last_ping_ms = round(rtt_ms, 1)
                        last_pong_ts = time.time()
            except BlockingIOError:
                pass
            except Exception:
                pass

            # status once per second
            if now - last_status >= 1.0:
                if last_pong_ts and (now - last_pong_ts) < 2.0:
                    print(f"status: connected - ping = {last_ping_ms:.1f} ms")
                else:
                    print("status: disconnected")
                last_status = now

            time.sleep(dt_send)
    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()
        if sim_proc:
            try:
                sim_proc.terminate()
            except Exception:
                pass
        print("\nUI exited.")


if __name__ == "__main__":
    main()
