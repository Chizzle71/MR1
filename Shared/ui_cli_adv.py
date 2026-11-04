"""
ui_cli.py — Ground-station CLI for sending rover commands over UDP.

 Modes:
   1) Launch Simulator (local)  — starts kinematic_sim.py and streams inputs
   2) Connect Rover             — streams inputs to a host:port
   3) Command Console           — parse text commands and send as "program"

Command Console syntax:
   MOVE <mode> [radius]
   MODE <time|distance> <value>
   DIR  <f|b|l|r|fl|fr|bl|br>
   SPEED <0-1>

Examples:
   MOVE crab MODE time 5000 DIR fr SPEED 0.5
   MOVE point 5 MODE distance 10 DIR fl SPEED 0.5

""" 


import json
import socket
import time
import subprocess
import sys
from pathlib import Path

from drivers.gamepad_driver import ControllerDriver  # if you want gamepad mode
from drivers.kb_driver import KeyboardDriverPynput   # updated keyboard driver

SEND_HZ = 50.0
PING_HZ = 2.0
DEFAULT_SIM_ADDR = ("127.0.0.1", 9999)

# Acceptable direction tokens in command console 
VALID_DIRS = {"f","b","l","r","fl","fr","bl","br"}

def parse_move_command(line: str):
    """
    Parse a command-line string for packets

    Expected grammar:
      MOVE <crab|point> [radius]
      MODE <time|distance> <value>
      DIR  <f|b|l|r|fl|fr|bl|br>
      SPEED <0-1>

    Returns: (program_dict, error_string)
    """
    toks = line.strip().split()
    if not toks:
        return None, "empty line"

    # normalize to lowercase except numeric tokens
    t = [tok if tok.replace('.', '', 1).isdigit() else tok.lower() for tok in toks]

    prog = {
        "move": None,         # "crab" or "point"
        "radius_m": None,     # float, only for point
        "mode": None,         # "time" or "distance"
        "time_ms": None,      # int ms if mode=time
        "distance_m": None,   # float if mode=distance
        "dir": None,          # compact direction f/b/l/r/fl/fr/bl/br
        "speed": None,        # 0-1
    }

    i = 0
    try:
        while i < len(t):
            tok = t[i]
            if tok == "move":
                if i+1 >= len(t): return None, "MOVE missing parameter"
                kind = t[i+1]
                if kind not in ("crab", "point"):
                    return None, f"unknown MOVE mode '{kind}'"
                prog["move"] = kind
                i += 2
                if kind == "point":
                    if i >= len(t): return None, "MOVE point needs radius"
                    # radius can follow immediately after 'point'
                    try:
                        prog["radius_m"] = float(t[i])
                        i += 1
                    except ValueError:
                        return None, "invalid radius for MOVE point"
            elif tok == "mode":
                if i+2 >= len(t): return None, "MODE needs <time|distance> <value>"
                m = t[i+1]
                v = t[i+2]
                if m not in ("time","distance"):
                    return None, f"unknown MODE '{m}'"
                prog["mode"] = m
                if m == "time":
                    # milliseconds
                    if not v.replace('.','',1).isdigit():
                        return None, "MODE time expects milliseconds"
                    prog["time_ms"] = int(float(v))
                else:
                    # meters (or your world units)
                    prog["distance_m"] = float(v)
                i += 3
            elif tok == "dir":
                if i+1 >= len(t): return None, "DIR needs a value"
                d = t[i+1]
                if d not in VALID_DIRS:
                    return None, f"DIR must be one of {sorted(VALID_DIRS)}"
                prog["dir"] = d
                i += 2
            elif tok == "speed":
                if i+1 >= len(t): return None, "SPEED needs a value"
                sp = float(t[i+1])
                if not (0.0 <= sp <= 1.0):
                    return None, "SPEED must be in [0..1]"
                prog["speed"] = sp
                i += 2
            else:
                return None, f"unknown token '{tok}'"
    except Exception as e:
        return None, f"parse error: {e}"

    
    missing = [k for k in ("move","mode","dir","speed") if prog[k] is None]
    if prog["move"] == "point" and prog["radius_m"] is None:
        missing.append("radius")
    if prog["mode"] == "time" and prog["time_ms"] is None:
        missing.append("time_ms")
    if prog["mode"] == "distance" and prog["distance_m"] is None:
        missing.append("distance_m")
    if missing:
        return None, "missing fields: " + ", ".join(missing)

    return prog, None


def run_stream_mode(target, driver):
    """Existing streaming loop: send 'cmd' packets with dir at SEND_HZ."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.0)

    driver.start()
    print(f"\nStreaming to {target[0]}:{target[1]}   Input: {'Keyboard' if isinstance(driver, KeyboardDriverPynput) else 'Controller'}")
    print("Press Ctrl+C to stop.\n")

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

            d = driver.read_dir()  # compact string or None
            msg = {"type": "cmd", "seq": seq, "ts": now, "dir": d, "ping_ms": last_ping_ms}
            sock.sendto(json.dumps(msg).encode(), target)
            seq += 1

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
        print("\nStream mode ended.")


def run_command_console(target):
    """Interactive command mode: parse lines and send as 'program' packets."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.0)
    seq = 0

    print("\n=== Command Console ===")
    print("Enter commands like:")
    print("  MOVE crab MODE time 5000 DIR fr SPEED 0.5")
    print("  MOVE point 5 MODE distance 10 DIR fl SPEED 0.5")
    print("Type 'help' to see this again, or 'quit' to exit.\n")

    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting console.")
            break

        if not line:
            continue
        if line.lower() in ("quit","exit"):
            break
        if line.lower() == "help":
            print("MOVE <crab|point> [radius]")
            print("MODE <time|distance> <value>")
            print("DIR  <f|b|l|r|fl|fr|bl|br>")
            print("SPEED <0..1>\n")
            continue

        program, err = parse_move_command(line)
        if err:
            print(f"ERROR: {err}")
            continue

        pkt = {"type": "program", "seq": seq, "ts": time.time(), "program": program}
        sock.sendto(json.dumps(pkt).encode(), target)
        seq += 1
        print("Sent program:", program)


def main():
    print("==== Rover UI ====")
    print("1) Launch Simulator (local)")
    print("2) Connect Rover")
    print("3) Command Console")
    menu_choice = input("Select [1/2/3]: ").strip()

    # choose input source for stream modes
    driver = None
    if menu_choice in ("1","2"):
        print("\nInput source: Keyboard (k) or Controller (c)? ")
        mode = input("> ").strip().lower()
        driver = KeyboardDriverPynput() if mode.startswith("k") else ControllerDriver(deadzone=8000)

    # Network target
    sim_proc = None
    if menu_choice == "1":
        target = DEFAULT_SIM_ADDR
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

    try:
        if menu_choice in ("1","2"):
            run_stream_mode(target, driver)
        elif menu_choice == "3":
            run_command_console(target)
        else:
            print("Invalid selection.")
    finally:
        if sim_proc:
            try:
                sim_proc.terminate()
            except Exception:
                pass
        print("\nUI exited.")


if __name__ == "__main__":
    main()
