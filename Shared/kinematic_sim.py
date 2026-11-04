"""
kinematic_sim.py

2D Rigid body kinematic simulator of 6-wheel rover dynamics using pygame.
Constraint conditions set per wheel, solved in a system of equations using
least squares to compute overall motion.

Modes:
- Sandbox: keyboard tuning of per-wheel speeds/angles + "driving" toggle
- Network: legacy simple net mapping (forward/backward/left/right + reset/stop)
- 1:1 mode: UDP commands -> motor_code4 math -> six wheel speeds/angles -> sim

Computes ICR and turn radius each step.

Math:
Each wheel i has steering angle θ_i and scalar wheel speed v_i along its axis.
Let u_i (axis), n_i (lateral), r_i (wheel position in body frame).

Constraints (no slip laterally, rolling along axis):
  u_i^T (v_b + ω×r_i) = v_i
  n_i^T (v_b + ω×r_i) = 0

Stack for 6 wheels → A ξ = b (12x3). Solve least squares for ξ=[v_x_b,v_y_b,ω].
Rotate to world (yaw=0 faces up), integrate pose, compute ICR.
"""
from network_driver import ProgramRunner
runner = ProgramRunner(max_speed_mps=1.5)  # set to your best estimate

# imports
import json
import math
import socket
import time

import numpy as np
import pygame

# --- import motor math for 1:1 mode ---
# ensure motor_code4.py is on PYTHONPATH next to this file
from motor_code4 import MotorController, D1, D2, D3, D4

# pygame colour constants for UI
BG = (210, 220, 235)
NAVY = (0, 30, 80)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (80, 180, 255)
INDEX_BLUE = (70, 150, 255)
ORANGE = (255, 120, 60)
RED = (200, 20, 20)
SLIDER_BG = (210, 210, 215)
SLIDER_FG = (50, 120, 220)
SLIDER_KNOB = (20, 20, 25)

# screen and object geometry constants
INIT_W, INIT_H = 1400, 1000
WHEEL_L, WHEEL_W = 78.0, 36.0
ARROW_LEN, HEAD_LEN = 90.0, 120.0
BODY_L, BODY_W = 360.0, 220.0

x_front_wheels, x_mid_wheels, x_rear_wheels = +BODY_L / 2, 0.0, -BODY_L / 2
y_left_wheels, y_right_wheels = +BODY_W / 2, -BODY_W / 2

wheel_pos = np.array(
    [
        [x_front_wheels, y_left_wheels],   # 1 -> 0
        [x_mid_wheels, y_left_wheels],     # 2 -> 1
        [x_rear_wheels, y_left_wheels],    # 3 -> 2
        [x_front_wheels, y_right_wheels],  # 4 -> 3
        [x_mid_wheels, y_right_wheels],    # 5 -> 4
        [x_rear_wheels, y_right_wheels],   # 6 -> 5
    ],
    float,
)
ROW_PAIRS = ((0, 3), (1, 4), (2, 5))
ROW_NAMES = ("front (1&4)", "mid (2&5)", "rear (3&6)")
FRONT_PAIR, MID_PAIR, REAR_PAIR = (0, 3), (1, 4), (2, 5)

# physics and control constants
speed_step = 20.0
angle_step = math.radians(5.0)

ANGLE_MIN, ANGLE_MAX = -math.pi / 2, math.pi / 2
OMEGA_EPSILON = 1e-3
STEER_IDLE_S = 0.6

# network consts - UDP
UDP_HOST, UDP_PORT = "0.0.0.0", 9999
NET_TIMEOUT_S = 1.0

# window init. variables
win_w, win_h, font = INIT_W, INIT_H, None


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


# UI Parts
class Slider:
    def __init__(self, x, y, w, label, init=0.5):
        self.rect = pygame.Rect(x, y, w, 26)
        self.label = label
        self.value = clamp(init, 0.0, 1.0)
        self.dragging = False

    def _set_from_mouse(self, mx):
        self.value = clamp((mx - self.rect.x) / max(1, self.rect.w), 0.0, 1.0)

    def handle(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.dragging = True
                self._set_from_mouse(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._set_from_mouse(event.pos[0])

    def draw(self, screen, valueText):
        screen.blit(
            font.render(self.label, True, (30, 30, 30)),
            (self.rect.x, self.rect.y - 32),
        )
        pygame.draw.rect(screen, SLIDER_BG, self.rect, border_radius=7)
        fill = self.rect.copy()
        fill.w = int(self.rect.w * self.value)
        pygame.draw.rect(screen, SLIDER_FG, fill, border_radius=7)
        kx = self.rect.x + int(self.rect.w * self.value)
        knob = pygame.Rect(kx - 9, self.rect.centery - 11, 18, 22)
        pygame.draw.rect(screen, SLIDER_KNOB, knob, border_radius=6)
        screen.blit(
            font.render(valueText, True, (30, 30, 30)),
            (self.rect.right + 12, self.rect.y - 2),
        )


class Button:
    def __init__(self, label):
        self.label = label
        self.rect = pygame.Rect(0, 0, 0, 0)

    def layout(self, rightX, y, pad=12):
        txt = font.render(self.label, True, (20, 20, 30))
        self.rect.size = (txt.get_width() + 2 * pad, txt.get_height() + 2 * pad)
        self.rect.topleft = (rightX - self.rect.w, y)

    def draw(self, screen, active=False):
        bgColor = (160, 210, 255) if active else (235, 240, 248)
        pygame.draw.rect(screen, bgColor, self.rect, border_radius=8)
        pygame.draw.rect(screen, (100, 110, 120), self.rect, 2, border_radius=8)
        txt = font.render(self.label, True, (20, 20, 30))
        screen.blit(
            txt,
            (
                self.rect.x + (self.rect.w - txt.get_width()) // 2,
                self.rect.y + (self.rect.h - txt.get_height()) // 2,
            ),
        )

    def hit(self, pos):
        return self.rect.collidepoint(pos)


# UDP server for net/1:1 modes
class UdpEndpoint:
    """UDP server that speaks ui_cli.py JSON protocol"""

    def __init__(self, host=UDP_HOST, port=UDP_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.lastHeard = 0.0
        self.lastFrom = None

    def recv_all(self):
        msgs = []
        while True:
            try:
                data, src = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                obj = json.loads(data.decode("utf-8", "ignore"))
            except Exception:
                continue
            self.lastHeard = time.time()
            self.lastFrom = src
            msgs.append(obj)
        return msgs

    def send_pong(self, seq, ts):
        if not self.lastFrom:
            return
        pkt = {"type": "pong", "seq": seq, "ts": ts}
        try:
            self.sock.sendto(json.dumps(pkt).encode(), self.lastFrom)
        except Exception:
            pass

    def is_connected(self):
        return (time.time() - self.lastHeard) < NET_TIMEOUT_S


# rover movement + swerve simulation main
class Sim:
    """Simple 6-wheel rigid body rover simulation"""

    def __init__(self):
        self.x = win_w / 2
        self.y = win_h / 2
        self.yaw = 0.0  # yaw=0 points UP on screen

        self.wheel_angles = np.zeros(6)  # θ_i (radians)
        self.wheel_speeds = np.zeros(6)  # along wheel axis (px/s)

        self.speed_drag = 1.2       # s^-1; exponential decay of wheel_speeds
        self.steer_drag = 60.0      # deg/s; steering returns to zero after idle
        self.max_speed = 240.0      # px/s cap per wheel

        self.last_steer_Ts = time.time()
        self.icr_world = None       # (x,y) of instantaneous center of rotation

        # for HUD in 1:1 mode
        self._net_connected = False

    def rot_body_to_world(self):
        psi = self.yaw + math.pi / 2
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, -s], [s, c]], float)

    def body_twist(self):
        rows = []
        rhs = []
        for i in range(6):
            th = self.wheel_angles[i]
            u = np.array([math.cos(th), math.sin(th)])
            n = np.array([-math.sin(th), math.cos(th)])
            r = wheel_pos[i]
            zxr = np.array([-r[1], r[0]])  # ω × r (2D)
            rows += [[u[0], u[1], float(u @ zxr)],
                     [n[0], n[1], float(n @ zxr)]]
            rhs += [self.wheel_speeds[i], 0.0]
        A = np.asarray(rows)
        b = np.asarray(rhs)
        xi, *_ = np.linalg.lstsq(A, b, rcond=None)
        return xi  # vx_b, vy_b, omega

    def step(self, dt):
        # simple drag for speed
        if self.speed_drag > 0:
            self.wheel_speeds *= max(0.0, 1.0 - self.speed_drag * dt)
            self.wheel_speeds[np.abs(self.wheel_speeds) < 0.02] = 0.0

        # steer return-to-center after idle
        if time.time() - self.last_steer_Ts >= STEER_IDLE_S:
            rate = math.radians(clamp(self.steer_drag, 0, 180))
            if rate > 0:
                dmax = rate * dt
                a = self.wheel_angles
                self.wheel_angles = np.clip(a + np.clip(-a, -dmax, dmax),
                                            ANGLE_MIN, ANGLE_MAX)
                self.wheel_angles[np.abs(self.wheel_angles)
                                  < math.radians(0.25)] = 0.0

        # compute body twist, convert to world, integrate pose
        vxB, vyB, omega = self.body_twist()
        R = self.rot_body_to_world()
        c, s = R[0, 0], R[1, 0]  # first column (cos,sin)
        vxW, vyW = c * vxB - s * vyB, s * vxB + c * vyB

        # ICR
        if abs(omega) > OMEGA_EPSILON:
            self.icr_world = (self.x + (-vyW) / omega, self.y + (vxW) / omega)
        else:
            self.icr_world = None

        self.x += vxW * dt
        self.y += vyW * dt
        self.yaw = wrap_pi(self.yaw + omega * dt)

        # keep inside; if out, recenter
        if not (0 <= self.x <= win_w and 0 <= self.y <= win_h):
            self.center()

    # ---------- sandbox helpers ----------
    def nudge_speed_all(self, delta):
        self.wheel_speeds[:] = np.clip(self.wheel_speeds + delta,
                                       -self.max_speed, self.max_speed)

    def counter_steer(self, sign):
        self.last_steer_Ts = time.time()
        for i in FRONT_PAIR:
            self.wheel_angles[i] = clamp(self.wheel_angles[i] + sign * angle_step,
                                         ANGLE_MIN, ANGLE_MAX)
        for i in REAR_PAIR:
            self.wheel_angles[i] = clamp(self.wheel_angles[i] - sign * angle_step,
                                         ANGLE_MIN, ANGLE_MAX)
        self.wheel_angles[list(MID_PAIR)] = 0.0

    def edit_steer(self, idxs, sign):
        self.last_steer_Ts = time.time()
        for i in idxs:
            self.wheel_angles[i] = clamp(self.wheel_angles[i] + sign * angle_step,
                                         ANGLE_MIN, ANGLE_MAX)

    def edit_speed(self, idxs, delta):
        for i in idxs:
            self.wheel_speeds[i] = clamp(self.wheel_speeds[i] + delta,
                                         -self.max_speed, self.max_speed)

    # ---------- 1:1 hook ----------
    def apply_virtual_outputs(self, speeds_norm, angles_rad):
        """
        Use hardware-equivalent wheel speeds/angles (after sign flips)
        to set the sim's wheels. Normalized speeds [-1,1] are mapped to px/s
        with self.max_speed.
        """
        a = np.asarray(angles_rad, float)
        s = np.asarray(speeds_norm, float) * float(self.max_speed)
        self.wheel_angles[:] = np.clip(a, ANGLE_MIN, ANGLE_MAX)
        self.wheel_speeds[:] = np.clip(s, -self.max_speed, self.max_speed)

    # misc.
    def center(self):
        self.x, self.y = win_w / 2, win_h / 2

    def reset(self):
        self.__init__()

    # DRAWING code for geometry
    def draw(self, screen, driving_mode, netConnected, selected_row, selected_wheel):
        screen.fill(BG)
        R = self.rot_body_to_world()

        # body polygon
        body = np.array(
            [
                [-BODY_L / 2, -BODY_W / 2],
                [BODY_L / 2, -BODY_W / 2],
                [BODY_L / 2, BODY_W / 2],
                [-BODY_L / 2, BODY_W / 2],
            ]
        )
        world = (body @ R.T) + np.array([self.x, self.y])
        poly = [(p[0], win_h - p[1]) for p in world]
        pygame.draw.polygon(screen, WHITE, poly, 0)
        pygame.draw.polygon(screen, BLACK, poly, 3)

        # center dot
        pygame.draw.circle(screen, INDEX_BLUE,
                           (int(self.x), int(win_h - self.y)), 8)

        # heading arrow
        psi = self.yaw + math.pi / 2
        nose = np.array([self.x, self.y]) + \
            np.array([math.cos(psi), math.sin(psi)]) * HEAD_LEN
        pygame.draw.line(screen, BLUE, (self.x, win_h - self.y),
                         (nose[0], win_h - nose[1]), 4)

        # ICR + dashed radius
        if self.icr_world:
            ix, iy = self.icr_world
            if -win_w * 5 < ix < win_w * 6 and -win_h * 5 < iy < win_h * 6:
                icr = (int(ix), int(win_h - iy))
                cen = (int(self.x), int(win_h - self.y))
                pygame.draw.circle(screen, RED, icr, 6)
                dash, gap = 20, 12
                dx, dy = icr[0] - cen[0], icr[1] - cen[1]
                dist = math.hypot(dx, dy)
                if dist > 1:
                    ux, uy = dx / dist, dy / dist
                    for k in range(int(dist // (dash + gap)) + 1):
                        s = (int(cen[0] + (dash + gap) * k * ux),
                             int(cen[1] + (dash + gap) * k * uy))
                        e = (int(s[0] + dash * ux), int(s[1] + dash * uy))
                        pygame.draw.line(screen, RED, s, e, 3)

        # which wheels to highlight
        if driving_mode:
            hi = set(FRONT_PAIR + REAR_PAIR)
        elif selected_wheel is not None:
            hi = {selected_wheel}
        else:
            hi = set(ROW_PAIRS[selected_row])

        # draw wheels + arrows + labels
        wheel_centers = []
        for i in range(6):
            r = wheel_pos[i]
            p = (r @ R.T) + np.array([self.x, self.y])
            th = self.wheel_angles[i]
            uBody = np.array([math.cos(th), math.sin(th)])
            uWorld = uBody @ R.T

            self._draw_wheel_box(screen, p, uWorld)
            p1 = (p[0], win_h - p[1])
            p2v = p + uWorld * ARROW_LEN
            p2 = (p2v[0], win_h - p2v[1])

            pygame.draw.circle(screen, ORANGE if i in hi else (60, 120, 220),
                               p1, 8)
            pygame.draw.line(screen, BLUE, p1, p2, 4)
            wheel_centers.append((i, p1))

        for i, p1 in wheel_centers:
            screen.blit(font.render(str(i + 1), True, INDEX_BLUE),
                        (p1[0] + 18, p1[1] - 18))

        # top HUD
        pad = 10
        t = font.render(self._hud_text, True, (255, 255, 255))
        hudBg = pygame.Rect(pad, pad, t.get_width() + 2 * pad,
                            t.get_height() + 2 * pad)
        pygame.draw.rect(screen, NAVY, hudBg, border_radius=6)
        screen.blit(t, (hudBg.x + pad, hudBg.y + pad))

        # connection badge
        t2 = font.render(
            f"Network: {'Connected' if netConnected else 'Disconnected'}",
            True, (255, 255, 255)
        )
        b2 = pygame.Rect(pad, hudBg.bottom + 8, t2.get_width() + 2 * pad,
                         t2.get_height() + 2 * pad)
        pygame.draw.rect(screen, NAVY, b2, border_radius=6)
        screen.blit(t2, (b2.x + pad, b2.y + pad))

        # bottom-right telemetry
        lines = [
            f"{i + 1}: angle={math.degrees(self.wheel_angles[i]):5.1f}°   "
            f"speed={self.wheel_speeds[i]:+6.1f} px/s"
            for i in range(6)
        ]
        rs = [font.render(s, True, (255, 255, 255)) for s in lines]
        tw = max(r.get_width() for r in rs)
        th = sum(r.get_height() for r in rs)
        bx, by = win_w - tw - 16, win_h - th - 16
        tb = pygame.Rect(bx - 10, by - 10, tw + 20, th + 20)
        pygame.draw.rect(screen, NAVY, tb, border_radius=6)
        y = by
        for r in rs:
            screen.blit(r, (bx, y))
            y += r.get_height()

    def _draw_wheel_box(self, screen, center, uWorld):
        u_world_float = np.array(uWorld, float)
        normal = np.linalg.norm(u_world_float)
        if normal < 1e-9:
            return
        u_world_float /= normal
        n = np.array([-u_world_float[1], u_world_float[0]])
        hl, hw = WHEEL_L / 2, WHEEL_W / 2

        pts = [
            center + u_world_float * hl + n * hw,
            center + u_world_float * hl - n * hw,
            center - u_world_float * hl - n * hw,
            center - u_world_float * hl + n * hw,
        ]
        pygame.draw.polygon(screen, BLACK, [(p[0], win_h - p[1]) for p in pts])


def main():
    global win_w, win_h, font
    pygame.init()
    screen = pygame.display.set_mode((INIT_W, INIT_H), pygame.RESIZABLE)
    pygame.display.set_caption("6-Wheel Kinematic Rover Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 36, bold=True)
    win_w, win_h = screen.get_size()

    sim = Sim()

    # Program runner for command-console programs (distance/time-based moves)
    runner = ProgramRunner(max_speed_mps=sim.max_speed)  # use px/s as our "m/s"


    # modes
    sandbox_mode = True
    network_mode = False
    one_to_one_mode = False

    driving_mode = False  # sandbox-only
    selected_row = 0
    selected_wheel = None

    # sliders (add 1:1 "virtual motor speed" above the others)
    def slider_motor_y(): return win_h - 230
    def slider_speed_drag_y(): return win_h - 170
    def slider_steer_drag_y(): return win_h - 110
    def slider_max_speed_y(): return win_h - 50

    slider_motor_speed = Slider(18, slider_motor_y(), 420,
                                "Virtual Motor Speed (0-1)", init=0.8)
    slider_speed_drag = Slider(18, slider_speed_drag_y(), 420,
                               "Speed Drag (0-3 s⁻¹)", sim.speed_drag / 3.0)
    slider_steer_drag = Slider(18, slider_steer_drag_y(), 420,
                               "Steer Return (0-180 deg/s)", sim.steer_drag / 180.0)

    def max_speed_from_val(v):
        return 50.0 + (v * v) * (600.0 - 50.0)

    def val_from_max_speed(ms):
        return math.sqrt((clamp(ms, 50, 600) - 50.0) / (600.0 - 50.0))

    slider_speed_max = Slider(18, slider_max_speed_y(), 420,
                              "Max Speed (50-600 px/s)",
                              val_from_max_speed(sim.max_speed))

    # buttons (add "1:1 mode")
    btnSandbox, btnNetwork, btnOneToOne = (
        Button("Sandbox mode"), Button("Network mode"), Button("1:1 mode")
    )

    def layout_buttons():
        btnOneToOne.layout(win_w - 12, 12)
        btnNetwork.layout(btnOneToOne.rect.left - 12, 12)
        btnSandbox.layout(btnNetwork.rect.left - 12, 12)

    layout_buttons()

    # UDP + virtual motor controller
    udp = None
    virt = MotorController(0.0, "f", D1, D2, D3, D4)

    # mapping for incoming dir strings (accept both old & new)
    DIR_MAP = {
        # compact codes (motor_code4)
        "f": "f", "b": "b", "pl": "pl", "pr": "pr",
        "fl": "fl", "fr": "fr", "bl": "bl", "br": "br",
        "fpl": "fpl", "fpr": "fpr", "bpl": "bpl", "bpr": "bpr",

        # NEW: compact arrows from the keyboard driver
        "l": "pl",
        "r": "pr",

        # legacy names (keep if you still use the old UI anywhere)
        "forward": "f", "backward": "b", "left": "pl", "right": "pr",
        "fwdl": "fl", "fwdr": "fr", "bwdl": "bl", "bwdr": "br",
    }


    running = True
    while running:
        dt = clock.tick(75) / 1000.0

        # HUD text (prepared early, updated as mode changes)
        if sandbox_mode:
            if driving_mode:
                sim._hud_text = ("SANDBOX: DRIVING — W/S: speed   A/D: counter-steer   "
                                 "M: edit   Space: zero   R: reset   Esc: quit")
            else:
                sel = (f"wheel {selected_wheel + 1}"
                       if selected_wheel is not None
                       else f"row: {ROW_NAMES[selected_row]}")
                sim._hud_text = ("SANDBOX: EDIT — " + sel +
                                 "   W/S: speed   A/D: steer   [ / ]: row   "
                                 "1-6: pick wheel   Shift=ALL   M: drive")
        elif network_mode:
            sim._hud_text = "NETWORK — simple net mapping (W/S/A/D). Esc: quit"
        # Optional program status line in 1:1 mode
        elif one_to_one_mode:
            st = runner.status()
            if st.get("active"):
                rem = st.get("remaining_s", 0.0)
                sim._hud_text += f"   |   Program: {st.get('move')}/{st.get('mode')}  dir={st.get('dir')}  v={st.get('speed'):0.2f}  T-={rem:0.1f}s"


        # events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.VIDEORESIZE:
                win_w, win_h = event.w, event.h
                screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
                slider_motor_speed.rect.y = slider_motor_y()
                slider_speed_drag.rect.y = slider_speed_drag_y()
                slider_steer_drag.rect.y = slider_steer_drag_y()
                slider_speed_max.rect.y = slider_max_speed_y()
                layout_buttons()
                if not (0 <= sim.x <= win_w and 0 <= sim.y <= win_h):
                    sim.center()

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if btnSandbox.hit(event.pos):
                    sandbox_mode, network_mode, one_to_one_mode = True, False, False
                    driving_mode = False
                    selected_row, selected_wheel = 0, None
                    sim.reset()
                    udp = None
                elif btnNetwork.hit(event.pos):
                    sandbox_mode, network_mode, one_to_one_mode = False, True, False
                    driving_mode = True
                    selected_row, selected_wheel = 0, None
                    sim.reset()
                    udp = UdpEndpoint()
                elif btnOneToOne.hit(event.pos):
                    sandbox_mode, network_mode, one_to_one_mode = False, False, True
                    driving_mode = True   # this flag only affects highlighting
                    selected_row, selected_wheel = 0, None
                    sim.reset()
                    udp = UdpEndpoint()

            # sliders always active
            slider_motor_speed.handle(event)
            slider_speed_drag.handle(event)
            slider_steer_drag.handle(event)
            slider_speed_max.handle(event)

            # keyboard (sandbox only)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sim.wheel_speeds[:] = 0.0
                elif event.key == pygame.K_r:
                    sim.reset()
                elif sandbox_mode:
                    if event.key == pygame.K_m:
                        driving_mode = not driving_mode
                        if driving_mode:
                            sim.wheel_angles[list(MID_PAIR)] = 0.0
                            selected_wheel = None
                    elif not driving_mode:
                        if event.key == pygame.K_LEFTBRACKET:
                            selected_row = (selected_row - 1) % 3
                            selected_wheel = None
                        elif event.key == pygame.K_RIGHTBRACKET:
                            selected_row = (selected_row + 1) % 3
                            selected_wheel = None
                        elif pygame.K_1 <= event.key <= pygame.K_6:
                            selected_wheel = event.key - pygame.K_1

        # apply slider params
        sim.speed_drag = 3.0 * slider_speed_drag.value
        sim.steer_drag = 180.0 * slider_steer_drag.value
        sim.max_speed = max_speed_from_val(slider_speed_max.value)
        runner.max_speed_mps = sim.max_speed


        # controls
        keys = pygame.key.get_pressed()
        shiftAll = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]

        if sandbox_mode:
            if driving_mode:
                if keys[pygame.K_w]:
                    sim.nudge_speed_all(+speed_step)
                if keys[pygame.K_s]:
                    sim.nudge_speed_all(-speed_step)
                if keys[pygame.K_a]:
                    sim.counter_steer(+1)
                if keys[pygame.K_d]:
                    sim.counter_steer(-1)
            else:
                targets = (
                    list(range(6))
                    if shiftAll
                    else ([selected_wheel] if selected_wheel is not None
                          else list(ROW_PAIRS[selected_row]))
                )
                if keys[pygame.K_a]:
                    sim.edit_steer(targets, +1)
                if keys[pygame.K_d]:
                    sim.edit_steer(targets, -1)
                ds = (speed_step if keys[pygame.K_w] else 0.0) + \
                     (-speed_step if keys[pygame.K_s] else 0.0)
                if ds:
                    sim.edit_speed(targets, ds)

        elif network_mode:
            # legacy simple net mapping
            if udp:
                for obj in udp.recv_all():
                    typ = obj.get("type")
                    if typ == "ping":
                        udp.send_pong(obj.get("seq", 0), obj.get("ts", time.time()))
                    elif typ == "cmd":
                        d = obj.get("dir")
                        if d == "forward":
                            sim.nudge_speed_all(+speed_step)
                        elif d == "backward":
                            sim.nudge_speed_all(-speed_step)
                        elif d == "left":
                            sim.counter_steer(+1)
                        elif d == "right":
                            sim.counter_steer(-1)
                        elif d == "stop":
                            sim.wheel_speeds[:] = 0.0
                        elif d == "reset":
                            sim.reset()

        else:
            # 1:1 mode — UI_CLI sends dir codes; we run motor_code4 and apply outputs
            # 1:1 mode — UI_CLI sends either low-level 'cmd' or high-level 'program'
            applied_from_cmd = False
            if udp:
                for obj in udp.recv_all():
                    typ = obj.get("type")
                    if typ == "ping":
                        udp.send_pong(obj.get("seq", 0), obj.get("ts", time.time()))
                    elif typ == "cmd":
                        # immediate low-level direction (compact code) — same as before
                        d_in = obj.get("dir")
                        code = DIR_MAP.get(d_in)
                        if code is None:
                            continue
                        motor_speed = float(slider_motor_speed.value)  # 0..1
                        speeds_norm, angles_rad = virt.compute_for_sim(motor_speed, code)
                        sim.apply_virtual_outputs(speeds_norm, angles_rad)
                        applied_from_cmd = True
                    elif typ == "program":
                        # high-level move program from Command Console
                        ok, err = runner.ingest(obj.get("program", {}))
                        if not ok:
                            print("Program error:", err)

                # If a program is running, it takes priority over any 'cmd' this frame
                active, dir_code, speed = runner.sample()
                if active and dir_code:
                    speeds_norm, angles_rad = virt.compute_for_sim(speed, dir_code)
                    sim.apply_virtual_outputs(speeds_norm, angles_rad)
                    applied_from_cmd = True  # mark as applied this frame

            sim._net_connected = udp.is_connected() if udp else False


        # simulate physics + draw
        sim.step(dt)
        connected = (udp.is_connected() if udp else False)
        sim.draw(
            screen,
            driving_mode if sandbox_mode else True,
            connected,
            selected_row,
            selected_wheel if sandbox_mode else None,
        )

        # buttons + sliders
        btnSandbox.draw(screen, active=sandbox_mode)
        btnNetwork.draw(screen, active=network_mode)
        btnOneToOne.draw(screen, active=one_to_one_mode)

        slider_motor_speed.draw(screen, f"{slider_motor_speed.value:0.2f}")
        slider_speed_drag.draw(screen, f"{sim.speed_drag:0.2f} s⁻¹")
        slider_steer_drag.draw(screen, f"{sim.steer_drag:0.0f} deg/s")
        slider_speed_max.draw(screen, f"{sim.max_speed:0.0f} px/s")

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
