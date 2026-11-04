
"""
Kinetic rover simulator

Uses a rigid body, with 6 wheels, solves least squares to compute

Each wheel i has steering angle θ_i and scalar wheel speed v_i along its axis. 

Let u_i (axis), n_i (lateral), r_i (wheel position in body frame). 
2 Constraints - rolling along wheel axis = commanded wheel speed, and no lateral slip:

ξ = [v_x_b, v_y_b, ω] satisfies, per wheel:

    u_i^T (v_b + ωxr_i) = v_i          (drive along axis)
    n_i^T (v_b + ωxr_i) = 0            (no lateral slip)

Stack for 6 wheels → A ξ = b (12x3). Linear system
Solve least squares. Convert to world coordinate system

integrate pose, compute ICR:

    Icr = worldPos + (-v_y_world, v_x_world) / ω
"""
# imports
import json
import math
import socket
import time

import numpy as np
import pygame

# pygame render/bg constants
bg = (210, 220, 235)
navy = (0, 30, 80)
white = (255, 255, 255)
black = (0, 0, 0)
blue = (80, 180, 255)
indexBlue = (70, 150, 255)
orange = (255, 120, 60)
red = (200, 20, 20)
slBg = (210, 210, 215)
slFg = (50, 120, 220)
slKnob = (20, 20, 25)

initW, initH = 1200, 800
wheelL, wheelW = 78.0, 36.0
arrowLen, headLen = 90.0, 120.0

# geometry constants
bodyL, bodyW = 360.0, 220.0
xFront, xMid, xRear = +bodyL / 2, 0.0, -bodyL / 2
yLeft, yRight = +bodyW / 2, -bodyW / 2
wpos = np.array(
    [
        [xFront, yLeft],   # 1 -> 0
        [xMid, yLeft],     # 2 -> 1
        [xRear, yLeft],    # 3 -> 2
        [xFront, yRight],  # 4 -> 3
        [xMid, yRight],    # 5 -> 4
        [xRear, yRight],   # 6 -> 5
    ],
    float,
)
rowPairs = ((0, 3), (1, 4), (2, 5))
rowNames = ("front (1&4)", "mid (2&5)", "rear (3&6)")
frontPair, midPair, rearPair = (0, 3), (1, 4), (2, 5)

# physics and control constants
speedStep = 20.0
angleStep = math.radians(5.0)
angleMin, angleMax = -math.pi / 2, math.pi / 2
omegaEps = 1e-3
steerIdleS = 0.6

# network consts
udpHost, udpPort = "0.0.0.0", 9999
netTimeoutS = 1.0

# pygame init
winW, winH, font = initW, initH, None


# clamp and wrap utilities
def clamp(x, lo, hi):
    """Clamp x to [lo, hi] for inputs"""
    return lo if x < lo else hi if x > hi else x


def wrap_pi(a):
    """Wrap angle to (-π, π) for clamp"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


# UI Parts
class Slider:
    """Horizontal slider with a label and info text"""

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
        pygame.draw.rect(screen, slBg, self.rect, border_radius=7)
        fill = self.rect.copy()
        fill.w = int(self.rect.w * self.value)
        pygame.draw.rect(screen, slFg, fill, border_radius=7)
        kx = self.rect.x + int(self.rect.w * self.value)
        knob = pygame.Rect(kx - 9, self.rect.centery - 11, 18, 22)
        pygame.draw.rect(screen, slKnob, knob, border_radius=6)
        screen.blit(
            font.render(valueText, True, (30, 30, 30)),
            (self.rect.right + 12, self.rect.y - 2),
        )


class Button:
    """Pill buttton object that resizes automatically, toggle"""

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


# UDP Server to connect over net
class UdpEndpoint:
    """UDP server that speaks ui_cli.py JSON protocol"""

    def __init__(self):
        # UDP setup, set socket IP etc.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((udpHost, udpPort))
        self.sock.setblocking(False)
        self.lastHeard = 0.0
        self.lastFrom = None  

    def recv_all(self):
        msgs = []
        while True:
            # receive all text messages from UDP endpoint
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
        # finish ping pong sequence to get latency
        if not self.lastFrom:
            return
        pkt = {"type": "pong", "seq": seq, "ts": ts}
        try:
            self.sock.sendto(json.dumps(pkt).encode(), self.lastFrom)
        except Exception:
            pass

    def is_connected(self):
        # check if still connected
        return (time.time() - self.lastHeard) < netTimeoutS


# rover movement + swerve simulation main
class Sim:
    """Simple 6-module swerve vehicle."""

    def __init__(self):
        # pos of box
        self.x = winW / 2
        self.y = winH / 2
        self.yaw = 0.0  
        # yaw=0 points UP on screen

        # per-wheel states for angle and speed
        self.angles = np.zeros(6)  # θ_i (radians)
        self.speeds = np.zeros(6)  # along wheel axis

        # can be changed with slider
        self.speedDrag = 1.2       # s^-1; exponential decay of wheel speeds
        self.returnDegPerS = 60.0  # deg/s; steering returns to zero after idle
        self.maxSpeed = 240.0      # px/s cap per wheel

        self.lastSteerTs = time.time()
        self.icrWorld = None  # (x,y) of instantaneous center of rotation

    # coordinate transforms on objects
    def rot_body_to_world(self):
        """2x2 rotation matrix body->world. yaw=0 means UP is +Y (screen up)."""
        psi = self.yaw + math.pi / 2
        c, s = math.cos(psi), math.sin(psi)
        return np.array([[c, -s], [s, c]], float)

    # solve overall velociy using lest squares integration
    def body_twist(self):
        """
        Solve A ξ = b for ξ = [v_x_b, v_y_b, ω] (body frame) in least squares
        Two equations per wheel: drive along axis, and no lateral slip
        """
        rows = []
        rhs = []
        for i in range(6):
            # for each wheel
            th = self.angles[i]
            u = np.array([math.cos(th), math.sin(th)])        # axis
            n = np.array([-math.sin(th), math.cos(th)])       # lateral
            r = wpos[i]
            zxr = np.array([-r[1], r[0]])                     # ω × r in 2D
            rows += [[u[0], u[1], float(u @ zxr)],
                     [n[0], n[1], float(n @ zxr)]]
            rhs += [self.speeds[i], 0.0]
        A = np.asarray(rows)
        b = np.asarray(rhs)
        xi, *_ = np.linalg.lstsq(A, b, rcond=None)
        return xi  # vx_b, vy_b, omega

    # integration funcs + misc.
    def step(self, dt):
        # speed drag: exponential decay per second -> like friction to slow rover down
        if self.speedDrag > 0:
            self.speeds *= max(0.0, 1.0 - self.speedDrag * dt)
            self.speeds[np.abs(self.speeds) < 0.02] = 0.0

        # steering self-centering after idle delay to rest pose
        if time.time() - self.lastSteerTs >= steerIdleS:
            rate = math.radians(clamp(self.returnDegPerS, 0, 180))
            if rate > 0:
                dmax = rate * dt
                a = self.angles
                self.angles = np.clip(a + np.clip(-a, -dmax, dmax), angleMin, angleMax)
                self.angles[np.abs(self.angles) < math.radians(0.25)] = 0.0

        # compute body twist, convert to world, integrate pose
        vxB, vyB, omega = self.body_twist()
        R = self.rot_body_to_world()
        c, s = R[0, 0], R[1, 0]  # first column (cos,sin)
        vxW, vyW = c * vxB - s * vyB, s * vxB + c * vyB

        # ICR (turn center) 
        if abs(omega) > omegaEps:
            self.icrWorld = (self.x + (-vyW) / omega, self.y + (vxW) / omega)
        else:
            self.icrWorld = None

        self.x += vxW * dt
        self.y += vyW * dt
        self.yaw = wrap_pi(self.yaw + omega * dt)

        # keep inside - if out, recenter
        if not (0 <= self.x <= winW and 0 <= self.y <= winH):
            self.center()

    # control commands for different modes and changing constants
    def nudge_speed_all(self, delta):
        """Increase/decrease all wheel speeds, capped by maxSpeed."""
        self.speeds[:] = np.clip(self.speeds + delta, -self.maxSpeed, self.maxSpeed)
    # driving mode
    def counter_steer(self, sign):
        """Front vs rear steer in opposite directions, mids straight."""
        self.lastSteerTs = time.time()
        for i in frontPair:
            self.angles[i] = clamp(self.angles[i] + sign * angleStep, angleMin, angleMax)
        for i in rearPair:
            self.angles[i] = clamp(self.angles[i] - sign * angleStep, angleMin, angleMax)
        self.angles[list(midPair)] = 0.0
    # change steer angle with arrow keys
    def edit_steer(self, idxs, sign):
        """Increment steering on specific wheel indices."""
        self.lastSteerTs = time.time()
        for i in idxs:
            self.angles[i] = clamp(self.angles[i] + sign * angleStep, angleMin, angleMax)
    # accel
    def edit_speed(self, idxs, delta):
        """Increment speeds on specific wheel indices (capped)."""
        for i in idxs:
            self.speeds[i] = clamp(self.speeds[i] + delta, -self.maxSpeed, self.maxSpeed)

    # misc.
    def center(self):
        self.x, self.y = winW / 2, winH / 2

    def reset(self):
        self.__init__()

    # DRAWING code for geometry
    def draw(self, screen, drivingMode, netConnected, selectedRow, selectedWheel):
        screen.fill(bg)
        R = self.rot_body_to_world()

        # body polygon (white fill + black outline) + center dot -> main rover body
        body = np.array(
            [
                [-bodyL / 2, -bodyW / 2],
                [bodyL / 2, -bodyW / 2],
                [bodyL / 2, bodyW / 2],
                [-bodyL / 2, bodyW / 2],
            ]
        )
        world = (body @ R.T) + np.array([self.x, self.y])
        poly = [(p[0], winH - p[1]) for p in world]
        pygame.draw.polygon(screen, white, poly, 0)
        pygame.draw.polygon(screen, black, poly, 3)
        pygame.draw.circle(screen, indexBlue, (int(self.x), int(winH - self.y)), 8)

        # heading arrow -> point forward or UP from vehicle aligned to world
        psi = self.yaw + math.pi / 2
        nose = np.array([self.x, self.y]) + np.array([math.cos(psi), math.sin(psi)]) * headLen
        pygame.draw.line(screen, blue, (self.x, winH - self.y), (nose[0], winH - nose[1]), 4)

        # ICR dot position + dashed radius to show turn size
        if self.icrWorld:
            ix, iy = self.icrWorld
            if -winW * 5 < ix < winW * 6 and -winH * 5 < iy < winH * 6:
                icr = (int(ix), int(winH - iy))
                cen = (int(self.x), int(winH - self.y))
                pygame.draw.circle(screen, red, icr, 6)
                dash, gap = 20, 12
                dx, dy = icr[0] - cen[0], icr[1] - cen[1]
                dist = math.hypot(dx, dy)
                if dist > 1:
                    ux, uy = dx / dist, dy / dist
                    for i in range(int(dist // (dash + gap)) + 1):
                        s = (
                            int(cen[0] + (dash + gap) * i * ux),
                            int(cen[1] + (dash + gap) * i * uy),
                        )
                        e = (int(s[0] + dash * ux), int(s[1] + dash * uy))
                        pygame.draw.line(screen, red, s, e, 3)

        # which wheels to highlight
        if drivingMode:
            hi = set(frontPair + rearPair)
        elif selectedWheel is not None:
            hi = {selectedWheel}
        else:
            hi = set(rowPairs[selectedRow])

        # draw wheels (black rectangles), axle dots, direction arrows, number labels
        centers = []
        for i in range(6):
            r = wpos[i]
            p = (r @ R.T) + np.array([self.x, self.y])
            th = self.angles[i]
            uBody = np.array([math.cos(th), math.sin(th)])
            uWorld = uBody @ R.T
            self._draw_wheel_box(screen, p, uWorld)
            p1 = (p[0], winH - p[1])
            p2v = p + uWorld * arrowLen
            p2 = (p2v[0], winH - p2v[1])
            pygame.draw.circle(screen, orange if i in hi else (60, 120, 220), p1, 8)
            pygame.draw.line(screen, blue, p1, p2, 4)
            centers.append((i, p1))

        for i, p1 in centers:
            screen.blit(font.render(str(i + 1), True, indexBlue), (p1[0] + 18, p1[1] - 18))

        # top HUD - text on controls UI etc.
        pad = 10
        if drivingMode:
            hud = "DRIVING — W/S: speed  A/D: counter-steer  M: edit  Space: zero  R: reset  Esc: quit"
        else:
            sel = f"wheel {selectedWheel + 1}" if selectedWheel is not None else f"row: {rowNames[selectedRow]}"
            hud = f"EDIT — {sel}   W/S: speed  A/D: steer   [ / ]: row   1-6: pick wheel   Shift=ALL   M: drive"
        t = font.render(hud, True, (255, 255, 255))
        hudBg = pygame.Rect(pad, pad, t.get_width() + 2 * pad, t.get_height() + 2 * pad)
        pygame.draw.rect(screen, navy, hudBg, border_radius=6)
        screen.blit(t, (hudBg.x + pad, hudBg.y + pad))

        # connection badge
        t2 = font.render(f"Network: {'Connected' if netConnected else 'Disconnected'}", True, (255, 255, 255))
        b2 = pygame.Rect(pad, hudBg.bottom + 8, t2.get_width() + 2 * pad, t2.get_height() + 2 * pad)
        pygame.draw.rect(screen, navy, b2, border_radius=6)
        screen.blit(t2, (b2.x + pad, b2.y + pad))

        # bottom-right telemetry (angles & speeds per wheel)
        lines = [
            f"{i + 1}: angle={math.degrees(self.angles[i]):5.1f}°   speed={self.speeds[i]:+6.1f} px/s"
            for i in range(6)
        ]
        rs = [font.render(s, True, (255, 255, 255)) for s in lines]
        tw = max(r.get_width() for r in rs)
        th = sum(r.get_height() for r in rs)
        bx, by = winW - tw - 16, winH - th - 16
        tb = pygame.Rect(bx - 10, by - 10, tw + 20, th + 20)
        pygame.draw.rect(screen, navy, tb, border_radius=6)
        y = by
        for r in rs:
            screen.blit(r, (bx, y))
            y += r.get_height()

    def _draw_wheel_box(self, screen, center, uWorld):
        """Draw a rectangular black rectangle -> wheel, aligned with world axis"""
        # record pos
        uw = np.array(uWorld, float)
        nrm = np.linalg.norm(uw)
        if nrm < 1e-9:
            return
        uw /= nrm
        n = np.array([-uw[1], uw[0]])
        hl, hw = wheelL / 2, wheelW / 2
        pts = [
            center + uw * hl + n * hw,
            center + uw * hl - n * hw,
            center - uw * hl - n * hw,
            center - uw * hl + n * hw,
        ]
        pygame.draw.polygon(screen, black, [(p[0], winH - p[1]) for p in pts])


# main func, handles pygame
def main():
    global winW, winH, font
    pygame.init()
    screen = pygame.display.set_mode((initW, initH), pygame.RESIZABLE)
    pygame.display.set_caption("6-Module Swerve Simulator — Sandbox / Network")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 36, bold=True)
    winW, winH = screen.get_size()

    sim = Sim()

    sandboxMode = True
    drivingMode = False
    selectedRow = 0          
    selectedWheel = None     

    # sliders (always visible)
    def y1():
        return winH - 170

    def y2():
        return winH - 110

    def y3():
        return winH - 50

    sDrag = Slider(18, y1(), 420, "Speed Drag (0-3 s⁻¹)", sim.speedDrag / 3.0)
    sRet = Slider(18, y2(), 420, "Steer Return (0-180 deg/s)", sim.returnDegPerS / 180.0)

    def max_speed_from_val(v):
        # quadratic easing instead of linear
        return 50.0 + (v * v) * (600.0 - 50.0)

    def val_from_max_speed(ms):
        # clamps speed values to range
        return math.sqrt((clamp(ms, 50, 600) - 50.0) / (600.0 - 50.0))

    sMax = Slider(18, y3(), 420, "Max Speed (50-600 px/s)", val_from_max_speed(sim.maxSpeed))

    # buttons setup
    btnSandbox, btnNetwork = Button("Sandbox mode"), Button("Network mode")

    def layout_buttons():
        btnNetwork.layout(winW - 12, 12)
        btnSandbox.layout(btnNetwork.rect.left - 12, 12)

    layout_buttons()

    udp = None  # UdpEndpoint or None
    running = True
    while running:
        # tick
        dt = clock.tick(75) / 1000.0

        # pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # rescale adaptively
            elif event.type == pygame.VIDEORESIZE:
                winW, winH = event.w, event.h
                screen = pygame.display.set_mode((winW, winH), pygame.RESIZABLE)
                sDrag.rect.y, sRet.rect.y, sMax.rect.y = y1(), y2(), y3()
                layout_buttons()
                if not (0 <= sim.x <= winW and 0 <= sim.y <= winH):
                    sim.center()

            # change mode or wheels
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if btnSandbox.hit(event.pos):
                    sandboxMode = True
                    drivingMode = False
                    selectedRow, selectedWheel = 0, None
                    sim.reset()
                    udp = None
                elif btnNetwork.hit(event.pos):
                    sandboxMode = False
                    drivingMode = True
                    selectedRow, selectedWheel = 0, None
                    sim.reset()
                    udp = UdpEndpoint()

            # sliders
            sDrag.handle(event)
            sRet.handle(event)
            sMax.handle(event)

            # keyboard mode toggles / selectors
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sim.speeds[:] = 0.0
                elif event.key == pygame.K_r:
                    sim.reset()
                elif sandboxMode:
                    # default mode. change to driving with M
                    if event.key == pygame.K_m:
                        drivingMode = not drivingMode
                        if drivingMode:
                            sim.angles[list(midPair)] = 0.0
                            selectedWheel = None
                    elif not drivingMode:
                        # use [] to change wheel pair
                        if event.key == pygame.K_LEFTBRACKET:
                            selectedRow = (selectedRow - 1) % 3
                            selectedWheel = None
                        elif event.key == pygame.K_RIGHTBRACKET:
                            selectedRow = (selectedRow + 1) % 3
                            selectedWheel = None
                        elif pygame.K_1 <= event.key <= pygame.K_6:
                            selectedWheel = event.key - pygame.K_1

        # apply slider params 
        sim.speedDrag = 3.0 * sDrag.value
        sim.returnDegPerS = 180.0 * sRet.value
        sim.maxSpeed = max_speed_from_val(sMax.value)

        # controls (keyboard in sandbox; network in network mode)
        keys = pygame.key.get_pressed()
        shiftAll = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]

        if sandboxMode:
            # set driving mode, front + rear wheel control
            if drivingMode:
                if keys[pygame.K_w]:
                    sim.nudge_speed_all(+speedStep)
                if keys[pygame.K_s]:
                    sim.nudge_speed_all(-speedStep)
                if keys[pygame.K_a]:
                    sim.counter_steer(+1)
                if keys[pygame.K_d]:
                    sim.counter_steer(-1)
            else:
                # select wheel pairs
                targets = (
                    list(range(6))
                    if shiftAll
                    else ([selectedWheel] if selectedWheel is not None else list(rowPairs[selectedRow]))
                )
                if keys[pygame.K_a]:
                    sim.edit_steer(targets, +1)
                if keys[pygame.K_d]:
                    sim.edit_steer(targets, -1)
                ds = (speedStep if keys[pygame.K_w] else 0.0) + (-speedStep if keys[pygame.K_s] else 0.0)
                if ds:
                    sim.edit_speed(targets, ds)
        else:
            # parse network packets and determine direction command
            if udp:
                for obj in udp.recv_all():
                    typ = obj.get("type")
                    if typ == "ping":
                        udp.send_pong(obj.get("seq", 0), obj.get("ts", time.time()))
                    elif typ == "cmd":
                        d = obj.get("dir")
                        if d == "forward":
                            sim.nudge_speed_all(+speedStep)
                        elif d == "backward":
                            sim.nudge_speed_all(-speedStep)
                        elif d == "left":
                            sim.counter_steer(+1)
                        elif d == "right":
                            sim.counter_steer(-1)
                        elif d == "stop":
                            sim.speeds[:] = 0.0
                        elif d == "reset":
                            sim.reset()

        # compute physics and draw result
        sim.step(dt)
        connected = udp.is_connected() if udp else False
        sim.draw(
            screen,
            drivingMode if sandboxMode else True,
            connected,
            selectedRow,
            selectedWheel if sandboxMode else None,
        )

        # buttons + sliders - always drawn
        btnSandbox.draw(screen, active=sandboxMode)
        btnNetwork.draw(screen, active=not sandboxMode)
        sDrag.draw(screen, f"{sim.speedDrag:0.2f} s⁻¹")
        sRet.draw(screen, f"{sim.returnDegPerS:0.0f} deg/s")
        sMax.draw(screen, f"{sim.maxSpeed:0.0f} px/s")

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
