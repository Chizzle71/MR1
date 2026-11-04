# pygame simulator environment

# imports
import json, math, os, socket, threading, time
import pygame

# network/pygame variables/config
LISTEN_PORT = 9999
WINDOW_SIZE = (1300, 900)
SPRITE_PATH = "assets/rover-sprite.png"
FWD_SPEED = 160.0
# vary based on turn point etc.
TURN_RATE = math.radians(180)
DEADMAN_S = 0.5                 # if no commands within this window -> disconnected/neutral
BG_COLOR = (140, 140, 220)
HUD_TEXT_COLOR = (255, 255, 255)
HUD_BG_COLOR = (0, 30, 80)
FONT_SIZE = 32


class UdpServer:
    """Receives command + ping; replies to ping with pong; stores last dir & ping_ms."""
    def __init__(self, port: int):
        self.port = port
        self._dir = None
        self._ping_ms = None
        self._last_rx = 0.0
        self._running = False
        self._th = None

    # rework to be more intuitive
    @property
    def dir(self): return self._dir
    @property
    def ping_ms(self): return self._ping_ms
    @property
    def last_rx_time(self): return self._last_rx

    # net connection loop for packet seeking/processing/connection and pinging
    def _loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.port))
        sock.settimeout(0.5)
        print(f"Simulator listening on UDP :{self.port}")
        while self._running:
            try:
                data, addr = sock.recvfrom(2048)
                obj = json.loads(data.decode("utf-8"))
                typ = obj.get("type")
                if typ == "cmd":
                    self._dir = obj.get("dir")
                    self._ping_ms = obj.get("ping_ms")
                    self._last_rx = time.time()
                elif typ == "ping":
                    # immediate pong (send seq and ts)
                    pong = {"type": "pong", "seq": obj.get("seq"), "ts": obj.get("ts")}
                    sock.sendto(json.dumps(pong).encode("utf-8"), addr)
            except socket.timeout:
                pass
            except Exception:
                time.sleep(0.01)

    def start(self):
        # run net process in bg
        if self._running: return
        self._running = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def stop(self):
        self._running = False

def load_sprite(path):
    surf = pygame.image.load(path).convert_alpha()
    return surf

def draw_hud(screen, font, text, topleft=(8,8), pad=6):
    text_surf = font.render(text, True, HUD_TEXT_COLOR)
    rect = text_surf.get_rect(topleft=topleft)
    bg = pygame.Rect(rect.left - pad, rect.top - pad, rect.width + pad*2, rect.height + pad*2)
    pygame.draw.rect(screen, HUD_BG_COLOR, bg, border_radius=6)
    screen.blit(text_surf, rect)

def main():
    rx = UdpServer(LISTEN_PORT); rx.start()

    # init pygame, clock + set font + sprite details
    pygame.init()
    pygame.display.set_caption("Rover Simulator")
    screen = pygame.display.set_mode(WINDOW_SIZE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", FONT_SIZE)

    #sprite_path = os.path.join(os.getcwd(), SPRITE_PATH)
    rover = load_sprite(SPRITE_PATH)

    # set scene scale based on window size
    x, y = WINDOW_SIZE[0]/2, WINDOW_SIZE[1]/2
    yaw = 0.0

    try:
        running = True
        while running:
            # set frame delay, event types
            dt = clock.tick(75) / 1000.0
            for e in pygame.event.get():
                if e.type == pygame.QUIT: running = False
                if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE: running = False

            # connection state + ping (age)
            age = time.time() - rx.last_rx_time
            connected = age <= DEADMAN_S
            d = rx.dir if connected else None

            # motion of sprite using trig
            if d == "forward":
                x +=  math.sin(yaw) * FWD_SPEED * dt
                y -=  math.cos(yaw) * FWD_SPEED * dt
            elif d == "backward":
                x -=  math.sin(yaw) * FWD_SPEED * dt
                y +=  math.cos(yaw) * FWD_SPEED * dt
            elif d == "left":
                yaw -= TURN_RATE * dt
            elif d == "right":
                yaw += TURN_RATE * dt

            # scale window
            w, h = WINDOW_SIZE
            x = (x + w) % w; y = (y + h) % h

            # draw scene and render
            screen.fill(BG_COLOR)
            angle = -math.degrees(yaw)
            rover_rot = pygame.transform.rotozoom(rover, angle, 1.0)
            rect = rover_rot.get_rect(center=(x, y))
            screen.blit(rover_rot, rect)

            if connected and (rx.ping_ms is not None):
                hud = f"connected - ping = {rx.ping_ms:0.1f} ms   dir: {d or 'neutral'}"
            else:
                hud = "disconnected"
            draw_hud(screen, font, hud)

            pygame.display.flip()
    finally:
        rx.stop()
        pygame.quit()

if __name__ == "__main__":
    main()
