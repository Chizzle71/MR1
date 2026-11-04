import pygame
import math
from motor_code4 import MotorController

#init
pygame.init()
screen = pygame.display.set_mode((1200, 800))
pygame.display.set_caption("Mars Rover Simulation")

#save as
clock = pygame.time.Clock()
mc = MotorController(motor_speed=0.8, direction='f', D1=1.9, D2=0.495, D3=0.65, D4=0.5)

#colour
WHITE = (200, 150, 150)
ROVER_COLOUR = (0, 0, 255)
WHEEL_COLOUR = (0, 0, 0)
LINE_COLOUR = (255, 255, 0)
LINE2_COLOUR = (0, 255, 0)

#global variable
speed = 3
speeds = [0.0] * 6
angles = [0.0] * 6
angle_ft = [0] * 6
modifier = [
    math.sqrt(mc.D2**2 + mc.D3**2), mc.D2, math.sqrt(mc.D2**2 + mc.D4**2),
    math.sqrt(mc.D2**2 + mc.D3**2), mc.D2, math.sqrt(mc.D2**2 + mc.D4**2)
]

#rover variables?
rover_size = [120, 85]
rover_x = 540
rover_y = 357.5
rover_angle = 0.0  # in radians

wheel_offsets_x = [55, -10, -60, 55, -10, -60]
wheel_offsets_y = [-57, -57, -57, 42, 42, 42]

wheel_facts = []
wheel_positions = []
wheel_angles = []
wheel_sizes = []

#make wheels exist
for i in range(6):
    x = 600 + wheel_offsets_x[i]
    y = 400 + wheel_offsets_y[i]
    wheel_facts.append([x, y, 10, 14])
    wheel_positions.append([x, y])
    wheel_angles.append(math.pi / 2)
    wheel_sizes.append((10, 14))


def draw_rover_chassis():
    width, height = rover_size
    rover_surface = pygame.Surface((width, height), pygame.SRCALPHA)
    rover_surface.fill(ROVER_COLOUR)

    rotated_rover = pygame.transform.rotate(rover_surface, -math.degrees(rover_angle))
    rect = rotated_rover.get_rect(center=(rover_x + width / 2, rover_y + height / 2))
    screen.blit(rotated_rover, rect)


def draw_wheels():
    for i, wheel in enumerate(wheel_facts):
        wheel_angle = angles[i] - angle_ft[i]
        width, height = wheel_sizes[i]

        wheel_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        wheel_surface.fill(WHEEL_COLOUR)
        rotated_wheel = pygame.transform.rotate(wheel_surface, math.degrees(wheel_angle))
        wheel_pos = (wheel[0], wheel[1])
        rotated_rect = rotated_wheel.get_rect(center=(wheel_pos[0] + width / 2, wheel_pos[1] + height / 2))
        screen.blit(rotated_wheel, rotated_rect)

        #direction line
        cx = wheel_pos[0] + width / 2
        cy = wheel_pos[1] + height / 2
        line_len = 15
        dx = line_len * math.cos(wheel_angle)
        dy = -line_len * math.sin(wheel_angle)
        pygame.draw.line(screen, LINE_COLOUR, (cx, cy), (cx + dx, cy + dy), 1)

        #tangent line
        tangent_len = 900
        dx_t = tangent_len * math.cos(-wheel_angle - math.pi / 2)
        dy_t = tangent_len * math.sin(-wheel_angle - math.pi / 2)
        pygame.draw.line(screen, LINE2_COLOUR, (cx - dx_t, cy - dy_t), (cx + dx_t, cy + dy_t), 1)


def main():
    global rover_x, rover_y, speeds, angles, rover_angle

    direction = 'stop'
    running = True

    while running:
        dt = clock.tick(30) / 1000.0
        screen.fill(WHITE)

        x = y = 0
        z = 1  #rotation direction

        #event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w] and keys[pygame.K_d]:
            direction = 'fr'
            x, y = 1 / math.sqrt(2), 1 / math.sqrt(2)
        elif keys[pygame.K_w] and keys[pygame.K_a]:
            direction = 'fl'
            x, y = 1 / math.sqrt(2), -1 / math.sqrt(2)
        elif keys[pygame.K_s] and keys[pygame.K_d]:
            direction = 'br'
            x, y = -1 / math.sqrt(2), 1 / math.sqrt(2)
        elif keys[pygame.K_s] and keys[pygame.K_a]:
            direction = 'bl'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)

        elif keys[pygame.K_w] and keys[pygame.K_e]:
            direction = 'fpr'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = 1
        elif keys[pygame.K_s] and keys[pygame.K_a]:
            direction = 'fpl'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = -1
        elif keys[pygame.K_s] and keys[pygame.K_d]:
            direction = 'bpr'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = 1
        elif keys[pygame.K_s] and keys[pygame.K_a]:
            direction = 'bpl'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = -1

        elif keys[pygame.K_w]:
            direction = 'f'
            x, y = 1, 0
        elif keys[pygame.K_s]:
            direction = 'b'
            x, y = -1, 0
        elif keys[pygame.K_a]:
            direction = 'l'
            x, y = 0, -1
        elif keys[pygame.K_d]:
            direction = 'r'
            x, y = 0, 1

        elif keys[pygame.K_e]:
            direction = 'pr'
            z = 1
        elif keys[pygame.K_q]:
            direction = 'pl'
            z = -1
        else:
            direction = 'stop'

        # use motor_code4.py
        if direction == 'stop':
            speeds = [0.0] * 6
        else:
            s1, s2, s3, s4, s5, s6, st1, st2, st3, st4, st5, st6 = mc.calc_motor_speed(
                mc.motor_speed, direction, mc.D1, mc.D2, mc.D3, mc.D4
            )
            speeds = [s1, s2, s3, s4, s5, s6]
            angles = [st1, st2, st3, st4, st5, st6]

        #move chassis
        width, height = rover_size
        center_x = rover_x + width / 2
        center_y = rover_y + height / 2

        if direction in ['fpr', 'fpl', 'bpl', 'bpr']:  #fixxxxxxxx
            c=0
        elif direction in ['pr', 'pl']:
            rover_angle += math.pi * (s2)/(mc.D2 * 104)
        else:
            dx = x * speed * mc.motor_speed
            dy = y * speed * mc.motor_speed
            move_x = dx * math.cos(rover_angle) - dy * math.sin(rover_angle)
            move_y = dx * math.sin(rover_angle) + dy * math.cos(rover_angle)
            center_x += move_x
            center_y += move_y

        rover_x = center_x - width / 2
        rover_y = center_y - height / 2

        #move wheels
        for i in range(6):
            if direction in ['fpr', 'fpl', 'bpl', 'bpr']:  #fix
                c=0
            elif direction in ['pr', 'pl']:
                angle_ft[i] += z * math.pi * abs(speeds[i]) / (104 * modifier[i])
            else:
                move_x = speeds[i] * math.cos(-angles[i] + angle_ft[i])
                move_y = speeds[i] * math.sin(-angles[i] + angle_ft[i])
                wheel_positions[i][0] += move_x * speed
                wheel_positions[i][1] += move_y * speed
                wheel_facts[i][0] = wheel_positions[i][0]
                wheel_facts[i][1] = wheel_positions[i][1]

        #draw
        draw_rover_chassis()
        draw_wheels()
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
