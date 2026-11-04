import pygame
import math
from motor_code4 import MotorController

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((1200, 800))
pygame.display.set_caption("Mars Rover Simulation")

# Motor controller
mc = MotorController(motor_speed=0.8, direction='f', D1=1.9, D2=0.495, D3=0.65, D4=0.5)

# Colours
WHITE = (180, 75, 50)
BLACK = (0, 0, 0)
ROVER_COLOUR = (50, 50, 50)
WHEEL_COLOUR = (0, 0, 0)
LINE_COLOUR = (255, 255, 0)
LINE2_COLOUR = (0, 255, 0)
OBSTACLE1 = (50, 0, 0)

x = 0
y = 0
z = 1
k = 1
n = 0
m = 0
speed = 3
speeds = [0.0] * 6
angles = [0.0] * 6
angle_ft = [0] * 6
modifier = [math.sqrt(mc.D2**2 + mc.D3**2), mc.D2, math.sqrt(mc.D2**2 + mc.D4**2), math.sqrt(mc.D2**2 + mc.D3**2), mc.D2, math.sqrt(mc.D2**2 + mc.D4**2)]
modifier2 = [math.sqrt((mc.D1 + mc.D2)**2 + mc.D3**2), mc.D1 + mc.D2, math.sqrt((mc.D2 + mc.D1)**2 + mc.D4**2), math.sqrt((mc.D1 - mc.D2)**2 + mc.D3**2), mc.D1 - mc.D2, math.sqrt((mc.D2 - mc.D1)**2 + mc.D4**2)]
modifier3 = [math.sqrt((mc.D1 - mc.D2)**2 + mc.D3**2), mc.D1 - mc.D2, math.sqrt((mc.D2 - mc.D1)**2 + mc.D4**2), math.sqrt((mc.D1 + mc.D2)**2 + mc.D3**2), mc.D1 + mc.D2, math.sqrt((mc.D2 + mc.D1)**2 + mc.D4**2)]

# Timing
clock = pygame.time.Clock()

# Rover chassis Configuration
rover_x = 540
rover_y = 357.5
rover_pos1 = [540, 357.5]
rover_size = (120, 85)
rover_facts = [rover_x, rover_y, 120, 85]  # x, y position in pixels + size

wheel_offsets_x = [55, -10, -60, 55, -10, -60]
wheel_offsets_y = [-57, -57, -57, 42, 42, 42]

wheel_facts = []
wheel_positions = []
wheel_angles = []
wheel_sizes = []

for i in range(6):
    x = 600 + wheel_offsets_x[i]
    y = 400 + wheel_offsets_y[i]
    
    wheel_facts.append([x, y, 10, 14])       # For drawing
    wheel_positions.append([x, y])         # Raw position
    wheel_angles.append(math.pi / 2)       # Default angle
    wheel_sizes.append((10, 14))            # Size

def draw_rover_chassis():
    '''pygame.draw.rect(screen, ROVER_COLOUR, rover_facts)'''

    #new draw chassis
    '''
    rover_angle = angles[1] - angle_ft[1]
    width, height = rover_size
    rover_surface = pygame.Surface((width, height), pygame.SRCALPHA)
    rover_surface.fill(ROVER_COLOUR)
    rotated_rover = pygame.transform.rotate(rover_surface, math.degrees(rover_angle + n * math.pi/4 + m * math.pi/2))
    rover_pos = rover_pos1
    rotated_rect = rotated_rover.get_rect(center=(rover_pos[0] + width / 2, rover_pos[1] + height / 2))
    screen.blit(rotated_rover, rotated_rect)
    '''

    #newest
    rover_angle = angles[1] - angle_ft[1]
    total_angle = rover_angle + n * math.pi / 4 + m * math.pi / 2

    width, height = rover_size
    rover_surface = pygame.Surface((width, height), pygame.SRCALPHA)
    rover_surface.fill(ROVER_COLOUR)

    rotated_rover = pygame.transform.rotate(rover_surface, math.degrees(total_angle))
    rotated_rect = rotated_rover.get_rect()
    wheel1_center = (wheel_positions[1][0] + 5, wheel_positions[1][1] + 7)
    wheel4_center = (wheel_positions[4][0] + 5, wheel_positions[4][1] + 7)
    pivot_x = (wheel1_center[0] + wheel4_center[0]) / 2
    pivot_y = (wheel1_center[1] + wheel4_center[1]) / 2
    pivot_point = (pivot_x, pivot_y)
    rotated_rect.center = pivot_point

    screen.blit(rotated_rover, rotated_rect)


    
def draw_wheels():
    for i, wheel in enumerate(wheel_facts):
        #wheels
        wheel_angle = angles[i] - angle_ft[i]
        width, height = wheel_sizes[i]         
        #make wheel surface
        wheel_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        wheel_surface.fill(WHEEL_COLOUR)
        #rotate wheel/surface
        rotated_wheel = pygame.transform.rotate(wheel_surface, math.degrees(wheel_angle))
        #apply rect onto wheel position
        wheel_pos = (wheel[0], wheel[1])
        rotated_rect = rotated_wheel.get_rect(center=(wheel_pos[0] + width / 2, wheel_pos[1] + height / 2))
        #draw
        screen.blit(rotated_wheel, rotated_rect)

        # Direction line
        line_len = 15
        line_surface = pygame.Surface((line_len, 2), pygame.SRCALPHA)
        pygame.draw.line(line_surface, LINE_COLOUR, (0, 1), (line_len, 1), 1)
        rotated_line = pygame.transform.rotate(line_surface, math.degrees(wheel_angle))
        line_rect = rotated_line.get_rect(center=(wheel_pos[0] + 5 + line_len/2, wheel_pos[1] + 7))
        screen.blit(rotated_line, line_rect)

        # tangent lines
        tangent_len = 1800
        tangent_surface = pygame.Surface((tangent_len, 2), pygame.SRCALPHA)
        pygame.draw.line(tangent_surface, LINE2_COLOUR, (0, 1), (tangent_len, 1), 1)
        #left tangent line
        rotated_tangent = pygame.transform.rotate(tangent_surface, math.degrees(wheel_angle + math.pi/2))
        tangent_rect = rotated_tangent.get_rect(center=(wheel_pos[0] + 5, wheel_pos[1] + 7))
        screen.blit(rotated_tangent, tangent_rect)
        #right tangent line
        rotated_tangent2 = pygame.transform.rotate(tangent_surface, math.degrees(wheel_angle - math.pi/2))
        tangent_rect2 = rotated_tangent2.get_rect(center=(wheel_pos[0] + 5, wheel_pos[1] + 7))
        screen.blit(rotated_tangent2, tangent_rect2)

def obstacle1():
    pygame.draw.circle(screen, OBSTACLE1, (213, 153), 31)
    pygame.draw.circle(screen, OBSTACLE1, (612, 676), 39)
    pygame.draw.circle(screen, OBSTACLE1, (932, 345), 37)

def main():
    #setup
    direction = 'stop'
    running = True

    while running:
        dt = clock.tick(30) / 1000.0  # Seconds per frame
        screen.fill(WHITE)
        #obstacles
        obstacle1()

        global speed, rover_x, rover_y, speeds, angles

        x = 0
        y = 0
        z = 1
        global n
        global m

        # Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w] and (keys[pygame.K_d]):
            direction = 'fr'
            x = 1/math.sqrt(2)
            y = 1/math.sqrt(2)
            n = 1
            m = 0
        elif keys[pygame.K_w] and (keys[pygame.K_a]):
            direction = 'fl'
            x = 1/math.sqrt(2)
            y = -1/math.sqrt(2)
            n = -1
            m = 0
        elif keys[pygame.K_s] and (keys[pygame.K_d]):
            direction = 'br'
            x = -1/math.sqrt(2)
            y = 1/math.sqrt(2)
            n = -1
            m = 0
        elif keys[pygame.K_s] and (keys[pygame.K_a]):
            direction = 'bl'
            x = -1/math.sqrt(2)
            y = -1/math.sqrt(2)
            n = 1
            m = 0

        elif keys[pygame.K_w] and (keys[pygame.K_e]):
            direction = 'fpr'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = 1
            k = 1
            n = 0
            m = 0
        elif keys[pygame.K_w] and (keys[pygame.K_q]):
            direction = 'fpl'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = -1
            k = 1
            n = 0
            m = 0
        elif keys[pygame.K_s] and (keys[pygame.K_e]):
            direction = 'bpr'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = 1
            k = -1
            n = 0
            m = 0
        elif keys[pygame.K_s] and (keys[pygame.K_q]):
            direction = 'bpl'
            x, y = -1 / math.sqrt(2), -1 / math.sqrt(2)
            z = -1
            k = -1
            n = 0
            m = 0

        elif keys[pygame.K_w]:
            direction = 'f'
            x = 1
            y = 0
            n = 0
            m = 0
        elif keys[pygame.K_s]:
            direction = 'b'
            x = -1
            y = 0
            n = 0
            m = 0
        elif keys[pygame.K_a]:
            direction = 'l'
            x = 0
            y = -1
            n = 0
            m = 1
        elif keys[pygame.K_d]:
            direction = 'r'
            x = 0
            y = 1
            n = 0
            m = 1
        elif keys[pygame.K_e]:
            direction = 'pr'
            x = 0
            y = 0
            z = 1
            n = 0
            m = 0
        elif keys[pygame.K_q]:
            direction = 'pl'
            x = 0
            y = 0
            z = -1
            n = 0
            m = 0
        else:
            direction = 'stop'


        # Get motor speeds and angles
        if direction == 'stop':
            speeds = [0.0] * 6
        else:
            s1, s2, s3, s4, s5, s6, st1, st2, st3, st4, st5, st6 = mc.calc_motor_speed(
                mc.motor_speed, direction, mc.D1, mc.D2, mc.D3, mc.D4
            )

            speeds = [s1, s2, s3, s4, s5, s6] #all good
            angles = [st1, st2, st3, st4, st5, st6] #radians with positive being anticlockwise

        #make wheels move (from speed and direction (+ replace some physics with angle_ft))
        #also rover chassis movement added below wheel movements
        if direction == 'fpr' or direction == 'bpr':
            for i in range(6):
                #done?
                angle_ft[i] += k * z * math.pi*(math.fabs(speeds[i]))/(52*modifier2[i])
                x = speeds[i] * math.cos(-angles[i] + angle_ft[i])
                y = speeds[i] * math.sin(-angles[i] + angle_ft[i])
                wheel_positions[i][0] += x * speed
                wheel_positions[i][1] += y * speed
                wheel_facts[i][0] = wheel_positions[i][0]
                wheel_facts[i][1] = wheel_positions[i][1]
            #chassis
            rover_x += x * speed
            rover_y += y * speed
            rover_pos1[0] = rover_x
            rover_pos1[1] = rover_y

        elif direction == 'fpl' or direction == 'bpl':
            for i in range(6):
                #done?
                angle_ft[i] += k * z * math.pi*(math.fabs(speeds[i]))/(52*modifier3[i])
                x = speeds[i] * math.cos(-angles[i] + angle_ft[i])
                y = speeds[i] * math.sin(-angles[i] + angle_ft[i])
                wheel_positions[i][0] += x * speed
                wheel_positions[i][1] += y * speed
                wheel_facts[i][0] = wheel_positions[i][0]
                wheel_facts[i][1] = wheel_positions[i][1]
        if direction == 'pr' or direction == 'pl':
            for i in range(6):
                #use d2 or d2 extension to turn. add constant angle
                angle_ft[i] += z * math.pi*(math.fabs(speeds[i]))/(104*modifier[i])
                x = speeds[i] * math.cos(-angles[i] + angle_ft[i])
                y = speeds[i] * math.sin(-angles[i] + angle_ft[i])
                wheel_positions[i][0] += x * speed
                wheel_positions[i][1] += y * speed
                wheel_facts[i][0] = wheel_positions[i][0]
                wheel_facts[i][1] = wheel_positions[i][1]
            #chassis
            rover_x += x * speed
            rover_y += y * speed
            rover_pos1[0] = rover_x
            rover_pos1[1] = rover_y

        else:
            for i in range(6):
                x = speeds[i] * math.cos(-angles[i] + angle_ft[i])
                y = speeds[i] * math.sin(-angles[i] + angle_ft[i])
                wheel_positions[i][0] += x * speed
                wheel_positions[i][1] += y * speed
                wheel_facts[i][0] = wheel_positions[i][0]
                wheel_facts[i][1] = wheel_positions[i][1]
            #chassis
            rover_x += x * speed
            rover_y += y * speed
            rover_pos1[0] = rover_x
            rover_pos1[1] = rover_y

        #draw them
        draw_rover_chassis()
        draw_wheels()

        #display
        pygame.display.flip()

    #quit
    pygame.quit()

if __name__ == "__main__":
    main()
