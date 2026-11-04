# Imports
import time
from motor_code7 import MotorController as MC7

controller = MotorController(motor_speed=0.6, direction="f", D1=0.25, D2=0.2, D3=0.1, D4=0.08)

speeds, angles = controller.calc_motor_speed(0.6, "f", 0.25, 0.2, 0.1, 0.08)
