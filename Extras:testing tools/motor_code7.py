# Imports
import time
import math
from adafruit_servokit import ServoKit

# Variables
motor_speed = 0.6  # Range [-1, 1] where 0 is stop --- 0.2 is now stop
FREQ = 500  # Frequency of the MG996R servos
stop_speed = 0.2
range_speed = 1 # ---test this range

# Geometry constants (m) (placeholders for now)
D1 = 0.25  # Radius of turn (point of turn to centre between middle two wheels)
D2 = 0.2  # Width between left wheel and centre (centre of wheel)
D3 = 0.1  # Length from front to middle wheel (centre to centre of wheels)
D4 = 0.08  # Length from back to middle wheel (centre to centre of wheels)

# Assign pins for the servos
servo_pins = [8,9,10,11,12]  # Postions on channel [0-15] for 180 servos
servo_pins2 = [0,1,2,3,4]  # Postions on channel [0-15] for continuous servos

s = [s1, s2, s3, s4, s5, s6] = [0, 0, 0, 0, 0, 0]
st = [st1, st2, st3, st4, st5, st6] = [0, 0, 0, 0, 0, 0]

# Initialise servo board
kit = ServoKit(channels=16)


class MotorController:
    """MotorController class containing n motor functions"""

    def __init__(
            self, motor_speed: float, direction: str,
            D1: float, D2: float, D3: float, D4: float
            ) -> None:
        """
        Initialize the MotorController
        Args:
            motor_speed (float): speed of the motors in range [-1, 1]
            direction (str): direction of movement
        """
        self.motor_speed = motor_speed
        self.direction = direction
        self.D1 = D1
        self.D2 = D2
        self.D3 = D3
        self.D4 = D4

    def calc_motor_speed(
            self, motor_speed: float, direction: str,
            D1: float, D2: float, D3: float, D4: float
            ) -> tuple:
        """
        Calculate individual motor speeds and angles from speed and direction
        Assumes D3 > D4
        Angle written in degrees with range [-90, 90], where 0 is forward
        Args:
            motor_speed (float): Speed of the motors
            direction (str): Direction keyword
        Returns:
            tuple: Six throttle values for the motors
            tuple: Six steering angles for the servos
        """
        if direction == "f":  # Forward
            st1 = st2 = st3 = st4 = st5 = st6 = 0
            s1 = s2 = s3 = s4 = s5 = s6 = motor_speed
        elif direction == "b":  # Backward
            st1 = st2 = st3 = st4 = st5 = st6 = 0
            s1 = s2 = s3 = s4 = s5 = s6 = -motor_speed
        elif direction == "l":  # Crab left
            st1 = st2 = st3 = st4 = st5 = st6 = math.pi/2
            s1 = s2 = s3 = s4 = s5 = s6 = motor_speed
        elif direction == "r":  # Crab right
            st1 = st2 = st3 = st4 = st5 = st6 = math.pi/2
            s1 = s2 = s3 = s4 = s5 = s6 = -motor_speed
        elif direction == "fl":  # Forward and left translation
            st1 = st2 = st3 = st4 = st5 = st6 = math.pi/4
            s1 = s2 = s3 = s4 = s5 = s6 = motor_speed
        elif direction == "fr":  # Forward and right translation
            st1 = st2 = st3 = st4 = st5 = st6 = -math.pi/4
            s1 = s2 = s3 = s4 = s5 = s6 = motor_speed
        elif direction == "bl":  # Backward and left translation
            st1 = st2 = st3 = st4 = st5 = st6 = -math.pi/4
            s1 = s2 = s3 = s4 = s5 = s6 = -motor_speed
        elif direction == "br":  # Backward and right translation
            st1 = st2 = st3 = st4 = st5 = st6 = math.pi/4
            s1 = s2 = s3 = s4 = s5 = s6 = -motor_speed
        elif direction == "pl":  # Pivot left
            st1 = -math.atan((D3)/(D2))
            st4 = -st1
            st2 = st5 = 0
            st3 = math.atan((D4)/(D2))
            st6 = -st3
            s1 = -motor_speed
            s4 = -s1
            s3 = s1 * math.sqrt((D2**2 + D4**2)/(D2**2 + D3**2))
            s6 = -s3
            s2 = s1 * (D2)/(math.sqrt(D2**2 + D3**2))
            s5 = -s2
        elif direction == "pr":  # Pivot right
            st1 = -math.atan((D3)/(D2))
            st4 = -st1
            st2 = st5 = 0
            st3 = math.atan((D4)/(D2))
            st6 = -st3
            s1 = motor_speed
            s4 = -s1
            s3 = s1 * math.sqrt((D2**2 + D4**2)/(D2**2 + D3**2))
            s6 = -s3
            s2 = s1 * (D2)/(math.sqrt(D2**2 + D3**2))
            s5 = -s2
        elif direction == "fpl":  # Forward and turn left
            st1 = math.atan((D3)/(D1-D2))
            st2 = st5 = 0
            st3 = -math.atan((D4)/(D1-D2))
            st4 = math.atan((D3)/(D1+D2))
            st6 = -math.atan((D4)/(D1+D2))
            s4 = motor_speed
            s5 = s4 * math.cos(st4)
            s6 = (s5)/(math.cos(-st6))
            s2 = s5 * (D1-D2)/(D1+D2)
            s1 = (s2)/(math.cos(st1))
            s3 = (s2)/(math.cos(-st3))
        elif direction == "fpr":  # Forward and turn right
            st1 = -math.atan((D3)/(D1+D2))
            st2 = st5 = 0
            st3 = math.atan((D4)/(D1+D2))
            st4 = -math.atan((D3)/(D1-D2))
            st6 = math.atan((D4)/(D1-D2))
            s1 = motor_speed
            s2 = s1 * math.cos(st1)
            s3 = (s2)/(math.cos(st3))
            s5 = s2 * (D1-D2)/(D1+D2)
            s4 = (s5)/(math.cos(st4))
            s6 = (s5)/(math.cos(st6))
        elif direction == "bpl":  # Backward and turn left
            st1 = math.atan((D3)/(D1-D2))
            st2 = st5 = 0
            st3 = -math.atan((D4)/(D1-D2))
            st4 = math.atan((D3)/(D1+D2))
            st6 = -math.atan((D4)/(D1+D2))
            s4 = -motor_speed
            s5 = s4 * math.cos(st4)
            s6 = (s5)/(math.cos(-st6))
            s2 = s5 * (D1-D2)/(D1+D2)
            s1 = (s2)/(math.cos(st1))
            s3 = (s2)/(math.cos(-st3))
        elif direction == "bpr":  # Backward and turn right
            st1 = -math.atan((D3)/(D1+D2))
            st2 = st5 = 0
            st3 = math.atan((D4)/(D1+D2))
            st4 = -math.atan((D3)/(D1-D2))
            st6 = math.atan((D4)/(D1-D2))
            s1 = -motor_speed
            s2 = s1 * math.cos(st1)
            s3 = (s2)/(math.cos(st3))
            s5 = s2 * (D1-D2)/(D1+D2)
            s4 = (s5)/(math.cos(st4))
            s6 = (s5)/(math.cos(st6))
        else:  # Default case if there is no input (stop)
            s1 = s2 = s3 = s4 = s5 = s6 = 0.2
            st1 = st2=st3=st4=st5=st6=9
            # Leave steering servoes at last position
        return s1, s2, s3, s4, s5, s6, st1, st2, st3, st4, st5, st6
    
    def set_angles(self, st: list) -> None:
        """
        Set the angles of the steering servos
        Args:
            st (list): List of steering angles
        """
        for i, val in enumerate(servo_pins):
            kit.servo[servo_pins[i]].angle = (st[i])*180/math.pi+90 # Set angle for 180 servos
    
    def set_throttles(self, s: list) -> None:
        """
        Set the throttles of the continuous servos
        Args:
            s (list): List of throttle values
        """
        for i, val in enumerate(servo_pins2):
            kit.continuous_servo[servo_pins2[i]].throttle = s[i] # Set throttle for continuous servos

    def cleanup(self) -> None:
        """
        Cleanup function to stop all servos
        """
        for i, val in enumerate(servo_pins):
            kit.servo[servo_pins[i]].angle = 90  # Set steering servos to neutral position
        
        for i, val in enumerate(servo_pins2):
            kit.continuous_servo[servo_pins2[i]].throttle = 0  # Stop continuous servos

