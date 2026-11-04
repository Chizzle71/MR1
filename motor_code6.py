# Imports
import time
import math

# try to import Servo -> will fail if not on RPI -> set Test mode if not on Pi
try:
    from servo import Servo  # type: ignore
    TEST_MODE = False
except Exception:
    TEST_MODE = True
    # set up mock Servo for compatibility with main code
    class Servo:
        def __init__(self, pin):
            self.pin = pin
            self.angle = 90.0
        def write(self, angle):
            self.angle = angle
        def stop(self):
            pass

# Variables
motor_speed = 0.8  # Range [-1, 1] where 0 is stop
PWM_NEUTRAL = 90.0  # Neutral angle (stop/forward) for continuous servo
PWM_RANGE = 90.0    # Range from 0–180 degrees equivalent

# Geometry constants (m) (placeholders for now)
D1 = 0.5  # Radius of turn (point of turn to centre between middle two wheels)
D2 = 0.2  # Width between left wheel and centre (centre of wheel)
D3 = 0.1  # Length from front to middle wheel (centre to centre of wheels)
D4 = 0.08 # Length from back to middle wheel (centre to centre of wheels)

# Assign pins for the servos
servo_pins = [2, 3, 4, 5, 6, 7]       # Placeholder values for continuous servos
servo_pins2 = [8, 9, 10, 11, 12, 13]  # Placeholder values for steering servos

# Initialize Servo objects for each motor
servos = []
for pin in servo_pins:
    servo = Servo(pin)
    servo.write(PWM_NEUTRAL)  # Start at neutral (stop)
    servos.append(servo)

servos2 = []
for pin in servo_pins2:
    servo = Servo(pin)
    servo.write(PWM_NEUTRAL)  # Start at 90 degrees (straight)
    servos2.append(servo)


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
            s1 = s2 = s3 = s4 = s5 = s6 = 0
        return s1, s2, s3, s4, s5, s6, st1, st2, st3, st4, st5, st6

    def angle_to_servo(self, angle: float) -> float:
        """
        Convert steering angle (-90 to 90) to range (0 to 180)
        """
        return (angle + 90)

    def throttle_to_servo(self, throttle: float) -> float:
        """
        Convert throttle [-1, 1] to range (0 tot 180)
        """
        return (90 + throttle * 90)

    def set_angles(self, st1, st2, st3, st4, st5, st6) -> None:
        """
        Set steering angles for the servos
        """
        angles = [st1, st2, st3, -st4, -st5, -st6]
        if TEST_MODE:
            print("[TEST motors] angles:", [f"{a:+0.2f}" for a in angles])
            return
        for servo, a in zip(servos2, angles):
            servo.write(self.angle_to_servo(a))

    def set_speed(self, s1, s2, s3, s4, s5, s6) -> None:
        """
        Set speed of continuous rotation servos
        """
        throttles = [s1, s2, s3, -s4, -s5, -s6]
        if TEST_MODE:
            print("[TEST motors] speeds:", [f"{t:+0.2f}" for t in throttles])
            return
        for servo, t in zip(servos, throttles):
            servo.write(self.throttle_to_servo(t))

    def cleanup(self) -> None:
        """Cleanup function to stop all motors"""
        if TEST_MODE:
            print("[TEST motors] cleanup: stop and release servos")
            return
        for servo in servos + servos2:
            servo.write(PWM_NEUTRAL)
            time.sleep(0.1)
            servo.stop()
