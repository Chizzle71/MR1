# Imports
import time
import math
from adafruit_servokit import ServoKit

# try to import GPIO -> will fail if not on RPI -> set Test mode if not on Pi
try:
    import RPi.GPIO as _REAL_GPIO  # type: ignore
    TEST_MODE = False
    GPIO = _REAL_GPIO
except Exception:
    TEST_MODE = True
    # set up mock PWM/GPIO for compatibility with main code
    class _MockPWM:
        def __init__(self, pin, freq):
            self.pin = pin
            self.freq = freq
            self.duty = 0.0
        def start(self, duty):
            self.duty = duty
        def ChangeDutyCycle(self, duty):
            self.duty = duty
        def stop(self):
            pass

    class _MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        def setmode(self, mode): pass
        def setwarnings(self, flag): pass
        def setup(self, pin, mode): pass
        def PWM(self, pin, freq): return _MockPWM(pin, freq)
        def cleanup(self): pass

    GPIO = _MockGPIO()  # mock instance

# Variables
motor_speed = 0.8  # Range [-1, 1] where 0 is stop
FREQ = 50  # Frequency of the MG996R servos
PWM_NEUTRAL = 7.5  # Neutral duty cycle (stop/forward) for PWM
PWM_RANGE = 5  # Range is [5, 10] for PWM signal

# Geometry constants (m) (placeholders for now)
D1 = 1  # Radius of turn (point of turn to centre between middle two wheels)
D2 = 0.2  # Width between left wheel and centre (centre of wheel)
D3 = 0.1  # Length from front to middle wheel (centre to centre of wheels)
D4 = 0.08  # Length from back to middle wheel (centre to centre of wheels)

# Assign pins for the servos
servo_pins = [0,1,2,3]  # Postions on channel [0-15] for 180 servos
servo_pins2 = [4]  # Postions on channel [0-15] for continuous servos

# Setup the GPIO pins
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setwarnings(False)  # Ignore previous pin usage

pwms = []
pwms2 = []

s = [s1, s2, s3, s4, s5, s6] = [0, 0, 0, 0, 0, 0]
st = [st1, st2, st3, st4, st5, st6] = [0, 0, 0, 0, 0, 0]


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
            # Leave steering servoes at last position
        return s1, s2, s3, s4, s5, s6, st1, st2, st3, st4, st5, st6

    def angle_to_duty(self, angle: float) -> float:
        """
        Convert steering angle to PWM duty cycle
        Args:
            angle (float): steering angle in range [-90, 90]
        Returns:
            float: PWM duty cycle corresponding to the steering angle
        """
        return PWM_NEUTRAL + ((2*angle/math.pi) * PWM_RANGE/2)  # Range [5, 10]

    def throttle_to_duty(self, throttle: float) -> float:
        """
        Convert throttle value to PWM duty cycle
        Args:
            throttle (float): throttle in range [-1, 1]
        Returns:
            float: PWM duty cycle corresponding to the throttle
        """
        return PWM_NEUTRAL + (throttle * PWM_RANGE/2)  # Range [5, 10]

    def set_angles(
            self, st1: float, st2: float, st3: float,
            st4: float, st5: float, st6: float
            ) -> None:
        """
        Set the angles of each motor by setting their PWM duty cycles
        Args:
            st1, st2, st3, st4, st5, st6 (float): steering angles for the motors
        """
        # Reversed sign of last 3 throttles due to orientation of motors
        angles = [st1, st2, st3, -st4, -st5, -st6]

        if TEST_MODE:
            print("[TEST motors] angles (st1..st6): "
                  f"{angles[0]:+0.2f}, {angles[1]:+0.2f}, {angles[2]:+0.2f}, "
                  f"{angles[3]:+0.2f}, {angles[4]:+0.2f}, {angles[5]:+0.2f}")
            return

        for pwm2, a in zip(pwms2, angles):
            duty = self.angle_to_duty(a)  # Converts angles to duty cycle
            pwm2.ChangeDutyCycle(duty)

    def set_speed(
            self, s1: float, s2: float, s3: float,
            s4: float, s5: float, s6: float,
            ) -> None:
        """
        Set the speed of each motor by setting their PWM duty cycles
        Args:
            s1, s2, s3, s4, s5, s6 (float): throttle values for the motors
        """
        # Reversed sign of last 3 throttles due to orientation of motors
        throttles = [s1, s2, s3, -s4, -s5, -s6]

        if TEST_MODE:
            print("[TEST motors] speeds (s1..s6): "
                  f"{throttles[0]:+0.2f}, {throttles[1]:+0.2f}, {throttles[2]:+0.2f}, "
                  f"{throttles[3]:+0.2f}, {throttles[4]:+0.2f}, {throttles[5]:+0.2f}")
            return

        for pwm, t in zip(pwms, throttles):
            duty = self.throttle_to_duty(t)  # Converts throttle to duty cycle
            pwm.ChangeDutyCycle(duty)

    def cleanup(self) -> None:
        """Cleanup function to stop all motors and then clean up GPIO"""
        if TEST_MODE:
            print("[TEST motors] cleanup: stop (neutral) and GPIO cleanup")
            return
        for pwm in pwms:
            pwm.ChangeDutyCycle(PWM_NEUTRAL)  # Set servos to 0 speed
            time.sleep(0.1)  # Small delay to ensure stop command is sent
            pwm.stop()  # Stop PWM signal
        for pwm in pwms2:
            pwm.ChangeDutyCycle(PWM_NEUTRAL)  # Set servos to neutral position
            time.sleep(0.1)  # Small delay to ensure stop command is sent
            pwm.stop()  # Stop PWM signal
        GPIO.cleanup()  # Clean up GPIO pin settings







def loop():
    time.sleep(1)
    direction = "fpr"
    
    controller = MotorController(motor_speed, direction, D1, D2, D3, D4)
    s1, s2, s3, s4, s5, s6, st1, st2, st3, st4, st5, st6 = controller.calc_motor_speed(
        motor_speed, direction, D1, D2, D3, D4
    )

    s = [s1, s2, s3, s4, s5, s6]
    st = [st1, st2, st3, st4, st5, st6]

    print("\nMotor speeds:")
    for i, val in enumerate(s):
        print(f"  s{i+1}: {val:+.3f}")
        kit.servo[i].angle = val # Set throttle for continuous servos

    print("\nSteering angles:")
    for i, val in enumerate(st):
        print(f"  st{i+1}: {-val*180/math.pi:+.3f}")
        kit.continuous_servo[i].throttle = val*180/math.pi  # Set angle for 180 servos



if __name__ == "__main__":
    kit = ServoKit(channels=16)
    loop()