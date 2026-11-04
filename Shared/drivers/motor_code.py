# Imports
import time
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
PWM_NEUTRAL = 7.5  # Neutral duty cycle (stop) for PWM
PWM_RANGE = 5  # Range is [5, 10] for PWM signal

# Assign pins for the servos
servo_pins = [2, 3, 4, 5, 6, 7]  # Placeholder values for GPIO pins

# Setup the GPIO pins
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setwarnings(False)  # Ignore previous pin usage

# Initialize PWM for each servo
pwms = []
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)  # Set pin as output
    pwm = GPIO.PWM(pin, FREQ)  # Set frequency
    pwm.start(PWM_NEUTRAL)  # Start at 0 speed, range [5, 10]
    pwms.append(pwm)


class MotorController:
    """MotorController class containing 4 motor functions"""
    
    def __init__(self, motor_speed: float, direction: str) -> None:
        """
        Initialize the MotorController

        Args:
            motor_speed (float): speed of the motors in range [-1, 1]
            direction (str): direction of movement
        """
        self.motor_speed = motor_speed
        self.direction = direction
    
    def calc_motor_speed(self, motor_speed: float, direction: str) -> tuple:
        """
        Calculate individual motor speeds from speed and direction

        Args:
            motor_speed (float): Speed of the motors
            direction (str): Direction keyword

        Returns:
            tuple: Six throttle values for the motors
        """
        if direction == "fwd":  # Forward
            s1 = s2 = s3 = s4 = s5 = s6 = motor_speed
        elif direction == "bwd":  # Backward
            s1 = s2 = s3 = s4 = s5 = s6 = -motor_speed
        elif direction == "fwdl":  # Forward and left
            s1 = s2 = s3 = motor_speed / 2
            s4 = s5 = s6 = motor_speed
        elif direction == "fwdr":  # Forward and right
            s1 = s2 = s3 = motor_speed
            s4 = s5 = s6 = motor_speed / 2
        elif direction == "bwdl":  # Backward and left
            s1 = s2 = s3 = -motor_speed / 2
            s4 = s5 = s6 = -motor_speed
        elif direction == "bwdr":  # Backward and right
            s1 = s2 = s3 = -motor_speed
            s4 = s5 = s6 = -motor_speed / 2
        elif direction == "l":  # Pivot left
            s1 = s2 = s3 = -motor_speed / 2
            s4 = s5 = s6 = motor_speed / 2
        elif direction == "r":  # Pivot right
            s1 = s2 = s3 = motor_speed / 2
            s4 = s5 = s6 = -motor_speed / 2
        else:  # Default case if there is no input (stop)
            s1 = s2 = s3 = s4 = s5 = s6 = 0
        return s1, s2, s3, s4, s5, s6
    
    def throttle_to_duty(self, throttle: float) -> float:
        """
        Convert throttle value to PWM duty cycle

        Args:
            throttle (float): throttle in range [-1, 1]

        Returns:
            float: PWM duty cycle corresponding to the throttle
        """
        return PWM_NEUTRAL + (throttle * PWM_RANGE/2)  # Range [5, 10]

    def set_speed(
            self, s1: float, s2: float, s3: float,
            s4: float, s5: float, s6: float
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
            pwm.stop()  # Stop PWM signal
        GPIO.cleanup()  # Clean up GPIO pin settings
