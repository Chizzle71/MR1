import pigpio
import math
import time

# Variables
motor_speed = 0.8  # Range [-1, 1] where 0 is stop
FREQ = 50  # Frequency of the MG996R servos
PWM_NEUTRAL = 7.5  # Neutral duty cycle (stop/forward) for PWM
PWM_RANGE = 5  # Range is [5, 10] for PWM signal

# Geometry constants (m) (placeholders for now)
D1 = 0.5  # Radius of turn (point of turn to centre between middle two wheels)
D2 = 0.2  # Width between left wheel and centre (centre of wheel)
D3 = 0.1  # Length from front to middle wheel (centre to centre of wheels)
D4 = 0.08  # Length from back to middle wheel (centre to centre of wheels)

# Assign pins for the servos
servo_pins = [2, 3, 4, 5, 6, 7]  # Placeholder values for continuous servos
servo_pins2 = [8, 9, 10, 11, 12, 13]  # Placeholder values for steering servos
servos = [PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL]
servos2 = [PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL]

# Initialise PWM for each servo
pwm = pigpio.pi() 
pwm.set_mode(servo_pins, pigpio.OUTPUT)
pwm.set_mode(servo_pins2, pigpio.OUTPUT) 
pwm.set_PWM_frequency(servo_pins, 50)
pwm.set_PWM_frequency(servo_pins2, 50)
pwm.set_PWM_range(servo_pins, 20000) # 1,000,000 / 50 = 20,000us for 100% duty cycle
pwm.set_PWM_range(servo_pins2, 20000) # 1,000,000 / 50 = 20,000us for 100% duty cycle


# tdClass
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

        for servo2, a in zip(servos2, angles):
            duty = self.angle_to_duty(a)  # Converts angles to duty cycle
            servo2.ChangeDutyCycle(duty)

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

        for servo, t in zip(servos, throttles):
            duty = self.throttle_to_duty(t)  # Converts throttle to duty cycle
            servo.ChangeDutyCycle(duty)
            
    def cleanup(self) -> None:
        """Cleanup function to stop all motors and then clean up GPIO"""
        for servo in servos:
            servo.ChangeDutyCycle(PWM_NEUTRAL)  # Set servos to 0 speed
            time.sleep(0.1)  # Small delay to ensure stop command is sent
            servo.stop()  # Stop PWM signal
        for servo2 in servos2:
            servo2.ChangeDutyCycle(PWM_NEUTRAL)  # Set servos to neutral position
            time.sleep(0.1)  # Small delay to ensure stop command is sent
            servo2.stop()  # Stop PWM signal






    # also add control through a console
    # and also make a main file