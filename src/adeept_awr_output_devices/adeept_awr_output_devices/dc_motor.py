import RPi.GPIO as GPIO

GPIO.cleanup()

left_side_motor_EN = 4
right_side_motor_EN = 17
left_side_motor_Pin1 = 26
left_side_motor_Pin2 = 21
right_side_motor_Pin1 = 27
right_side_motor_Pin2 = 18

class DCMotor():

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(left_side_motor_EN, GPIO.OUT)
        GPIO.setup(left_side_motor_Pin1, GPIO.OUT)
        GPIO.setup(left_side_motor_Pin2, GPIO.OUT)
        GPIO.setup(right_side_motor_EN, GPIO.OUT)
        GPIO.setup(right_side_motor_Pin1, GPIO.OUT)
        GPIO.setup(right_side_motor_Pin2, GPIO.OUT)
        
        self.stopMotors() #set all gpio to low

        global pwm_A, pwm_B
        try:
            pwm_A = GPIO.PWM(left_side_motor_EN, 1000)
            pwm_B = GPIO.PWM(right_side_motor_EN, 1000)
        except:
            pass

    def stopMotors(self):
        GPIO.output(left_side_motor_Pin1, GPIO.LOW)
        GPIO.output(left_side_motor_Pin2, GPIO.LOW)
        GPIO.output(right_side_motor_Pin1, GPIO.LOW)
        GPIO.output(right_side_motor_Pin2, GPIO.LOW)
        GPIO.output(left_side_motor_EN, GPIO.LOW)
        GPIO.output(right_side_motor_EN, GPIO.LOW)


    def left_side_motor(self, direction, speed):
        if direction == -1:
            GPIO.output(left_side_motor_Pin1, GPIO.HIGH)
            GPIO.output(left_side_motor_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        
        elif direction == 1:
            GPIO.output(left_side_motor_Pin1, GPIO.LOW)
            GPIO.output(left_side_motor_Pin2, GPIO.HIGH)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)


    def right_side_motor(self, direction, speed):
        if direction == 1:
            GPIO.output(right_side_motor_Pin1, GPIO.HIGH)
            GPIO.output(right_side_motor_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        
        elif direction == -1:
            GPIO.output(right_side_motor_Pin1, GPIO.LOW)
            GPIO.output(right_side_motor_Pin2, GPIO.HIGH)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)