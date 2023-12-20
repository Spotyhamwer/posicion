# posicion
Posicion motor
import RPi.GPIO as GPIO
import time

# GPIO pins for motor control
motor_pwm_pin = 18
motor_direction_pin = 23
encoder_pin_A = 17
encoder_pin_B = 27

# Motor parameters
motor_frequency = 1000  # PWM frequency in Hz
encoder_resolution = 360  # Encoder resolution in pulses per revolution

# Motor position parameters
target_position = 180  # Target position in degrees

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pwm_pin, GPIO.OUT)
GPIO.setup(motor_direction_pin, GPIO.OUT)
GPIO.setup(encoder_pin_A, GPIO.IN)
GPIO.setup(encoder_pin_B, GPIO.IN)

# Setup PWM
motor_pwm = GPIO.PWM(motor_pwm_pin, motor_frequency)
motor_pwm.start(0)

# Motor control function
def set_motor_speed(speed, direction):
    GPIO.output(motor_direction_pin, direction)
    motor_pwm.ChangeDutyCycle(speed)

# Encoder reading function
def read_encoder():
    encoder_A = GPIO.input(encoder_pin_A)
    encoder_B = GPIO.input(encoder_pin_B)
    return (encoder_A, encoder_B)

# Calculate current motor position
def get_motor_position(pulses):
    return (pulses % encoder_resolution) * (360 / encoder_resolution)

try:
    # Main control loop
    while True:
        # Read encoder
        encoder_A, encoder_B = read_encoder()

        # Calculate motor position in degrees
        motor_position = get_motor_position(encoder_A + (encoder_B << 1))

        # PID control (adjust according to your requirements)
        error = target_position - motor_position
        pid_output = error * 0.1  # Placeholder PID coefficient, adjust as needed

        # Motor control
        set_motor_speed(pid_output, 1)  # Assume clockwise rotation

        time.sleep(0.1)  # Control loop delay

except KeyboardInterrupt:
    pass
finally:
    # Cleanup GPIO
    motor_pwm.stop()
    GPIO.cleanup()
