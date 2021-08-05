import time
import RPi.GPIO as GPIO


# === INITIALISING LIBRARIES ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


# === INITIALISING PINS (BCM Numbering System) ===
RED_LED_PIN = 26       # Board Pin 37
RED_RGBLED_PIN = 16    # Board Pin 36
GREEN_RGBLED_PIN = 20  # Board Pin 38
BLUE_RGBLED_PIN = 21   # Board Pin 40
STEPPER_STEP_PIN = 6   # Board Pin 31
STEPPER_DIR_PIN = 5    # Board Pin 29
STEPPER_EN_PIN = 19
DRILL_PWM_PIN = 18     # Board Pin 33 (PWM1)
DRILL_DIR_PIN = 23     # Board Pin 35 (PWM)
SERVO_PIN = 12         # Board Pin 32 (PWM0)
WATER_RELAY_PIN = 17   # Board Pin 11


# === INITIALISING MODULES ===

current_waypoint = 1
drive_speed = 100    # 10m/s
walk_speed = 10      # 1m/s

key_pressed = []
CW = 1                      # Clockwise Rotation
CCW = 0                     # Counterclockwise Rotation
STEP_PER_REVOLUTION = 3200  # 200 step/revolution - full-step drive
DUTY_CYCLE = 100            # Full duty cycle by default

# GPIOs
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(RED_RGBLED_PIN, GPIO.OUT)
GPIO.setup(GREEN_RGBLED_PIN, GPIO.OUT)
GPIO.setup(BLUE_RGBLED_PIN, GPIO.OUT)
GPIO.setup(STEPPER_STEP_PIN, GPIO.OUT)
GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT)
GPIO.setup(STEPPER_EN_PIN, GPIO.OUT)
GPIO.setup(DRILL_PWM_PIN, GPIO.OUT)
GPIO.setup(DRILL_DIR_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(WATER_RELAY_PIN, GPIO.OUT)

GPIO.output(STEPPER_EN_PIN, False)
dc_motor1 = GPIO.PWM(DRILL_PWM_PIN, 100)  # Previously 100, Try 50 or 1000
dc_motor1.start(0)
servo1 = GPIO.PWM(SERVO_PIN, 100)
servo1.start(0)


# === [OPTIONAL] SET UP ===
# SERVO starting position?
# Drill starting position?
