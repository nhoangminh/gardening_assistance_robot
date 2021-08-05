import concurrent.futures
from initializer import *
from plant_module import Drill, Water, Seed
from dronekit_guided import Controller


class Plant:
    @staticmethod
    def start():
        for _ in range(3):
            GPIO.output(RED_LED_PIN, True)  # Blink 3 times
            time.sleep(0.5)
            GPIO.output(RED_LED_PIN, False)
            time.sleep(0.5)
        GPIO.output(RED_LED_PIN, True)      # Leave RED LED on as long as RPi is powered

    @staticmethod
    def cleanup():
        # RESET all functions
        GPIO.cleanup()

    def plant(self):
        # Create Objects
        drill_object = Drill(STEPPER_STEP_PIN, STEPPER_DIR_PIN,
                             dc_motor1, DRILL_DIR_PIN, DUTY_CYCLE, drill_speed_change=1,
                             rgb_pin=RED_RGBLED_PIN, latency=2)

        seed_object = Seed(servo1, GREEN_RGBLED_PIN, latency=7.5)
        
        water_object = Water(WATER_RELAY_PIN, BLUE_RGBLED_PIN, latency=2)

        print("Drill Deploying...")
        drill_object.rgb()
        drill_object.stepper(CCW, 3.5, 0.0001)
        
        with concurrent.futures.ThreadPoolExecutor() as executor:
            executor.submit(drill_object.stepper, CCW, 1, 0.0005)
            drill_object.drill(CW)

        GPIO.output(DRILL_DIR_PIN, False)
        dc_motor1.ChangeDutyCycle(0)
        
        drill_object.stepper(CW, 4.5, 0.0001)
        drill_object.rgb()
        
        Controller.send_ned_velocity(walk_speed, 0, 0, 6)

        time.sleep(1)
        print("Seed Deploying...")
        seed_object.rgb()
        seed_object.seed()
        seed_object.rgb()
        
        time.sleep(1)
        print("Water Deploying...")
        water_object.rgb()
        water_object.water()
        water_object.rgb()
