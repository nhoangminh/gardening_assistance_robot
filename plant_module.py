from initializer import *


class Skeleton:
    def __init__(self, rgb_pin, latency):
        self.rgb_flag = False
        self.rgb_pin = rgb_pin
        self.latency = latency
    
    def rgb(self):
        if not self.rgb_flag:
            self.rgb_flag = True
            GPIO.output(self.rgb_pin, self.rgb_flag)  # Change RGB Light
        else:
            self.rgb_flag = False
            GPIO.output(self.rgb_pin, self.rgb_flag)  # Off RGB Light


class Drill(Skeleton):
    def __init__(self, stepper_step_pin, stepper_dir_pin,
                 drill_motor, drill_dir_pin, duty_cycle, drill_speed_change,
                 rgb_pin, latency):
        super().__init__(rgb_pin, latency)
        self.stepper_step_pin = stepper_step_pin
        self.stepper_dir_pin = stepper_dir_pin
        self.spr = STEP_PER_REVOLUTION
        self.drill_motor = drill_motor
        self.drill_dir_pin = drill_dir_pin
        self.duty_cycle = duty_cycle
        self.drill_speed_change = drill_speed_change

    def __del__(self):
        # STEPPER CLEANUP (eg. Drill returns to default position)
        pass
    
    def stepper(self, rotation, multiplier, delay):
        # STEPPER LOWER/RAISES
        GPIO.output(self.stepper_dir_pin, rotation)
        for _ in range(int(self.spr * multiplier)):
            GPIO.output(self.stepper_step_pin, True)
            time.sleep(delay)
            GPIO.output(self.stepper_step_pin, False)
            time.sleep(delay)
    
    def drill(self, rotation):
        # DRILL SPINS
        GPIO.output(self.drill_dir_pin, rotation)
        self.drill_motor.ChangeDutyCycle(self.duty_cycle)
        time.sleep(self.latency)
    
    def drill_speed(self, change):
        if change == "increase":
            if self.duty_cycle == 100:
                pass
            else:
                self.duty_cycle += self.drill_speed_change
            print(f"Increasing drilling speed, {self.duty_cycle}")
        elif change == "decrease":
            if self.duty_cycle == 0:  # Likely chance that self.duty_cycle<=10 will stall, if it does change this
                pass
            else:
                self.duty_cycle -= self.drill_speed_change
            print(f"Decreasing drilling speed, {self.duty_cycle}")


class Seed(Skeleton):
    def __init__(self, servo, rgb_pin, latency):
        super().__init__(rgb_pin, latency)
        self.servo = servo

    def __del__(self):
        self.servo.stop(0)                  # Kill servo

    def seed(self):
        self.servo.ChangeDutyCycle(14.8)    # Clockwise
        time.sleep(self.latency)
        self.servo.stop(14)                 # STOP


class Water(Skeleton):
    def __init__(self, relay, rgb_pin, latency):
        super().__init__(rgb_pin, latency)
        self.relay = relay

    def water(self):
        GPIO.output(self.relay, True)   # Module on
        time.sleep(self.latency)        # Dispense an amount of water
        GPIO.output(self.relay, False)  # Module off
