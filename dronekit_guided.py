# ======== Pseudocode ================
# Read waypoint file, obtain waypoint coordinates
# Set GUIDED mode, move to waypoint 1 (eg. Vehicle.simple_goto())
# Checks: Distance to target, Reached target
# Perform action (eg. Drilling)
# Move to next waypoint, repeat action
# If last waypoint, exit code

import time
import math
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
from planter import Plant


class Controller:
    def __init__(self, current_waypoint, drive_speed, walk_speed, return_to_launch):
        self.current_waypoint = current_waypoint
        self.drive_speed = drive_speed
        self.walk_speed = walk_speed
        self.return_to_launch = return_to_launch
        
        self.vehicle = None
        self.n_WP = None
        self.waypoint_iter = None
        self.a_location = None

    def __del__(self):
        self.vehicle.close()

    def connect_my_vehicle(self, connection_string):
        print("Connecting...")
        self.vehicle = connect(connection_string, wait_ready=True)
        print(f"Connecting to rover on: {connection_string}")

    def change_mode(self, mode):
        while self.vehicle.mode != VehicleMode(mode):
            self.vehicle.mode = VehicleMode(mode)
            print(f"Waiting to enter {mode} mode")
            time.sleep(1)

    def arm_and_guided(self):
        while not self.vehicle.is_armable:
            print("Waiting for rover to become armable")  # Pass Checks
            time.sleep(1)
        
        self.change_mode("GUIDED")

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(f"Waiting for {self.vehicle} to be disarmed")
            time.sleep(1)
        print(f"{self.vehicle} is now disarmed")

    def hold_and_disarm(self):
        self.change_mode("HOLD")

        self.vehicle.armed = False
        while self.vehicle.armed:
            print(f"Waiting for {self.vehicle} to be disarmed")
            time.sleep(1)
        print(f"{self.vehicle} is now disarmed")

    def planting(self):
        planter = Plant()

        try:
            planter.start()
            print("Start Planting...")
            planter.plant()
            print("Finish Planting...")

        except KeyboardInterrupt:
            print("Stop Planting: KeyboardInterrupt")

        planter.cleanup()
        time.sleep(1)

    def obtain_waypoints(self, aFileName):
        print(f"\nReading mission from file: {aFileName}")
        waypoint_list = []
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i == 0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray = line.split('\t')
                    ln_param5 = float(linearray[8])   # LAT
                    ln_param6 = float(linearray[9])   # LONG
                    ln_param7 = float(linearray[10])  # ALT
                    waypoint_list.append((ln_param5, ln_param6, ln_param7))
        waypoint_list.pop(0)  # METHOD 2: RTL
        self.n_WP = len(waypoint_list)
        self.waypoint_iter = iter(waypoint_list)
        return self.n_WP, waypoint_list, self.waypoint_iter
    
    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def go_there(self):
        print(f"\n> Going to Waypoint {self.current_waypoint}")
        if self.current_waypoint < self.n_WP:  # Not Last Waypoint
            self.current_waypoint += 1
        else:
            self.return_to_launch = True  # FLAG

        element = next(self.waypoint_iter)
        self.a_location = LocationGlobalRelative(element[0], element[1], element[2])
        # print(a_location)  # For Debugging
        self.vehicle.simple_goto(self.a_location, groundspeed=self.drive_speed)

    def check_if_reached(self):
        while True:
            currentLocation = self.vehicle.location.global_relative_frame
            targetDistance = self.get_distance_metres(currentLocation, self.a_location)
            # print(f"Distance from next waypoint: {round(targetDistance,1)}m")  # For Debugging
            if targetDistance < 3:
                break
            time.sleep(2)
        print(f"Reached Waypoint {self.current_waypoint}")

    def guided_mission(self):
        while True:
            self.go_there()
            self.check_if_reached()
            if not self.return_to_launch:
                self.planting()
            else:
                self.change_mode("RTL")
                time.sleep(30)  # waiting to RTL
                break

        print("Done i guess")

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to rover on 1 Hz cycle
        self.vehicle.send_mavlink(msg)
        time.sleep(duration)

    def key(self, event):
        if event.char == event.keysym:  # -- standard keys
            if event.keysym == 'r':
                print("r pressed >> Set the vehicle to RTL")  #Return-to-Launch
                self.change_mode("RTL")

        else:  # -- non standard keys
            if event.keysym == 'Up':
                print("UP")
                self.send_ned_velocity(self.walk_speed, 0, 0, 1)
            elif event.keysym == 'Down':
                print("Down")
                self.send_ned_velocity(-self.walk_speed, 0, 0, 1)
            elif event.keysym == 'Left':
                print("Left")
                self.send_ned_velocity(0, -self.walk_speed, 0, 1)
            elif event.keysym == 'Right':
                print("Right")
                self.send_ned_velocity(0, self.walk_speed, 0, 1)
