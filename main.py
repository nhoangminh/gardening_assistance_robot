import sys
import tkinter as tk
from initializer import *
from dronekit_guided import Controller


if __name__ == "__main__":
    rover = Controller(current_waypoint, drive_speed, walk_speed, return_to_launch=False)

    # OBTAINING WAYPOINT COORDINATES
    import_mission_filename = 'demo.waypoints'
    n_WP, waypoint_list, waypoint_iter = rover.obtain_waypoints(import_mission_filename)
    # print(n_WP, waypoint_list)  # For Debugging

    # CHECK 1: APPROPRIATE WAYPOINT MISSION
    if n_WP > 0:
        print("A valid mission has been uploaded\n")
    else:
        print("Invalid mission... Terminating Script\n")
        sys.exit()

    # INITIALISING ROVER
    rover.connect_my_vehicle(connection_string = "udp:127.0.0.1:14551")  # Originally, "/dev/serial0", baud=921600
    rover.arm_and_guided()

    # GUIDED MOVEMENT
    # rover.guided_mission()

    # MANUAL MOVEMENT (for Seed over hole)
    rover.planting()
    # rover.send_ned_velocity(walk_speed, 0, 0, 1)  # [Edwin] Duration '1' can only be found though experimentation
    # root = tk.Tk()
    # print(">> Control the drone with the arrow keys. Press r for RTL mode")
    # root.bind_all('<Key>', rover.key)
    # root.mainloop()
    
    # CLEAN UP
    rover.hold_and_disarm()
    print("End of Program")


# ADDITIONAL FUNCTIONALITIES
# import os, and press 'X' key to shutdown pi
# PiCamera, and press 'X' key to stream tank POV to laptop
