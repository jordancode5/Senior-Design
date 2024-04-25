import time
from collections.abc import MutableMapping
from multiprocessing import connection

import DroneCtrl
import conversions
import ImageDetectionHelper
from dronekit import LocationGlobalRelative, VehicleMode, connect

sim_bool = True
fly_height = 5
fall_speed = .3
fly_speed = 1

def main() :
    global HOME, fly_height  # global variables
    Drone = DroneCtrl.Drone(fly_speed, fly_height, fall_speed, sim_bool)
    # Switch to GUIDED, arm and then take off
    Drone.switch_mode("GUIDED")
    Drone.take_off(fly_height)
    print("Current field_angle: %s" % Drone.field_angle)
    Drone.fix_yaw(270)
    print("Current heading: %s" % Drone.drone.heading)
    Drone.send_velocity(0,0,0,3)  # hold position for 3 seconds
    Drone.fix_yaw(0)
    print("Current heading: %s" % Drone.field_angle)
    print("Drone now landing")
    Drone.land()

if __name__ == "__main__":
    main()