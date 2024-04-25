import time
from collections.abc import MutableMapping
from multiprocessing import connection

import DroneCtrl
import conversions
#import ImageDetectionHelper
from dronekit import LocationGlobalRelative, VehicleMode, connect

sim_bool = False
fly_height = 5
fall_speed = .3
fly_speed = 1

def main() :
    global HOME, fly_height  # global variables
    Drone = DroneCtrl.Drone(fly_speed, fly_height, fall_speed, sim_bool)
    # Switch to GUIDED, arm and then take off
    Drone.switch_mode("GUIDED")
    Drone.take_off(fly_height)
    Drone.send_velocity(0,0,0,3)  # hold position for 3 seconds
    print("Current location: %s" % Drone.get_location())
    print("Home position: %s" % Drone.home)
    print("Current heading: %s" % Drone.field_angle)
    Drone.goto_plot_coord(1,3)
    print("Current location: %s" % Drone.get_location())
    Drone.goto_location(Drone.home)
    print("Drone now landing")
    Drone.land()

if __name__ == "__main__":
    main()