import time
from collections.abc import MutableMapping
from multiprocessing import connection

import DroneCtrl
import conversions
#import ImageDetectionHelper
from dronekit import LocationGlobalRelative, VehicleMode, connect

sim_bool = True
fly_height = 2
fall_speed = .3
fly_speed = 1

def main() :
    global HOME, fly_height  # global variables
    Drone = DroneCtrl.Drone(fly_speed, fly_height, fall_speed, sim_bool)
    # Switch to GUIDED, arm and then take off
    Drone.switch_mode("GUIDED")
    Drone.take_off(fly_height)

    scan = drone.wait_scan()
    ugv_id = scan[0]
    print("UGV", ugv_id, "detected at: %s" % drone.get_location(True))
    f.write("UGV %s detected at: %s\n" % (ugv_id, drone.get_location(True)))


    print("Current location: %s" % Drone.get_location())
    print("Home position: %s" % Drone.home)
    print("Current heading: %s" % Drone.field_angle)
    Drone.send_velocity(0,0,0,10)
    print("Drone now landing")
    Drone.land()

if __name__ == "__main__":
    main()