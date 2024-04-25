import time
from collections.abc import MutableMapping
from multiprocessing import connection

import DroneCtrl
import conversions
import ImageDetectionHelper


sim_bool = True
fly_height = 2
fall_speed = .3
fly_speed = 1

def main() :
    global HOME, fly_height  # global variables
    drone = DroneCtrl.Drone(fly_speed, fly_height, fall_speed, sim_bool)

    #TEST Vision

    

    if (drone.wait_scan() != 0):  # scans for UGVs
            """
            This is where the drone attacks the UGV
            """
            drone.play_tune()
            f.write("Found marker\n")

    save_location = drone.get_location(True)
    scan = drone.wait_scan()
    if(scan == 0):
        f.write("UGV lost\n")

    ugv_id = scan[0]
    print("UGV", ugv_id, "detected at: %s" % drone.get_location(True))
    f.write("UGV %s detected at: %s\n" % (ugv_id, drone.get_location(True)))
    if(drone.attack_ugv(ugv_id) == 0):
        f.write("Attack failed\n")
