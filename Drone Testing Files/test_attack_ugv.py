import time
import DroneCtrl

sim_bool = False
fly_height = 8
fall_speed = .5
fly_speed = 1

def main() :
    global HOME, fly_height  # global variables
    Drone = DroneCtrl.Drone(fly_speed, fly_height, fall_speed, sim_bool)
    # Switch to GUIDED, arm and then take off
    Drone.switch_mode("GUIDED")
    Drone.take_off(fly_height)
    Drone.send_velocity(0,0,0,3)  # hold position for 3 seconds
    time_start = time.perf_counter()
    save_location = Drone.get_location(True)

    print("Current heading: %s" % Drone.drone.heading)
    print("Field angle: %s" % Drone.field_angle)
    print("Fixing yaw, starting scan")
    Drone.fix_yaw()
    scan = Drone.wait_scan()
    if (scan == 0):  # no target in view after 5 seconds
        Drone.land()
        return 0
    print("UGV found at:", Drone.get_location(True))
    ugv_id = scan[0]
    if(Drone.attack_ugv(ugv_id) == 0):
        print("Attack failed")
    else:
        print("Attack successful")

    print("Going to previous location")
    Drone.goto_location(save_location)
    print("Time elapsed: %s" % (time.perf_counter() - time_start))
    print("Drone now landing")
    Drone.land()

if __name__ == "__main__":
    main()