import time
import DroneCtrl

f = open(f'log_showcase1.txt', 'w')
sim_bool = True
captured_ugvs = 0
HOME = 0
fly_height = 4
fly_speed = 2
fall_speed = 0.8
State = 'A'

def main() :
    global sim_bool, fly_speed, captured_ugvs, HOME, fly_height, fall_speed, State  # global variables
    
    time_start = time.perf_counter()
    drone = DroneCtrl.Drone(fly_speed, fly_height, fall_speed, f, sim_bool)

    # Switch to GUIDED, arm and then take off
    drone.switch_mode("GUIDED")
    drone.take_off(fly_height)
    drone.send_velocity(0,0,0,3)
    print("Current heading: %s" % drone.drone.heading)
    drone.set_yaw(0)
    # create the path for the drone to follow
    path_list = [[1,.5], [1, 1], [1,1.5], [1, 2], [1,2.5], [1, 3], [1,3.5], [1, 4], [1,4.5],
                 [1.5,4.5], [1.5, 4], [1.5,3.5], [1.5, 3], [1.5,2.5], [1.5, 2], [1.5,1.5], [1.5, 1], [1.5,.5],
                 [2,.5], [2, 1], [2,1.5], [2, 2], [2,2.5], [2, 3], [2,3.5], [2, 4], [2,4.5],
                 [2.5,4.5], [2.5, 4], [2.5,3.5], [2.5, 3], [2.5,2.5], [2.5, 2], [2.5,1.5], [2.5, 1], [2.5,.5],
                 [3,.5], [3, 1], [3,1.5], [3, 2], [3,2.5], [3, 3], [3,3.5], [3, 4], [3,4.5]]
    path_index = 0  # need to save last coordinate in the path list if a UGV is detected
    while(True):
        if(State == 'A'):
            """
            This is where the path for the drone is performed
            """
            # if the drone has reached the end of the path
            if(path_index >= len(path_list) or captured_ugvs >= 3):  # need to capture 3 UGVs
                f.write("All UGVs captured\n")
                State = 'C'
                continue
            coords = path_list[path_index]
            drone.goto_plot_coord(coords[0], coords[1])
            f.write("Captured UGVs: %s\n" % drone.captured_ids)
            if (drone.wait_scan() != 0):  # scans for UGVs
                State = 'B'
                path_index += 1
                continue
            path_index += 1
        elif(State == 'B'):
            """
            This is where the drone attacks the UGV
            """
            save_location = drone.get_location(True)
            scan = drone.wait_scan()
            if(scan == 0):
                f.write("UGV lost\n")
                State = 'A'
                continue
            ugv_id = scan[0]
            print("UGV", ugv_id, "detected at: %s" % drone.get_location(True))
            f.write("UGV %s detected at: %s\n" % (ugv_id, drone.get_location(True)))
            if(drone.attack_ugv(ugv_id) == 0):
                f.write("Attack failed\n")
                drone.goto_location(save_location)
                State = 'A'
                path_index -= 1
                continue
            
            drone.goto_location(save_location)
            captured_ugvs += 1
            State = 'A'
        
        elif(State == 'C'):
            print("drone ready to land")
            #drone.goto_location(drone.home)
            drone.land()  # lands the drone at the finish line
            print("UGV's Captured:", captured_ugvs)
            print("Time taken:", time.perf_counter()-time_start)
            f.close()
            break

if __name__ == "__main__":
    main()
