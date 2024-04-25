from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, mavutil, Vehicle
import conversions
import math
import ImageDetectionHelper
from threading import Thread
#import RPi.GPIO as IO
import time

laserpin = 13
# IO.setwarnings(False)
# IO.setmode(IO.BOARD)
# IO.setup(laserpin, IO.OUT)
# pulse = IO.PWM(laserpin, 50)
# pulse.start(5)
hover_height = 1
captured_ids = []
calib_data_path = "../calib_data/MultiMatrix.npz"
camera = ImageDetectionHelper.Camera(calib_data_path)

def wait_scan(id):
    #  if no target in view, wait & scan for 5 seconds
    #create a timer to allow the drone to continue scanning for UGV in case of failure
    coords = camera.scan_ugv(id)
    t_start = time.perf_counter()
    t_end = t_start + 5
    while (time.perf_counter() < t_end and coords == 0):
        print("searching")
        coords = camera.scan_ugv(id)
        time.sleep(.1)
    if (coords != 0):
        return coords
    else:
        print("No target in view")
        return 0

def track_hover(id, duration):
    time_end = time.perf_counter() + duration
    while (time.perf_counter() < time_end):
        coords = wait_scan(id)
        if (coords == 0):  # no target in view after 5 seconds
            print("No target in view")
            return 0
        coords = coords[1]
        x_vel, y_vel, is_offcenter = conversions.coords_to_vel(coords)
        z_vel = 0
        if (coords[2] < (hover_height-0.2)):
            z_vel = -0.1  # move up
        elif (coords[2] > (hover_height+0.2)):
            z_vel = 0.1  # move down
        print("image coordinates:", coords)
        if (is_offcenter):
            print("x_vel:",x_vel,"\ny_vel:",y_vel,"\nz_vel:",z_vel)
        else:
            print("x_vel:",0,"\ny_vel:",0,"\nz_vel:",z_vel)
        time.sleep(.1)

def fire_laser_helper(seconds) :
    # Simple python script to turn IO pin on for 3 seconds then off
    # RUN WITHIN NEW THREAD
    print("Laser on")
    pulse.ChangeDutyCycle(10)
    time.sleep(seconds)
    pulse.ChangeDutyCycle(5)

def fire_laser(seconds):
    # create a new thread to fire the laser
    laser_thread = Thread(target=fire_laser_helper, args=[seconds])
    laser_thread.start()

field_angle = 0
def attack_ugv(id):
    """
    ### TESTING: ADJUST SEND_VELOCITY DURATION VALUE & VEL_FACTOR VALUE ###
    function to attack a target UGV
        * ugv_coords is the camera coordinates of the UGV (50x35 pixels for now)
        * returns to start_location at finish
        * drone will move forward/backward and left/right to keep the UGV in the center of the frame
            - center of frame is considered within a 50 pixel box around the center of the frame
        * drone will move down at a constant rate until it is at a specific height above the UGV
        * drone will fire twice at the target and then return to start_location
    """
    coords = wait_scan(id)  # scan for UGV
    if (coords == 0):  # no target in view after 5 seconds
        return 0
    id = coords[0]
    coords = coords[1]
    while (coords[2] > hover_height):  # will fly down until 40 in (1 m) above the ground, then fire the laser
        coords = wait_scan(id)
        if (coords == 0):  # no target in view after 5 seconds
            return 0
        coords = coords[1]
        
        # calculate velocity values according to the coordinates of the UGV
        x_vel, y_vel, is_offcenter = conversions.coords_to_vel(coords)

        print("Altitude: ", coords[2])
        if (is_offcenter):
            print("x_vel:",x_vel,"\ny_vel:",y_vel),"\nz_vel:",0
        else:
            print("x_vel:",0,"\ny_vel:",0),"\nz_vel:",0.3
        print("image coordinates:", coords)
        time.sleep(.1)
    
    fire_len = 1
    # fire_laser(fire_len)
    track_hover(id, fire_len+10)
    # fire_laser(fire_len)
    track_hover(id, fire_len)

    camera.set_incapacitated(id)  # disable camera from tracking this UGV
    captured_ids.append(id)
    return 1


if __name__ == "__main__":

    attack_ugv(-1)
