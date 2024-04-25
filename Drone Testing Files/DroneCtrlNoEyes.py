from dronekit import connect, VehicleMode, mavutil
import conversions
# import ImageDetectionHelper as ImageDetection
#import ACE_Combat
#from IRC_Class import *
import time

class Drone:
    #connect to IRC channel
    #irc = IRC("TAMU_Drone", "TAMU_RTX_Drone", "password")
    #irc.connect()  # channel, botnick
    # initialize camera
    calib_data_path = "../calib_data/MultiMatrix.npz"
    # camera = ImageDetection.Camera(calib_data_path)
    captured_ids = []
    # initialize drone
    def __init__(self, flyspeed, flyheight, fallspeed, f, simulated=True):
        # connect to drone
        self.simulated = simulated
        #if (not simulated):
            #self.ACE = ACE_Combat.ACE_Combat()
        pathtoDrone = self.get_drone_path(simulated)
        print("Connecting to drone on: %s" % pathtoDrone)
        self.drone = connect(pathtoDrone, baud=57600, wait_ready=True, rate=10, heartbeat_timeout=15)
        cmds = self.drone.commands
        cmds.download()
        # set drone parameters
        self.drone.groundspeed = flyspeed
        self.flyheight = flyheight
        self.fallspeed = fallspeed
        self.hover_height = 3  # height (meters) to hover at when attacking UGV
        self.home = self.drone.location.global_frame  # home position is the location of the drone when it is armed at fly height
        self.home.alt = self.flyheight + self.home.alt
        self.field_angle = self.drone.heading
        self.f = f
        print("Drone initialized")

    def get_drone_path(self, simulated=False):  # returns the physical path of the drone, either the serial port or the simulated port
        if (simulated):
            import dronekit_sitl
            sitl = dronekit_sitl.start_default()
            #return "127.0.0.1:14550"
            #return "127.0.0.1:5501"
            #return "192.168.240.1:14550"
            return "localhost:14550"
        else:
            return "/dev/serial0"

    def get_location(self, relative=False):
        if relative:
            return self.drone.location.global_relative_frame
        else:
            return self.drone.location.global_frame

    def switch_mode(self, mode_str):
        self.drone.mode = VehicleMode(mode_str)
        while self.drone.mode!=mode_str:
            print("Waiting for drone to enter", mode_str, "mode")
            time.sleep(1)

    def take_off(self, alt=5):
        # arming the drone
        i=0
        print ("arming drone")
        while not self.drone.is_armable:
            print("Waiting for vehicle to become armable")
            time.sleep(2)
            if (i==10):
                break
            i += 1
        self.drone.arm(wait=True, timeout=15)

        # takeoff to alt (meters)
        self.drone.simple_takeoff(self.flyheight)
        while True:
            #Break and return from function just below target altitude.
            if self.drone.location.global_relative_frame.alt>=self.flyheight*0.95:
                print ("Takeoff complete")
                break
            time.sleep(1)


    def fly_up(self, alt):
        """
        don't use for flying down
        """
        target_location = self.drone.location.global_relative_frame
        target_location.alt += alt
        self.drone.simple_goto(target_location)
        while True:
            print (" Altitude: ", self.drone.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if self.drone.location.global_relative_frame.alt>=target_location.alt*0.95:
                print ("Reached target altitude")
                break
            time.sleep(1)
        print("Current coords:", self.drone.location.global_relative_frame)

    def send_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        * Only use one instance of this function at a time, might not be efficient to use multiple instances
        * x = North, y = East, z = Down
        * Positive velocity_z will move downward and negative velocity_z will move upward
        """
        velocity_x, velocity_y = conversions.field_to_global_vel(velocity_x, velocity_y, self.drone.heading)
        velocity_x = round(velocity_x, 3)
        velocity_y = round(velocity_y, 3)
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        if (duration > 1):  # send multiple times at 1 Hz for duration > 1
            for x in range(0,duration):
                self.drone.send_mavlink(msg)
                time.sleep(1)
        else:  # send once for duration < 1
            self.drone.send_mavlink(msg)
            time.sleep(duration)

    def set_yaw(self, angle, rotation_speed=30):
        """
        sets the yaw at an angle relative to the field
        """
        current_heading = self.drone.heading  # could also use self.field_angle
        target_heading = self.field_angle+angle
        if target_heading > 360:
            target_heading -= 360

        angle_diff = target_heading - current_heading  
        if angle_diff < 0:
            if angle_diff < -180:
                direction = 1
            else:
                direction = -1
        else:
            if angle_diff < 180:
                direction = 1
            else:
                direction = -1
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.drone.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            target_heading,    # param 1, yaw in degrees
            rotation_speed,          # param 2, yaw speed deg/s
            direction,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.drone.send_mavlink(msg)
        while (abs(self.drone.heading-target_heading) > 5):
            time.sleep(0.25)
        time.sleep(.5)  # wait for drone to finish turning

    def goto_location(self, target_location):
        self.drone.simple_goto(target_location)
        while True:
            remainingDistance=conversions.get_distance_metres(self.drone.location.global_frame, target_location)
            if remainingDistance<=1.5: #Close enough target, in case of undershoot
                break
            time.sleep(.5)
        return 1

    def goto_plot_coord(self, x, y):
        """
        Goes to x,y in field coordinates. 
        Converts to global coordinates, and sends drone to that location
        """
        targetLocation = conversions.plot_to_global_coords(self.home, x, y, self.field_angle)  # convert to global coordinates
        self.goto_location(targetLocation)

    def land(self):
        """
        need to alter the finish location to make it at (25,51) (x,y) relative to the start pos (0,0)
        """
        self.switch_mode("LAND")
        print("Landing, shutting down")

    # def wait_scan(self, id=-1):
    #     #  if no target in view, wait & scan for 5 seconds
    #     #create a timer to allow the drone to continue scanning for UGV in case of failure
    #     duration = 2.5
    #     coords = self.camera.scan_ugv(id)
    #     t_start = time.perf_counter()
    #     t_end = t_start + duration
    #     while (time.perf_counter() < t_end and coords == 0):
    #         coords = self.camera.scan_ugv(id)
    #         time.sleep(.1)
    #     if (coords != 0):
    #         return coords
    #     else:
    #         self.f.write("No target in view\n")
    #         return 0
    
    # def track_hover(self, id, duration):
    #     time_end = time.perf_counter() + duration
    #     while (time.perf_counter() < time_end):
    #         coords = self.wait_scan(id)
    #         if (coords == 0):  # no target in view after 5 seconds
    #             return 0
    #         coords = coords[1]
    #         x_vel, y_vel, is_offcenter = conversions.coords_to_vel(coords)
    #         z_vel = 0
    #         if (coords[2] < (self.hover_height-0.2)):
    #             z_vel = -0.1  # move up
    #         elif (coords[2] > (self.hover_height+0.2)):
    #             z_vel = 0.1  # move down
    #         if (is_offcenter):
    #             self.send_velocity(x_vel, y_vel, z_vel, .1)
    #         else:
    #             self.send_velocity(0, 0, z_vel, .1)
    #     return 1
    
    # def attack_ugv(self, id):
    #     """
    #     ### TESTING: ADJUST SEND_VELOCITY DURATION VALUE & VEL_FACTOR VALUE ###
    #     function to attack a target UGV
    #         * ugv_coords is the camera coordinates of the UGV (50x35 pixels for now)
    #         * returns to start_location at finish
    #         * drone will move forward/backward and left/right to keep the UGV in the center of the frame
    #             - center of frame is considered within a 50 pixel box around the center of the frame
    #         * drone will move down at a constant rate until it is at a specific height above the UGV
    #             - only move down once the UGV is in the center of the frame
    #         * drone will fire twice at the target and then return to start_location
    #     """
    #     coords = self.wait_scan(id)
    #     if (coords == 0):  # no target in view after 5 seconds
    #         return 0
    #     coords = coords[1]  # coords = (x,y,z)
    #     while (coords[2] > self.hover_height):  # will fly down until set height above the ground, then fire the laser
    #         coords = self.wait_scan(id) # scan for ugv
    #         if (coords == 0):  # no target in view after 5 seconds
    #              return 0
    #         coords = coords[1]
            
    #         # calculate velocity values according to the coordinates of the UGV
    #         x_vel, y_vel, is_offcenter = conversions.coords_to_vel(coords)

    #         if (is_offcenter):
    #             self.send_velocity(x_vel, y_vel, 0, .1)  # last argument affects the reaction time of horizontal movement
    #         else:
    #             self.send_velocity(0, 0, self.fallspeed, .1)
        
    #     #  fire laser twice and hover over target for 10 seconds
    #     fire_len = 1
    #     current_location = self.drone.location.global_frame
    #     #self.irc.send_fire(id, current_location)
    #     #if (not self.simulated):
    #         #self.ACE.fire_laser(fire_len)
    #     self.track_hover(id, fire_len+3)
    #     self.f.write("UGV hit!\n")
    #     self.track_hover(id, fire_len) # track UGV and hover over it for fire duration

    #     self.camera.set_incapacitated(id)  # disable camera from tracking this UGV
    #     self.captured_ids.append(id)  # not necessary, but useful for debugging
    #     return 1

    def __del__(self):
            print("closing drone connection")
            self.drone.close()
            # print("closing camera connection")
            # self.camera.__del__()
