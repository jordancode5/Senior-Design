import cv2 as cv
from cv2 import LINE_AA
import cv2.aruco as aruco
import numpy as np
import os

print("OpenCV version:", cv.__version__)

class arUcoMarker:
    def __init__(self, id, distance, x_coord, y_coord, z_coord, top_right, top_left, bottom_right, bottom_left, friendBool, incapacitatedBool, inCameraBool):
        self.id = id
        self.distance = distance
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.z_coord = z_coord
        self.top_right = top_right
        self.top_left = top_left
        self.botton_right = bottom_right
        self.bottom_left = bottom_left
        self.incapacitatedBool = incapacitatedBool
        self.inCameraBool = inCameraBool
        
    def updateAttributes(self, distance, x_coord, y_coord, z_coord, top_right, top_left, bottom_right, bottom_left):
        self.distance = distance
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.z_coord = z_coord
        self.top_right = top_right
        self.top_left = top_left
        self.bottom_right = bottom_right
        self.bottom_left = bottom_left

    def set_incapacitated(self):
        self.incapacitatedBool = True

    def inCamera(self, markerIDs):
        if markerIDs is not None:
            newmarkerIDs = markerIDs.reshape(-1)
            if (self.id not in newmarkerIDs):
                self.inCameraBool = False
            else:
                self.inCameraBool = True
        else:
            self.inCameraBool = False
        
    def printData(self):
        f = open(f'arUcoMarker_data/arUcoMarker_{self.id}.txt', 'w')
        f.write(f'ID: {self.id} \n')
        f.write(f'Distance: {self.distance} in \n')
        f.write(f'X: {self.x_coord} in \n')
        f.write(f'Y: {self.y_coord} in \n')
        f.write(f'Z: {self.z_coord} in \n')
        f.write(f'Top Right: {self.top_right} px \n')
        f.write(f'Top Left: {self.top_left} px \n')
        f.write(f'Bottom Right: {self.bottom_right} px \n')
        f.write(f'Bottom Left: {self.bottom_left} px \n')
        f.write(f'Friend: {self.friendBool} \n')
        f.write(f'Incapacitated: {self.incapacitatedBool} \n')
        f.write(f'In Camera: {self.inCameraBool} \n')

intrinsic_camera = np.array(((933.15867, 0, 657.59),(0,933.1586, 400.36993),(0,0,1)))
#need to find real distortion values
distortion = np.array((-0.43948,0.18514,0,0))

class Camera:
    def __init__(self, calib_data_path):
        self.dir = "arUcoMarker_data"
        #calib_data = np.load(calib_data_path)       # what on earth is this calibration data?
        #self.cam_matrix = calib_data["camMatrix"]
        self.cam_matrix = intrinsic_camera
        self.dist_coef = distortion
        #self.dist_coef = calib_data["distCoef"]
        #self.r_vectors = calib_data["rVector"]          # rotational vector (not used)
        #self.t_vectors = calib_data["tVector"]          # translational vector (stuff you need)
        self.MARKER_SIZE = 30.48 #centimeters
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)      # used marker dictionary
        self.param_markers = aruco.DetectorParameters()
        self.arUcoMarker_list = {}  # dictionary of arUcoMarkers
        self.incapacitated_list = []  # list of incapacitated UGVs, matches each ArUco object incapacitated bool
        self.cap = cv.VideoCapture(0)  # Start capture
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)  #max width is 1280 pixels
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)  #max height is 720 pixels
        
        self.lockedID = -1

    def set_incapacitated(self, id):
        self.incapacitated_list.append(id)
        self.arUcoMarker_list[id].set_incapacitated()

    def scan_ugv(self, id=-1):
        """
        function to return the camera coordinates of the identified UGV, if any
            * make sure to lock in on only one UGV if multiple are in view (could be done using the closest UGV)
            * won't ignore a target until it is attacked, or the target is out of view
            * if no enemy UGV is in view, return None
            * return's the coordinates (x,y) with origin at the center of the camera frame
        """
        ret, frame = self.cap.read()

        # cv.imshow("OpenCV implementation", frame) # not working for some reason

        # key = cv.waitKey(1) & 0xFF
        # if key == ord('q'):
        #    print("closing frame")

        if ret == False:
            print("There was a problem")
            return 0
        #else:
        #    print("there was not a problem")

        #set the lockedID, if any
        self.lockedID = id

        # convert to grayscale
        grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # show the image very useful for debugging
        # * note if you are not connected to a virtual or visual screen this will draw an error
        cv.imshow("OpenCV implementation", grayFrame) # not working for some reason

        marker_corners, marker_IDs, reject = aruco.detectMarkers( # marker detector
            grayFrame, self.marker_dict, parameters=self.param_markers)

        if marker_corners: #only operate calculations/displaying if there are markers on the screeng

            # Determine which marker to lock onto for calculating coordinates
            total_markers = range(0, marker_IDs.size)

            #if there is a locked marker, only calculate coordinates for that marker
            if self.lockedID != -1:
                # for loop that iterates through every arUco marker on the screen, checking if it's being attacked
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    if self.lockedID == ids[0]:
                        break
            # if there is no locked marker, calculate coordinates for the next available marker
            else:
                #for loop for each arUco marker on the screen
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    # if the marker is not a friend and not incapacitated
                    # be careful with this data format
                    if( ids[0] == 7 ):  # insert your UGV aruco marker here (ours was 7)
                        print("friendly marker detected, ignoring it")
                    if( (ids[0] != 7) and (ids[0] not in self.incapacitated_list) ):
                        self.lockedID = ids[0]
                        break
            
            #If marker isnt in array, put marker in array
            if marker_IDs[i][0] not in self.arUcoMarker_list.keys():
                #print(f"added id: {marker_IDs[i][0]}")
                #initialize the new arUco marker with fake data to save time
                self.arUcoMarker_list[marker_IDs[i][0]] = arUcoMarker(marker_IDs[i][0], 1000, 0, 0, 0, 0, 0, 0, 0, False, False, True)
            
            #if the locked on arUco marker is gone and another one is in view, return 0
            if self.lockedID != ids[0]:
                return 0
            
            #Calculate the coordinates of that arUco marker if it's being attacked
            #Focuses on one arUco marker on the screen
            marker_corners = [corners]
            # rVect = rotational vector
            # tVect = translational vector (x, y, and z positions) aka: the important ones
            rVect, tVect, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.MARKER_SIZE, self.cam_matrix, self.dist_coef)
            
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_left = tuple(corners[0].ravel())
            top_right = tuple(corners[1].ravel())
            bottom_right = tuple(corners[2].ravel())
            bottom_left = tuple(corners[3].ravel())

            #Below draws axis on each arUco marker
            point = cv.drawFrameAxes(frame, self.cam_matrix, self.dist_coef, rVect[0], tVect[0], 3, 2)

            #XYZ Coords for each arUco marker
            x_coord = tVect[0][0][0] * 0.393701 #X (inches)
            y_coord = -1 * tVect[0][0][1] * 0.393701 #Y (inches)
            z_coord = tVect[0][0][2] / 100 #Z (meters)

            # Uses pythagorean theorum for x y and z, measures in centimeters
            # determine the range of your blaster and compare later
            distance = np.sqrt(x_coord ** 2 + y_coord ** 2 + z_coord ** 2) 
            
            #attributes have to be updated because they will change after initialization.
            self.arUcoMarker_list[marker_IDs[i][0]].updateAttributes(distance, x_coord, y_coord, z_coord, top_right, top_left, bottom_right, bottom_left)
            #self.arUcoMarker_list[marker_IDs[i][0]].incapacitated()

            #marker_IDs[i][0] are the IDs of each arUco marker, which are also the keys to access each marker in the dictionary

        #Return coordinates of arUco marker, or 0 if none detected
        if marker_corners:
            return (ids[0], (round(x_coord,1), round(y_coord,1), round(z_coord,1)) )
        else:
            return 0
        
    def __del__(self):
        #End capture
        self.cap.release()
        cv.destroyAllWindows()

def main():

    # my line is:
    # cap = cv.VideoCapture(0)
    # calib_data = ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    calib_data_path = "../calib_data/MultiMatrix.npz" # wesley code
    camera = Camera(calib_data_path)
    id = -1
    while(True):
        ugv_scan = camera.scan_ugv(id)
        if ugv_scan != 0:
            id = ugv_scan[0]
            coords = ugv_scan[1]
            print("id:", id, "x:", coords[0], "y:", coords[1], "z:", coords[2])
    

if __name__ == "__main__":
    main()