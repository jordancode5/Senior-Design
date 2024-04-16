import cv2 as cv
from cv2 import aruco
import numpy as np
#import pyrealsense2 as rs
import datetime

targetList = np.array([])

# will delete all but necessary aruco modules here we don't need them loaded
ARUCO_DICT = {  # all standard aruco libraries listed
    "DICT_4X4_50": cv.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
}

# fire control function

def engage(arucoid, engagedlist):
    if arucoid in engagedlist:
        print("Already engaged")
    else:
        engagedlist.append(arucoid)

# function that displays the aruco

def aruco_display(corners, ids, rejected, image, tvec, rvec):
    if len(corners) > 0:

        ids = ids.flatten()
									#new line
        for (markerCorner, markerID) in zip(corners, ids):  # added i in and total markers
            #how do I iterate through this stuff?
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            color = (0, 255, 0)             #default color green
            #print("MarkerID =", markerID)   #debug statement

            global targetList       #I point out that targetList is a global variable

            distance = np.sqrt(
                tvec[0][0][2] ** 2 + tvec[0][0][0] ** 2 + tvec[0][0][1] ** 2
                # new function to estimate distance of each aruco Marker
            )
            #print("distance =", round(distance, 2))
            intdistance = distance
            #distance = format(round(distance, 2), 'f')
            cx = int((topLeft[0] + bottomRight[0]) / 2.0)
            cy = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv.circle(image, (cx, cy), 4, (0, 0, 255), -1)
            cv.putText(image,
                       f"{str(markerID)} Dist: {round(distance, 2)} "
                       f"X: {round(tvec[0][0][0], 2)} "
                       f"Y: {round(tvec[0][0][1], 2)} "
                       f"Z: {round(tvec[0][0][2], 2)}",
                       (topLeft[0], topLeft[1] - 40), cv.FONT_HERSHEY_SIMPLEX,
                       0.5, (255, 0, 0), 2)
            cv.putText(image,
                       f"Rotation - X: {round(180 / 3.14 * rvec[0][0][0] - 180, 3)} "
                       f"Y: {round(180 / 3.14 * rvec[0][0][2], 3)} "
                       f"Z: {round(180 / 3.14 * rvec[0][0][1], 3)}",
                       (topLeft[0], topLeft[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
                       0.5, (255, 0, 0), 2)

            if markerID in targetList:
                #print("already engaged")
                color = (0, 0, 255)         #change color to red during engagement
            elif intdistance < 1:
                targetList = np.append(targetList, markerID)
                color = (0, 0, 255)         #change color to red if engaged
                print(targetList)

            cv.line(image, topLeft, topRight, color, 2)
            cv.line(image, topRight, bottomRight, color, 2)
            cv.line(image, bottomRight, bottomLeft, color, 2)
            cv.line(image, bottomLeft, topLeft, color, 2)

    return image

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.aruco_dict = cv.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv.aruco.DetectorParameters()

    corners, ids, rejected_img_points = cv.aruco.detectMarkers(gray, cv.aruco_dict, parameters=parameters)
    # cameraMatrix=matrix_coefficients,              #detect markers no longer deals with intrinsics
    # distCoeff=distortion_coefficients)              #should I remove this as well?
    # print("len(ids) =", len(ids))
    if len(corners) > 0:
        for i in range(0, len(ids)):
            #print("pose estimation loop has run", i, "times")
            #print("current id =", ids[i])
            rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i], 0.096, matrix_coefficients,
                                                                          distortion_coefficients)

            # rvec = rotation vector
            # tvec = translation vector
            cv.aruco.drawDetectedMarkers(frame, corners)  # draw around detected markers

            frame = aruco_display(corners[i], ids[i], rejected_img_points, frame, tvec, rvec)  # displays the aruco designator

            # cv.aruco.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)   #changed from drawAxis to drawFrameAxes
            frame = cv.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec,
                                     0.01)  # changed from drawAxis to drawFrameAxes

    return frame

aruco_type = "DICT_6X6_250"     #define which aruco Library is needed

arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
# detect the marker
param_markers = aruco.DetectorParameters()
# will likely include a function here for determining dimension and including a larger dictionary in this
# utilizes default camera/webcam driver

#must find real intrinsic camera values this is messing up program
intrinsic_camera = np.array(((933.15867, 0, 657.59),(0,933.1586, 400.36993),(0,0,1)))
#need to find real distortion values
distortion = np.array((-0.43948,0.18514,0,0))

# use a realsense camera
#pipe = rs.pipeline()
#cfg = rs.config()

#cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

#pipe.start(cfg)

# initialize the video capture object
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)  #max width is 1280 pixels
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)  #max height is 720 pixels

while True:
    # start time to compute the fps
    start = datetime.datetime.now()

    #rsFrame = pipe.wait_for_frames()

    #color_frame = rsFrame.get_color_frame()
    #depth_frame = rsFrame.get_depth_frame()

    # depth_image = np.asanyarray(depth_frame.get_data())       #potentially use depth
    #color_image = np.asanyarray(color_frame.get_data())
    #frame = color_image

    ret, frame = cap.read()

    # if there are no more frames to process, break out of the loop
    # if not ret:
    #     break

    # run the YOLO model on the frame
    # detections = model(frame)[0]

    output = pose_estimation(frame, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)

    # output2 = aruco_display()
    # cv.imshow('Estimated Pose', output)
    # cv.imshow('ID', output2)

    key = cv.waitKey(1) & 0xFF
    if key == ord('q'):
        print("Target List =", np.sort(targetList))
        break

    end = datetime.datetime.now()
    # show the time it took to process 1 frame
    total = (end - start).total_seconds() + 0.00001
    print(f"Time to process 1 frame: {total * 1000:.0f} milliseconds")

    # calculate the frame per second and draw it on the frame
    fps = f"FPS: {1 / total:.2f}"
    cv.putText(frame, fps, (50, 50),
                cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 8)

    # show the frame to our screen
    cv.imshow("OpenCV implementation", frame)

# cap.release()     #webcam
pipe.stop()     # realsense camera
cv.destroyAllWindows()

