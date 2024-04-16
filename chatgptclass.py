import cv2
import numpy as np

class Camera:
    def __init__(self):
        self.id = -1  # Initialize id attribute

    def scan_ugv(self):
        # Capture video from the default camera (0)
        cap = cv2.VideoCapture(0)

        # Load ArUco dictionary
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

        # Initialize ArUco parameters
        parameters = cv2.aruco.DetectorParameters_create()

        while True:
            # Read frame from the camera
            ret, frame = cap.read()
            if not ret:
                print("Error: Cannot read frame.")
                break

            # Detect ArUco markers
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            # If markers are detected
            if ids is not None:
                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Assume only one marker is detected for simplicity
                marker_index = 0  # Index of the first detected marker
                detected_id = ids[marker_index][0]

                # Get rvec and tvec of the detected marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion_coeff)

                # Update self.id with the detected ID
                self.id = detected_id

                # Return rvec and tvec
                return rvec, tvec

            # Display the frame
            cv2.imshow('Frame', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the video capture and close all OpenCV windows
        cap.release()
        cv2.destroyAllWindows()


# Example usage:
if __name__ == "__main__":
    # Create an instance of the Camera class
    camera = Camera()

    # Scan for ArUco markers
    rvec, tvec = camera.scan_ugv()

    # Print the results
    print("rvec:", rvec)
    print("tvec:", tvec)
    print("Detected ID:", camera.id)