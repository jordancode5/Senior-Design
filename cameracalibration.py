import numpy as np
import cv2 as cv

def calibrate_camera(calibration_images, chessboard_size):
    objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)

    objpoints = [] # 3D points in real world space
    imgpoints = [] # 2D points in image plane

    for img in calibration_images:
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

    ret, cam_matrix, dist_coef, r_vectors, t_vectors = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return cam_matrix, dist_coef, r_vectors, t_vectors

def main():
    cap = cv.VideoCapture(0)  # Initialize camera capture
    calibration_images = []  # List to store calibration images
    chessboard_size = (9, 6)  # Size of the chessboard pattern used in calibration
    image_count = 0

    while True:
        ret, frame = cap.read()  # Capture frame from camera
        cv.imshow('Calibration', frame)

        key = cv.waitKey(1)
        if key == ord('c'):  # Press 'c' to capture an image
            calibration_images.append(frame.copy())
            image_count += 1
            print(f'Image {image_count} captured.')
            if image_count >= 10:  # Adjust the number of images you want to capture
                break

    cap.release()
    cv.destroyAllWindows()

    if len(calibration_images) > 0:
        print("Calibrating camera...")
        cam_matrix, dist_coef, r_vectors, t_vectors = calibrate_camera(calibration_images, chessboard_size)

        # Save calibration data to a .npz file
        np.savez('../calib_data/MultiMatrix.npz', camMatrix=cam_matrix, distCoef=dist_coef, rVector=r_vectors, tVector=t_vectors)
        print("Calibration data saved to ../calib_data/MultiMatrix.npz")
    else:
        print("No calibration images captured. Exiting.")

if __name__ == "__main__":
    main()