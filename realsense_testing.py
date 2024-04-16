import datetime
import cv2 as cv
import pyrealsense2 as rs
import numpy as np

# use a realsense camera
pipe = rs.pipeline()
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

pipe.start(cfg)

while True:
    start = datetime.datetime.now()

    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()   #won't use to start
    color_frame = frame.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_cm = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha = 0.5),
                                cv.COLORMAP_JET)

    #cv.imshow('rgb', color_image)
    cv.imshow('depth', depth_cm)

    end = datetime.datetime.now()
    # show the time it took to process 1 frame
    total = (end - start).total_seconds()
    print(f"Time to process 1 frame: {total * 1000:.0f} milliseconds")

    # calculate the frame per second and draw it on the frame
    fps = f"FPS: {1 / (total + 0.00001):.2f}"
    cv.putText(color_image, fps, (50, 50),
               cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 8)
    cv.imshow('rgb', color_image)
    if cv.waitKey(1) == ord('q'):
        break

pipe.stop()
