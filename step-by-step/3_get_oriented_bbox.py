import pyrealsense2 as rs
import numpy as np
import time

import sys
try:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
finally:
    import cv2
from detect import detect, draw_bbox
from ..utils import *


CALIB_CAM2ARM = np.array(
    [[ 0.01606431, -0.99982658,  0.00942071,  0.06578977],
     [ 0.99986186,  0.01602326, -0.00441669, -0.0344229 ],
     [ 0.00426497,  0.00949036,  0.99994587,  0.02654628],
     [ 0.,          0.,          0.,          1.        ]]
)
gripper_offset = np.array([ 0.0, 0.0, 172 ]) # meters

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
align = rs.align(rs.stream.color)
color_intrinsics = profile.get_stream(
        rs.stream.color).as_video_stream_profile().get_intrinsics()
intrinsics = [color_intrinsics.ppx, color_intrinsics.ppy, color_intrinsics.fx, color_intrinsics.fy]

try:
    while True:

        frames = pipeline.wait_for_frames()

        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale
        color_image = np.asanyarray(color_frame.get_data())

        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        bboxes = detect(color_image)
        if bboxes is not None:
            detected_color_image = draw_bbox(color_image, bboxes)
            detected_depth_image = draw_bbox(colorized_depth, bboxes)
        else:
            detected_color_image = color_image.copy() #[:, :, ::-1]
            detected_depth_image = colorized_depth.copy()
        
        images = np.hstack((detected_color_image, detected_depth_image))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        if bboxes is not None:
            x, y, w, h = bboxes[0]
            p_leftop = get_camera_xyz(depth_image, x, y, intrinsics)
            p_rightbottom = get_camera_xyz(depth_image, x + w, y + h, intrinsics)
            bbox_diag = compute_distance(p_leftop, p_rightbottom)
            print("diagonal of detected bbox:", bbox_diag)
            p_center = get_camera_xyz(depth_image, x + w//2, y + h//2, intrinsics)

            print("image:", p_center)
            center, max_bound, min_bound = get_oriented_bbox(depth_image[y:y+h, x:x+w], color_intrinsics)
            print("center:", center)
            print("max bound:", max_bound)
            print("min_bound:", min_bound)
            print("------------------------------------------")

        # p_end = CALIB_CAM2ARM @ np.concatenate([p_center, np.array([1.])])
        # p_end[:3] *= 1000
        # print("end effector: x, y, z =", p_end)
        # p_gripper = p_end[:3] - gripper_offset

        
        

finally:
    pipeline.stop()