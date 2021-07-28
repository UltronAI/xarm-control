
import pyrealsense2 as rs
import numpy as np
import time

import sys
try:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
finally:
    import cv2

from detect import detect, draw_bbox
from utils import get_camera_xyz, compute_distance, euler2matrix, get_transformation

from xarm.wrapper import XArmAPI 


############## XArm setup ##############
def hangle_err_warn_changed(item):
    print("ErrorCode: {}, WarnCode: {}".format(item['error_code'], item['warn_code']))

XARM7_IP = "192.168.1.226" # ip address to our xarm7

arm = XArmAPI(XARM7_IP)
arm.register_error_warn_changed_callback(hangle_err_warn_changed)
arm.connect()

# enable motion
arm.motion_enable(True)
arm.set_mode(0) # mode 0: position control mode, see details in API document
arm.set_state(state=0) # mode 0: sport state

arm.set_tcp_offset([0,0,172,0,0,0])
arm.set_tcp_load(0.82,[0,0,48])

init_pos = arm.get_position()[1]
##########################################


############# RealSense setup #############
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
##########################################


############# Parameters setup #############
CALIB_CAM2ARM = np.array(
    [[ 0.01606431, -0.99982658,  0.00942071,  0.06578977],
     [ 0.99986186,  0.01602326, -0.00441669, -0.0344229 ],
     [ 0.00426497,  0.00949036,  0.99994587,  0.02654628],
     [ 0.,          0.,          0.,          1.        ]]
)
# CALIB_CAM2ARM = np.array(
#     [[ 0.01681656, -0.99985404,  0.00301729,  0.06713764],
#      [ 0.99984396,  0.01683256,  0.00535895, -0.03584064],
#      [-0.00540896,  0.0029267 ,  0.99998109,  0.03207806],
#      [ 0.,          0.,          0.,          1.        ]]
# )
p_camera = np.array([ 0.03324806,  0.06607799, -0.02731666 ]) # optical
gripper_offset = np.array([ 10, -15, 172 ])
##########################################


try: 
    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            continue

        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # show image stream and depth stream
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
            print("w / h:", w / h)
            if bbox_diag < 0.07 or w / h < 0.5 or w / h > 2:
                continue
            p_camera = get_camera_xyz(depth_image, x + w//2, y + h//2, intrinsics)
            print("cube to camera:", p_camera * 1000)

            p_end = CALIB_CAM2ARM @ np.concatenate([p_camera, np.array([1.])])
            p_end[:3] *= 1000
            print("cube to end: x, y, z =", p_end[:3])
            p_gripper = p_end[:3] - gripper_offset

            pos_end2base = arm.get_position()[1]
            print("end to base:", pos_end2base)
            T_end2base = get_transformation(pos_end2base)
            p_base = T_end2base @ np.concatenate([p_gripper, np.array([1.])])
            p_base[2] = p_base[2] / 3
            print("cube to base: x, y, z =", p_base[:3])

            arm.set_mode(0) # mode 0: position control mode, see details in API document
            arm.set_state(state=0) # mode 0: sport state

            if np.sqrt(np.sum(p_gripper[:2] ** 2)) > 8 or np.abs(w / h - 1) > 0.2:
                print("distance in xy plane:", np.sqrt(np.sum(p_gripper[:2] ** 2)))
                print("target pos:", *p_base[:2], *pos_end2base[2:])
                arm.set_position(*p_base[:2], *pos_end2base[2:], wait=True)
                new_pos = arm.get_position()[1]
                print("new pos:", new_pos)
                print("------------------")
            else:
                print("!!! ready for grasping")

                code = arm.set_gripper_mode(0)
                print('>> set gripper mode: location mode, code={}'.format(code))
                code = arm.set_gripper_enable(True)
                print('>> set gripper enable, code={}'.format(code))
                code = arm.set_gripper_speed(5000)
                print('>> set gripper speed, code={}'.format(code))
                code = arm.set_gripper_position(800, wait=True, speed=6000)
                print('>> set gripper pos, code={}'.format(code))

                arm.set_position(*pos_end2base[:2], p_base[2], *pos_end2base[3:], wait=True)
                new_pos = arm.get_position()[1]
                print(new_pos)
                time.sleep(0.2)

                code = arm.set_gripper_position(300, wait=True, speed=6000)
                print('set gripper pos, code={}'.format(code))
                time.sleep(0.2)

                print("!!! successfully grasp the cube")
                # rel_x = float(input("move x (in millimeters):"))
                # rel_y = float(input("move y (in millimeters):"))
                rel_x = -100
                rel_y = 100

                new_target_1 = np.array(new_pos) + np.array([0, 0, 200, 0, 0, 0])
                arm.set_position(*new_target_1, wait=True)
                print("1:", arm.get_position())
                time.sleep(0.5)

                new_target_2 = np.array(new_pos) + np.array([rel_x, 0, 200, 0, 0, 0])
                arm.set_position(*new_target_2, wait=True)
                print("2:", arm.get_position())
                time.sleep(0.5)

                new_target_3 = np.array(new_pos) + np.array([rel_x, rel_y, 200, 0, 0, 0])
                arm.set_position(*new_target_3, wait=True)
                print("3:", arm.get_position())
                time.sleep(0.5)

                new_target_4 = new_target_3.copy()
                new_target_4[2] = p_base[2]
                arm.set_position(*new_target_4, wait=True)
                print("4:", arm.get_position())

                code = arm.set_gripper_position(800, wait=True, speed=6000)
                print('set gripper pos, code={}'.format(code))

                arm.set_position(*new_target_3)
                arm.set_position(*init_pos)
                print("END.")
                break

finally:
    pipeline.stop()
    arm.disconnect()
