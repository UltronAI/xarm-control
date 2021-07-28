import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
print(sys.path) # TODO: why to append this path?

from xarm.wrapper import XArmAPI 

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

time.sleep(1)

print('=' * 50)
print('default_is_radian:', arm.default_is_radian)
print('version:', arm.version)
print('state:', arm.state)
print('mode:', arm.mode)
print('cmdnum:', arm.cmd_num)
print('error_code:', arm.error_code)
print('warn_code:', arm.warn_code)
print('collision_sensitivity:', arm.collision_sensitivity) # sensitivity value of collision
print('teach_sensitivity:', arm.teach_sensitivity) # sensitivity value of drag and teach
print('world_offset:', arm.world_offset) # base coordinate offset, [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)
print('gravity_direction:', arm.gravity_direction)

print('============TCP============') # NOTE: this TCP is Tool Center Point, i.e. the end of the arm (or the gripper or the camera)
print('* position:', arm.position) # [x, y, z, roll, pitch, yaw]
print('* tcp_jerk:', arm.tcp_jerk)
print('* tcp_load:', arm.tcp_load) # weight and center of gravity, [weight(kg), [x(mm), y(mm), z(mm)]]
# NOTE: TCP offset may be important when using image as input
print('* tcp_offset:', arm.tcp_offset) # Cartesion position offset, [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)]
print('* tcp_speed_limit:', arm.tcp_speed_limit) # [min_tcp_speed(mm/s), max_tcp_speed(mm/s)]
print('* tcp_acc_limit:', arm.tcp_acc_limit) # [min_tcp_acc(mm/s^2), max_tcp_acc(mm/s^2)]

print('===========JOINT===========') # for xarm7, there are 7 joints in total
print('* angles:', arm.angles) # [angle1(° or rad), angle2(° or rad), ..., anglen7(° or rad)]
print('* joint_jerk:', arm.joint_jerk)
print('* joint_speed_limit:', arm.joint_speed_limit)
print('* joint_acc_limit:', arm.joint_acc_limit)
print('* joints_torque:', arm.joints_torque)

print('===========API_GET=========')
print('version:', arm.get_version())
print('state:', arm.get_state())
print('cmdnum:', arm.get_cmdnum())
print('err_warn_code:', arm.get_err_warn_code())
print('position(°):', arm.get_position(is_radian=False))
print('position(radian):', arm.get_position(is_radian=True))
print('angles(°):', arm.get_servo_angle(is_radian=False))
print('angles(radian):', arm.get_servo_angle(is_radian=True))
print('angles(°)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=False)) # index of servo start with 1
print('angles(radian)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=True))


arm.disconnect()
