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

time.sleep(10)

print(arm.connected)

arm.disconnect()
