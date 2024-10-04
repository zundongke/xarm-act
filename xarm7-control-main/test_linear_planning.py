import os
from xarm.wrapper import XArmAPI

ip = os.getenv("XARM_IP", "192.168.1.212")

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(state=0)

INITIAL_ARM_JOINT_POS = [0, 0, 0, 60, 0, 60, -70]
arm.set_servo_angle(angle=INITIAL_ARM_JOINT_POS, wait=True)

# arm.set_position(401.8, 234.3, 184.7, 180, 30, 120, wait=True, radius=-1.0)
# arm.set_position(401.8, -234.3, 184.7, 180, -30, 60, wait=True, radius=-1.0)

arm.disconnect()
