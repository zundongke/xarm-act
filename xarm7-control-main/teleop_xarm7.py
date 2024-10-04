import os
import time

import numpy as np
from scipy.spatial.transform import Rotation
from xarm.wrapper import XArmAPI

from devices.spacemouse import SpaceMouseThread

ip = os.getenv("XARM_IP", "192.168.1.203")

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(state=0)

arm.set_gripper_mode(0)
arm.set_gripper_enable(True)
arm.set_gripper_speed(2000)
arm.set_gripper_position(600, wait=True)

# print("Initial joint position: ", arm.get_initial_point())
# NOTE(jigu): Seem to always return to zero position
# arm.reset(wait=True)

INITIAL_ARM_JOINT_POS = [0, 0, 0, 60, 0, 60, -90]
arm.set_servo_angle(angle=INITIAL_ARM_JOINT_POS, wait=True)
# arm.set_position(x=464, y=0, z=355, roll=180, pitch=-45, yaw=90, wait=True)
arm.set_mode(7)
arm.set_state(0)
time.sleep(0.1)

device = SpaceMouseThread()
scale_t = 1  # mm
scale_r = 1  # deg
freq = 30

code, (x, y, z, roll, pitch, yaw) = arm.get_position(is_radian=False)
print("Current position: ", x, y, z, roll, pitch, yaw)

last_left_button = False
is_closed = False
gripper_pos = 600

while True:
    # for i in range(100):
    if device.get_right_button():
        break

    R = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)

    # Read from teleop device
    d_xyz = device.get_xyz()
    print("d_xyz", d_xyz)
    d_rpy = device.get_rpy()
    # d_rpy = np.array([0, 0, 1])
    print("d_rpy", d_rpy)

    x2 = x + d_xyz[0] * scale_t
    y2 = y + d_xyz[1] * scale_t
    z2 = z + d_xyz[2] * scale_t

    # NOTE(jigu): I have not figured out what rpy we actually get from the spacemouse.
    d_R = Rotation.from_euler("xyz", d_rpy * scale_r, degrees=True)
    # print(d_R.as_rotvec())
    # Rotation is represented in the tool frame
    R2 = R * d_R
    roll2, pitch2, yaw2 = R2.as_euler("xyz", degrees=True)
    print("rpy2", roll2, pitch2, yaw2)

    # NOTE(jigu): `is_relative`` does not work in mode 7.
    # roll, pitch, yaw here is in the base frame
    arm.set_position(
        x=x2, y=y2, z=z2, roll=roll2, pitch=pitch2, yaw=yaw2, is_radian=False
    )

    current_left_button = device.get_left_button()
    if current_left_button and not last_left_button:
        if is_closed:
            gripper_pos = 0
        else:
            gripper_pos = 600
        is_closed = not is_closed
    arm.set_gripper_position(gripper_pos)
    last_left_button = current_left_button

    time.sleep(1 / freq)

    # Update target pose
    x, y, z = x2, y2, z2
    roll, pitch, yaw = roll2, pitch2, yaw2

arm.disconnect()
device.close()
