import os
import time

import numpy as np
from xarm.wrapper import XArmAPI

ip = os.getenv("XARM_IP", "192.168.1.212")

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(state=0)

INITIAL_ARM_JOINT_POS = [0, 0, 0, 60, 0, 60, -90]
arm.set_servo_angle(angle=INITIAL_ARM_JOINT_POS, wait=True)

# NOTE(jigu): I use the manual mode to get those waypoints.
WAYPOINTS = [
    [0, 0, 0, 60, 0, 60, -90],
    [2.2, 0, 18.7, 60.3, 0, 60, -90],
    [2.3, -7.5, 37.4, 64.1, -32.1, 60, -90],
    [2.1, -3.4, -6.2, 64.1, -3.5, 25.5, -90],
    [2, -3.1, -50.7, 60.9, -1.6, 88.7, -60.3],
    [2.9, -0.9, -3, 19.7, -1.5, -45.8, -1],
    # [2.8, -50.1, -4, 84, -1.6, 45.4, -1],
    # [-146.9, -20, -26.5, 83.4, -1.6, 75.9, 53.2],
    [-114.4, -4.5, 30.6, 29.9, 1.4, -37.8, 50.1],
    [-97.3, -8.7, 87.5, 68, 3.9, 63.9, 104.2],
]

arm.set_mode(6)
arm.set_state(0)
time.sleep(0.1)

control_freq = 60
trajectory = []
start_time = time.time()
log_count=0
while True:
    # arm.set_servo_angle(angle=waypoint, wait=True)
    for i in range(control_freq * 3):
        log_count+=1
        timestamp = time.time() - start_time
        code, (qpos, qvel, qeff) = arm.get_joint_states()
        if log_count%30==0:
           print(timestamp, qpos)
        # arm.set_servo_angle(angle=waypoint, speed=30, wait=False)
        # trajectory.append(
        #     {
        #         "timestamp": timestamp,
        #         "qpos": qpos,
        #         "qvel": qvel,
        #         "qeff": qeff,
        #         "action": waypoint,
        #     }
        # )
        time.sleep(1 / control_freq)

# # Save trajectory
# np.save("real_trajectory.npy", trajectory)

arm.disconnect()
