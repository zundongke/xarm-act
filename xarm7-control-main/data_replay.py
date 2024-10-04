import os
from xarm.wrapper import XArmAPI
import keyboard
import time
import collections
import numpy as np
import cv2
import threading
import time
import pyrealsense2 as rs
import pandas as pd

def xarm_setup(arm):
   arm.motion_enable(enable=True)
   arm.clean_error()
   arm.set_mode(0)
   arm.set_state(state=0)

# 设置机械臂的运动参数
   arm.set_gripper_mode(0)
   arm.set_gripper_enable(True) 
   arm.set_gripper_speed(2000)
def real_sense_setup():
    pipeline = rs.pipeline()
    config = rs.config()
    #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 确保深度流已启用
    config.enable_device_from_file('picture-1.bag')#确保可以读取数据
    pipeline.start(config)
    return pipeline
    
def data_replay():
    ip = os.getenv("XARM_IP", "192.168.1.212")
    arm = XArmAPI(ip)
    xarm_setup(arm)
    pipeline=real_sense_setup()
    xarm_data=pd.read_csv('final_state.csv')
    start_time=time.time()
    try:
        for index, row in xarm_data.iterrows():
        # Calculate elapsed time
        #     elapsed_time = time.time() - start_time
        
        # # Wait until the correct time to play the next frame
        #     while elapsed_time < row['timestamp']:
        #         elapsed_time = time.time() - start_time

        # Play xArm data
            joint_positions = [row[f'joint_{i+1}'] for i in range(6)]
            arm.set_servo_angle(angle=joint_positions, wait=False)
            joint_positions = [row[f'joint_{i+1}'] for i in range(6)]
            arm.set_servo_angle(angle=joint_positions, wait=False)

        # Get Realsense frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
        # 等待一个frame
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()


        # 获取深度和彩色图像
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

        # 使用OpenCV显示
            cv2.imshow('Color Frame', color_image)
            cv2.imshow('Depth Frame', depth_image)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()