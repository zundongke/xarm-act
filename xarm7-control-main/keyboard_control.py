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
import numpy as np
import cv2
import csv
import pandas as pd
import glob

def xarm_setup(arm):
    arm.motion_enable(enable=True)
    arm.clean_error()
    arm.set_mode(0)
    arm.set_state(state=0)

# 设置机械臂的运动参数
    arm.set_gripper_mode(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_speed(2000)
# 设置机械臂的运动速度和加速度
#arm.set_joint_position_speed(90)  # 设置关节运动速度，单位：度/秒
#arm.set_joint_position_acceleration(60)  # 设置关节运动加速度，单位：度/秒²

#INITIAL_ARM_JOINT_POS = [0, 0, 0, 60, 0, 60, -90]
#arm.set_servo_angle(angle=INITIAL_ARM_JOINT_POS, wait=True)
def move_robot_arm(direction,arm):
    if direction == 'up':
        code,position =arm.get_position()
        arm.set_position(position[0], position[1], position[2]+16, position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'down':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1], position[2]-16, position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'left':
        code, position=arm.get_position()
        arm.set_position(position[0]-16, position[1], position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'right':
        code,position=arm.get_position()
        arm.set_position(position[0]+16 
                         , position[1], position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'forward':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1]+16, position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'backward':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1]-16, position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction =='grasp':
        arm.set_gripper_position(850, wait=True)
        #time.sleep(1/freq)
    elif direction=='loose':
        arm.set_gripper_position(200, wait=True)
        #time.sleep(1/freq)
    elif direction=='back':
        arm.set_position(200,-1.2,205,175,5,5.3)
        #time.sleep(1/freq)
        #time.sleep(1/freq)

def change_command(direction,arm):
    if direction == 'up':
        code,position =arm.get_position()
        arm.set_position(position[0], position[1], position[2]+16, position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'down':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1], position[2]-16, position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'left':
        code, position=arm.get_position()
        arm.set_position(position[0]-10, position[1], position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'right':
        code,position=arm.get_position()
        arm.set_position(position[0]+10 
                         , position[1], position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'forward':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1]+20, position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction == 'backward':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1]-20, position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
        #time.sleep(1/freq)
    elif direction =='grasp':
        arm.set_gripper_position(850, wait=True)
        #time.sleep(1/freq)
    elif direction=='loose':
        arm.set_gripper_position(200, wait=True)
        #time.sleep(1/freq)
    elif direction=='back':
        arm.set_position(200,-1.2,205,175,5,5.3)

# timestamp = time.time()  # 当前时间戳，单位秒
# agent_timestamp = time.time()

# 设置相机和智能体

output_dir = "data_sync-2"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

def concentrate_csv(temp_file,count):
    if count==1:
        final_file=temp_file
        merged_df=pd.read_excel(final_file)
        merged_df.to_csv('final_file.csv', index=False)
    else:
        data_frame=[]
        df=pd.read_csv(temp_file)
        df2=pd.read_csv('final_file.csv')
        data_frame.append(df)
        data_frame.append(df2)
        merged_df=pd.concat(data_frame, ignore_index=True)
        merged_df.to_csv('final_file.csv',index=False)
# 定义文件保存格式
def save_data(timestamp, color_image, depth_image, arm_state):
    # 保存图像数据
    #color_image_path = os.path.join(output_dir, f'color_{timestamp}.png')
    #depth_image_path = os.path.join(output_dir, f'depth_{timestamp}.png')
    
    # 保存彩色图像
    #cv2.imwrite(color_image_path, color_image)
    
    # 保存深度图像（保存为 PNG 图像的深度数据）
    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    #cv2.imwrite(depth_image_path, depth_colormap)
    
    # 保存机械臂状态
    # state_file = os.path.join(output_dir, f'state_{timestamp}.txt')
    # with open(state_file, 'w') as f:
    #     f.write(str(arm_state))
    csv_file = 'temp.csv'

# 获取字典的键作为 CSV 文件的列名
    fieldnames = arm_state[0].keys()
    count=0

# 打开文件，准备写入
    with open(csv_file, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)

    # 写入列名
        writer.writeheader()
        count=count+1

    # 写入每一行字典数据
        for row in arm_state:
            writer.writerow(row)
        concentrate_csv(csv_file,count)
def record_state(arm_state):
    csv_file = 'temp.csv'

# 获取字典的键作为 CSV 文件的列名
    fieldnames = arm_state[0].keys()
    count=0

# 打开文件，准备写入
    with open(csv_file, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)

    # 写入列名
        writer.writeheader()
        count=count+1

    # 写入每一行字典数据
        for row in arm_state:
            writer.writerow(row)
        concentrate_csv(csv_file,count)

def sync_data(image_data, agent_data):
    synced_data = []
    agent_idx = 0
    
    for image_entry in image_data:
        image_time = image_entry['timestamp']
        # 找到与图像时间戳最接近的智能体状态
        while agent_idx < len(agent_data) - 1 and abs(agent_data[agent_idx]['timestamp'] - image_time) > abs(agent_data[agent_idx + 1]['timestamp'] - image_time):
            agent_idx += 1

        # 同步后的数据
        synced_data.append({
            'timestamp': image_time,
            'image': image_entry['image'],
            'agent_state': agent_data[agent_idx]['agent_state']
        })
    
    return synced_data

# 示例智能体接口（例如位置、速度等）
def get_xarm_state(arm):
    # 获取关节角度 (单位: 度)
    joint_angles = arm.angles
    
    # 获取关节速度 (单位: 度/秒)
    joint_speeds = arm.get_joint_states
    print(f'joint_speeds:{joint_speeds}')
    
    # 获取关节力矩 (单位: Nm)
    joint_torques = arm.joints_torque
    
    return {
        'angles': joint_angles,
        'speeds': joint_speeds,
        'torques': joint_torques
    }

# 同步采集数据的循环
def collect_data(arm):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 确保深度流已启用
    config.enable_record_to_file("picture-1.bag")#将图片数据保存到picture-1.bag里面
    pipeline.start(config)
    data = []
    freq=30
    try:
        while True:
            # 获取当前时间戳aq
            current_time = time.time()

        # 获取一帧图像
            frames = pipeline.wait_for_frames()#存取picture
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            timestamp=color_frame.get_timestamp()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            print(color_image)

        # 获取智能体的状态
            agent_state = get_xarm_state(arm)

        # 保存图片数据和智能体数据，附加时间戳
            #save_data(current_time, color_image, depth_image, agent_state)
            #
            record_state(agent_state)#存取state
            time.sleep(1/freq)#按一定的频率存取
        # 实时显示图片  
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            #cv2.imshow('RealSense Image', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
    # 停止数据流和关闭窗口
        pipeline.stop()
        cv2.destroyAllWindows()
        
# def command_arm():
#     global control_buffer
#     global arm
#     dt=0.05
#     while True:
#         with control_lock:
#             arm.set_position(control_buffer[0], control_buffer[1], control_buffer[2]+10, control_buffer[3], control_buffer[4], control_buffer[5], wait=True, radius=-1.0)     
#             time.sleep(dt)
# def move_robot_arm(direction,arm):
#     global control_buffer
#     if direction == 'up':
#         code,position =arm.get_position()
#         control_buffer=[position[0], position[1], position[2]+10, position[3], position[4], position[5]]
    # arm.set_position(position[0], position[1], position[2]+10, position[3], position[4], position[5], wait=True, radius=-1.0)
    #time.sleep(1/freq)
def key_board_cotrol(arm): 
    keyboard.add_hotkey('up', lambda : move_robot_arm("up",arm))
    keyboard.add_hotkey('down', lambda: move_robot_arm("down",arm))
    keyboard.add_hotkey('left', lambda: move_robot_arm("left",arm))
    keyboard.add_hotkey('right', lambda: move_robot_arm("right",arm))
    keyboard.add_hotkey('space', lambda: move_robot_arm("loose",arm))
    keyboard.add_hotkey('enter', lambda: move_robot_arm("grasp",arm))
    keyboard.add_hotkey('z', lambda: move_robot_arm('backward',arm))
    keyboard.add_hotkey('a', lambda: move_robot_arm('forward',arm))
    keyboard.add_hotkey('tab', lambda: move_robot_arm('back',arm))
    # keyboard.wait('esc')
    #time.sleep(1/freq)
     

if __name__ == '__main__':
    '''
    ip = os.getenv("XARM_IP", "192.168.1.212")
    arm = XArmAPI(ip)
    freq=50
    xarm_setup(arm)  
    keyboard.add_hotkey('up', lambda: move_robot_arm("up"))
    keyboard.add_hotkey('down', lambda: move_robot_arm("down"))
    keyboard.add_hotkey('left', lambda: move_robot_arm("left"))
    keyboard.add_hotkey('right', lambda: move_robot_arm("right"))
    keyboard.add_hotkey('space', lambda: move_robot_arm("loose"))
    keyboard.add_hotkey('enter', lambda: move_robot_arm("grasp"))
    keyboard.add_hotkey('q', lambda: move_robot_arm('backward'))
    keyboard.add_hotkey('a', lambda: move_robot_arm('forward'))
    keyboard.add_hotkey('tab', lambda: move_robot_arm('back'))
    keyboard.wait('esc')
    #time.sleep(1/freq)
    
     
    arm.disconnect()  
    '''
    ip = os.getenv("XARM_IP", "192.168.1.212")
    arm = XArmAPI(ip)
    freq=50
    xarm_setup(arm) 
    key_board_cotrol(arm)
    
    collect_data(arm)
    #arm.disconnect() 
    # t1 = threading.Thread(target=key_board_cotrol(arm))
    # t2 = threading.Thread(target=collect_data(arm))
    # t1.start()
    # t2.start()
    # t1.join()
    # t2.join()
    
