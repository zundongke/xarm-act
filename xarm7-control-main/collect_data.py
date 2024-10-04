import time
import pyrealsense2 as rs
import numpy as np
import cv2
timestamp = time.time()  # 当前时间戳，单位秒
agent_timestamp = time.time()

# 设置相机和智能体



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
def get_xarm_state():
    # 获取关节角度 (单位: 度)
    joint_angles = arm.angles
    
    # 获取关节速度 (单位: 度/秒)
    joint_speeds = arm.joint_speeds
    
    # 获取关节力矩 (单位: Nm)
    joint_torques = arm.joint_torques
    
    return {
        'angles': joint_angles,
        'speeds': joint_speeds,
        'torques': joint_torques
    }

# 同步采集数据的循环
def collect_data():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    data = []
    try:
        while True:
            # 获取当前时间戳
            current_time = time.time()

        # 获取一帧图像
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

        # 获取智能体的状态
            agent_state = get_xarm_state()

        # 保存图片数据和智能体数据，附加时间戳
            data.append(sync_data(color_image,agent_state))
        

        # 实时显示图片
            cv2.imshow('RealSense Image', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
    # 停止数据流和关闭窗口
        pipeline.stop()
        cv2.destroyAllWindows()