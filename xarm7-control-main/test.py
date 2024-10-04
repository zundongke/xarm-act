import keyboard

ip = os.getenv("XARM_IP", "192.168.1.212")

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(state=0)

# 设置机械臂的运动参数
arm.set_gripper_mode(1)  # 设置夹爪模式为1（默认模式）
arm.set_gripper_enable(True)  # 启用夹爪
arm.set_gripper_speed(2000)
# 设置机械臂的运动速度和加速度
#arm.set_joint_position_speed(90)  # 设置关节运动速度，单位：度/秒
#arm.set_joint_position_acceleration(60)  # 设置关节运动加速度，单位：度/秒²


#INITIAL_ARM_JOINT_POS = [0, 0, 0, 60, 0, 60, -90]
#arm.set_servo_angle(angle=INITIAL_ARM_JOINT_POS, wait=True)
def move_robot_arm(direction):
    if direction == 'up':
        code,position =arm.get_position()
        arm.set_position(180.6, -3.6, 199.5, 177.8, -0.5, 6.7, wait=True, radius=-1.0)
    elif direction == 'down':
        code,position=arm.get_position()
        arm.set_position(531.3, -27.3, 237.7, 177.8, -0.5, 6.7, wait=True, radius=-1.0)
    elif direction == 'left':
        code, position=arm.get_position()
        arm.set_position(511.6, -147.8, 224.8, 177.8, -0.5, 6.7, wait=True, radius=-1.0)
    elif direction == 'right':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1], position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
    elif direction == 'stop':
        code,position=arm.get_position()
        arm.set_position(position[0], position[1], position[2], position[3], position[4], position[5], wait=True, radius=-1.0)
    elif direction =='grasp':
        arm.set_gripper_position(404)
    elif direction=='loose':
        arm.set_gripper_position(600)
# arm.set_position(401.8, 234.3, 184.7, 180, 30, 120, wait=True, radius=-1.0)
# arm.set_position(401.8, -234.3, 184.7, 180, -30, 60, wait=True, radius=-1.0)
# while True:
    keyboard.add_hotkey('up', lambda: move_robot_arm("up"))
    keyboard.add_hotkey('down', lambda: move_robot_arm("down"))
    keyboard.add_hotkey('left', lambda: move_robot_arm("left"))
    keyboard.add_hotkey('right', lambda: move_robot_arm("right"))
    keyboard.add_hotkey('enter', lambda: move_robot_arm("grasp"))
    keyboard.add_hotkey('space', lambda: move_robot_arm("loose"))
arm.disconnect()