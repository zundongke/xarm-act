  
# 保存数据函数
def save_data(args, timesteps, actions, dataset_path):
    # 数据字典
    data_size = len(actions)
    data_dict = {
        # 一个是奖励里面的qpos，qvel， effort ,一个是实际发的acition
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
        '/base_action': [],
        # '/base_action_t265': [],
    }

    # 相机字典  观察的图像
    for cam_name in args.camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []
        if args.use_depth_image:
            data_dict[f'/observations/images_depth/{cam_name}'] = []

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    # 动作长度 遍历动作
    while actions:
        # 循环弹出一个队列
        action = actions.pop(0)   # 动作  当前动作
        ts = timesteps.pop(0)     # 奖励  前一帧

        # 往字典里面添值
        # Timestep返回的qpos，qvel,effort
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])

        # 实际发的action
        data_dict['/action'].append(action)
        data_dict['/base_action'].append(ts.observation['base_vel'])

        # 相机数据
        # data_dict['/base_action_t265'].append(ts.observation['base_vel_t265'])
        for cam_name in args.camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
            if args.use_depth_image:
                data_dict[f'/observations/images_depth/{cam_name}'].append(ts.observation['images_depth'][cam_name])

    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        # 文本的属性：
        # 1 是否仿真
        # 2 图像是否压缩
        #
        root.attrs['sim'] = False
        root.attrs['compress'] = False

        # 创建一个新的组observations，观测状态组
        # 图像组
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in args.camera_names:
            _ = image.create_dataset(cam_name, (data_size, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3), )
        if args.use_depth_image:
            image_depth = obs.create_group('images_depth')
            for cam_name in args.camera_names:
                _ = image_depth.create_dataset(cam_name, (data_size, 480, 640), dtype='uint16',
                                             chunks=(1, 480, 640), )

        _ = obs.create_dataset('qpos', (data_size, 14))
        _ = obs.create_dataset('qvel', (data_size, 14))
        _ = obs.create_dataset('effort', (data_size, 14))
        _ = root.create_dataset('action', (data_size, 14))
        _ = root.create_dataset('base_action', (data_size, 2))

        # data_dict write into h5py.File
        for name, array in data_dict.items():  
            root[name][...] = array
    print(f'\033[32m\nSaving: {time.time() - t0:.1f} secs. %s \033[0m\n'%dataset_path)
def process(self):
    timesteps = []
    actions = []
    max_timesteps=500
    use_depth_image=True
        # 图像数据
    image = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8)
    image_dict = dict()
    image_dict['forward_picture'] = image
    count = 0
        
        # input_key = input("please input s:")
        # while input_key != 's' and not rospy.is_shutdown():
        #     input_key = input("please input s:")

    rate = 50
    print_flag = True

    while (count < max_timesteps + 1):
            # 2 收集数据
        result = self.get_frame()
        if not result:
            if print_flag:
                print("syn fail")
                print_flag = False
            rate.sleep()
            continue
        print_flag = True
        count += 1
        (img_front, img_left, img_right, img_front_depth, img_left_depth, img_right_depth,
        puppet_arm, puppet_arm, master_arm, master_arm, robot_base) = result
            # 2.1 图像信息
        image_dict = dict()
        image_dict['forward_picture'] = img_front
            #image_dict[self.args.camera_names[1]] = img_left
            #image_dict[self.args.camera_names[2]] = img_right

            # 2.2 从臂的信息从臂的状态 机械臂示教模式时 会自动订阅
        obs = collections.OrderedDict()  # 有序的字典
        obs['images'] = image_dict
        if use_depth_image:
            image_dict_depth = dict()
            image_dict_depth['forwar_picture'] = img_front_depth
                #image_dict_depth[self.args.camera_names[1]] = img_left_depth
               # image_dict_depth[self.args.camera_names[2]] = img_right_depth
            obs['images_depth'] = image_dict_depth
        _, position =self.arm.get_position()
        obs['qpos'] = position
        obs['qvel'] =self.arm.get_suction_cup() 
        obs['effort'] = self.arm.get_joint_torque()
            #if self.args.use_robot_base:
               # obs['base_vel'] = [robot_base.twist.twist.linear.x, robot_base.twist.twist.angular.z]
            #else:
              #  obs['base_vel'] = [0.0, 0.0]

            # 第一帧 只包含first， fisrt只保存StepType.FIRST
        if count == 1:
            ts = dm_env.TimeStep(
                step_type=dm_env.StepType.FIRST,
                reward=None,
                discount=None,
                observation=obs)
            timesteps.append(ts)
            continue

            # 时间步
        ts = dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
                reward=None,
                discount=None,
                observation=obs)

            # 主臂保存状态
        action = np.concatenate((np.array(master_arm_left.position), np.array(master_arm_right.position)), axis=0)
        actions.append(action)
        timesteps.append(ts)
        print("Frame data: ", count)
        rate.sleep()

    print("len(timesteps): ", len(timesteps))
    print("len(actions)  : ", len(actions))
    return timesteps, actions
def main():
    
    timesteps, actions = process()
    dataset_dir = os.path.join('./', 'xarm_task')
    
    if(len(actions) < 500):
        print("\033[31m\nSave failure, please record %s timesteps of data.\033[0m\n" %args.max_timesteps)
        exit(-1)

    if not os.path.exists(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, "episode_" + str(args.episode_idx))
    save_data(args, timesteps, actions, dataset_path)
