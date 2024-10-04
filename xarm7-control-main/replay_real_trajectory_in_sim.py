import imageio
import numpy as np
from sapien.utils import Viewer
from tqdm.auto import tqdm

from sim.envs.xarm7 import xArm7Env


def replay(kinematic_only=False, enable_viewer=True, video_path="video.mp4"):
    env = xArm7Env()
    env.reset()
    env.step()

    if enable_viewer:
        viewer = Viewer(env._renderer)
        viewer.set_scene(env.scene)

    real_traj = np.load("data/real_trajectory.npy", allow_pickle=True)

    sim_t = real_traj[0]["timestamp"]
    frames = []
    for step in tqdm(real_traj):
        real_t = step["timestamp"]
        # In degrees
        sensed_qpos = step["qpos"]
        sensed_qvel = step["qvel"]
        target_qpos = step["action"]

        if kinematic_only:
            env.robot.set_qpos(np.deg2rad(sensed_qpos))
            env.robot.set_drive_target(np.deg2rad(sensed_qpos))
        else:
            env.robot.set_servo_angle(target_qpos, 50, is_radian=False)

        env.scene.update_render()
        if enable_viewer:
            viewer.render()

        if video_path:
            env.camera.take_picture()
            rgba = env.camera.get_picture("Color")
            rgb = np.clip(rgba[..., :3] * 255, 0, 255).astype(np.uint8)
            frames.append(rgb)

        while sim_t <= real_t:
            env.step()
            sim_t += 1 / env._control_freq

    if video_path:
        imageio.mimwrite(video_path, frames, fps=60)


if __name__ == "__main__":
    replay(kinematic_only=True, enable_viewer=False, video_path="replay_real_trajectory_in_sim_kinematic.mp4")
    replay(kinematic_only=False, enable_viewer=False, video_path="replay_real_trajectory_in_sim_dynamic.mp4")
    
    # from moviepy.editor import VideoFileClip, clips_array
    # clip1 = VideoFileClip("replay_real_trajectory_in_sim_kinematic.mp4")
    # clip2 = VideoFileClip("replay_real_trajectory_in_sim_dynamic.mp4")
    # combined = clips_array([[clip1, clip2]])
    # combined.write_videofile("replay_real_trajectory_in_sim_side_by_side.mp4")
