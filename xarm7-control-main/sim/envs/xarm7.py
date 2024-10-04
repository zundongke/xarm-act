from __future__ import annotations

import numpy as np
import ruckig
import sapien


def parameterize_path(
    current_position: list[float],
    target_position: list[float],
    current_velocity: float | list[float],
    max_velocity: float | list[float],
    max_acceleration: float | list[float] = None,
    max_jerk: float | list[float] = None,
    delta_time: float = None,
):
    # https://github.com/pantor/ruckig/blob/c3dee09fd3db515acc63a5571764db6e99853507/examples/01_position.py
    dof = len(current_position)

    def as_vector(x):
        if isinstance(x, (int, float)):
            return np.full(dof, x)
        else:
            return x

    inp = ruckig.InputParameter(dof)
    inp.current_position = current_position
    inp.current_velocity = as_vector(current_velocity)
    inp.current_acceleration = np.zeros(dof)
    inp.target_position = target_position
    inp.target_velocity = np.zeros(dof)
    inp.target_acceleration = np.zeros(dof)

    inp.max_velocity = as_vector(max_velocity)
    if max_acceleration is not None:
        inp.max_acceleration = as_vector(max_acceleration)
    if max_jerk is not None:  # Otherwise infinity
        inp.max_jerk = as_vector(max_jerk)
    # The negative maximum velocity and acceleration are used for minimum.

    if delta_time is None:
        otg = ruckig.Ruckig(dof)
    else:
        inp.duration_discretization = ruckig.DurationDiscretization.Discrete
        otg = ruckig.Ruckig(dof, delta_time)
    trajectory = ruckig.Trajectory(dof)
    # Calculate the trajectory in an offline manner
    result = otg.calculate(inp, trajectory)
    if result == ruckig.Result.ErrorInvalidInput:
        return None
    return trajectory


class xArm7:
    URDF_PATH = "sim/assets/descriptions/xarm7_pris_finger.urdf"
    ARM_DEFAULT_INIT = [0, 0, 0, 60, 0, 60, -90]
    GRIPPER_DEFAULT_INIT = 840

    def __init__(self, scene: sapien.Scene, control_freq=250, is_radian=None):
        self.scene = scene
        self._control_freq = control_freq

        # Follow the xArm7 python SDK interface
        self.is_radian = is_radian

        # Constants
        self._max_speed = np.pi
        self._max_acc = self._max_speed * self._control_freq

        # Load robot
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.art_obj = loader.load(self.URDF_PATH)
        self.joints = self.art_obj.get_active_joints()
        assert len(self.joints) == 9, self.art_obj.dof

        # TODO: Tune PD parameters
        for joint in self.art_obj.get_active_joints():
            joint.set_drive_property(1000, 100)

    # ---------------------------------------------------------------------------- #
    # SAPIEN API wrapper. The unit is meter or radian.
    # ---------------------------------------------------------------------------- #
    def set_pose(self, pose: sapien.Pose):
        self.art_obj.set_pose(pose)

    def get_qpos(self):
        return self.art_obj.get_qpos()[:7]

    def set_qpos(self, qpos):
        gripper_qpos = self.art_obj.get_qpos()[7:]
        self.art_obj.set_qpos(np.hstack([qpos, gripper_qpos]))

    def set_drive_target(self, qpos):
        for joint, x in zip(self.joints[:7], qpos):
            joint.set_drive_target(x)

    def get_drive_target(self):
        return [joint.get_drive_target() for joint in self.joints[:7]]

    def get_qvel(self):
        return self.art_obj.get_qvel()[:7]

    def set_drive_velocity_target(self, qvel):
        for joint, x in zip(self.joints[:7], qvel):
            joint.set_drive_velocity_target(x)

    def get_drive_velocity_target(self):
        return [joint.get_drive_velocity_target() for joint in self.joints[:7]]

    def set_gripper_qpos(self, qpos):
        arm_qpos = self.art_obj.get_qpos()[:7]
        self.art_obj.set_qpos(np.hstack([arm_qpos, qpos]))

    def set_gripper_drive_target(self, qpos):
        for joint, x in zip(self.joints[7:], qpos):
            joint.set_drive_target(x)

    # ---------------------------------------------------------------------------- #
    # Helper functions
    # ---------------------------------------------------------------------------- #
    def balance_passive_force(self):
        qf = self.art_obj.compute_passive_force()
        self.art_obj.set_qf(qf)

    def to_radian(self, angle, is_radian):
        if (not is_radian) or (not self.is_radian):
            angle = np.deg2rad(angle)
        return angle

    # ---------------------------------------------------------------------------- #
    # Follow xArm7 Python SDK API
    # ---------------------------------------------------------------------------- #
    def set_servo_angle(self, angle, speed=None, mvacc=None, is_radian=None):
        target_qpos = self.to_radian(angle, is_radian)

        current_qpos = self.art_obj.get_qpos()[:7]
        current_qvel = self.art_obj.get_qvel()[:7]

        if speed is None:
            speed = self._max_speed
        else:
            speed = min(self._max_speed, self.to_radian(speed, is_radian))
        if mvacc is None:
            mvacc = self._max_acc
        else:
            mvacc = min(self._max_acc, self.to_radian(mvacc, is_radian))

        trajectory = parameterize_path(
            current_qpos,
            target_qpos,
            current_velocity=current_qvel,
            max_velocity=speed,
            max_acceleration=mvacc,
            delta_time=1.0 / self._control_freq,
        )

        plan = []
        n = int(np.ceil(trajectory.duration * self._control_freq))
        for i in range(n + 1):
            t = i / self._control_freq
            p, v, a = trajectory.at_time(t)
            plan.append([p, v, a])

        self._plan = plan[1:]

    # ---------------------------------------------------------------------------- #
    # Interface with Simulator
    # ---------------------------------------------------------------------------- #
    def reset(self):
        arm_default_init = np.deg2rad(self.ARM_DEFAULT_INIT)
        self.set_qpos(arm_default_init)
        self.set_drive_target(arm_default_init)

        # 0.045 is hardcoded for pris finger
        gripper_default_init = self.GRIPPER_DEFAULT_INIT / 840 * 0.045
        self.set_gripper_qpos([gripper_default_init] * 2)
        self.set_gripper_drive_target([gripper_default_init] * 2)

        # Online trajectory planning
        self._plan = []

    def step(self):
        self.balance_passive_force()
        if len(self._plan) == 0:
            return
        qpos, qvel, _ = self._plan.pop(0)
        self.set_drive_target(qpos)
        self.set_drive_velocity_target(qvel)


class xArm7Env:
    def __init__(self, sim_freq=500, control_freq=250):
        self._engine = sapien.Engine()
        self._renderer = sapien.SapienRenderer()
        self._engine.set_renderer(self._renderer)

        self._sim_freq = sim_freq
        self._physx_system = sapien.physx.PhysxCpuSystem()
        self._physx_system.set_timestep(1.0 / sim_freq)
        self.scene = sapien.Scene([self._physx_system, sapien.render.RenderSystem()])

        self._control_freq = control_freq
        self._sim_steps_per_control_step = sim_freq // control_freq
        self.robot = xArm7(self.scene, self._control_freq)

        mtl = self._renderer.create_material()
        mtl.base_color = [0.5, 0.8, 0.5, 1.0]
        self.scene.add_ground(0, render_material=mtl)

        self.scene.set_ambient_light([0.3, 0.3, 0.3])
        self.scene.add_point_light([0, 0, 1], [1, 1, 1], shadow=True)

        self.camera = self.scene.add_camera("camera", 1280, 720, 1.57, 0.01, 10)
        self.camera.set_pose(
            sapien.Pose(
                [0.116002, 1.21269, 0.924025], [0.706835, 0.170747, 0.182368, -0.661794]
            )
        )

    def reset(self):
        self.robot.reset()
        self._elapsed_steps = 0

    def step(self):
        if self._elapsed_steps % self._sim_steps_per_control_step == 0:
            self.robot.step()
        self.scene.step()
        self._elapsed_steps += 1


def main():
    from sapien.utils import Viewer

    env = xArm7Env()
    env.reset()

    viewer = Viewer(env._renderer)
    viewer.set_scene(env.scene)

    while not viewer.closed:
        if viewer.window.key_press("n"):
            env.robot.set_servo_angle(
                [2.2, 0, 18.7, 60.3, 0, 60, -90], 30, is_radian=False
            )
        env.step()
        env.scene.update_render()
        viewer.render()


if __name__ == "__main__":
    main()
