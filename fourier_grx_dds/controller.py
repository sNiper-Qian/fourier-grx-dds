from pathlib import Path
from loguru import logger
from omegaconf import OmegaConf

import os
import numpy as np
import time
import threading

from fourier_grx_dds.exceptions import FourierValueError
from fourier_grx_dds.kinematics import KinematicsSolver
from fourier_grx_dds.utils import BaseControlGroup, GR1ControlGroup, GR2ControlGroup, Trajectory
from fourier_grx_dds.state_machine import DDSPipeline
from fourier_grx_dds.utils import ControlMode
from fourier_grx_dds.pydds.parallel_joints_solver import PoseSolver

class RobotController:
    def __init__(self, 
                 cfg_path: Path,
                 ):
        """
        Args:
            cfg_path (Path): Path to the configuration file.
        """
        self.config = OmegaConf.load(cfg_path)
        self.joints = self.config.joints
        self.joint_names = list(self.joints.keys())
        self.enabled_joint_names = set([joint_name for joint_name in self.joints if self.joints[joint_name]["enable"]]) # TODO: fix joint names
        self.robot_type = self.config.robot.upper()
        self.encoders_state_path = Path(self.config.encoders_state_path)
        self.freq = self.config.frequency
        if self.robot_type.startswith("GR1"):
            self.control_group = GR1ControlGroup
        elif self.robot_type.startswith("GR2"):
            self.control_group = GR2ControlGroup
            raise NotImplementedError("GR2 robot type is not supported yet.")
        else:
            raise ValueError(f"Unknown robot type: {self.robot_type}")

        self.connector = DDSPipeline(self.joints, 
                                     self.config.encoders, 
                                     self.config.imu, 
                                     self.config.frequency, 
                                     self.config.use_imu,
                                     self.enabled_joint_names
                                     )
        self.init_encoders()
        self.connector.set_joints_pid_param()
        logger.info("PID parameters set.")
        time.sleep(1)   
        pvc_states, integrality = self.connector.get_pvc_states()
        if not integrality:
            logger.error("Error: Self-checking failed.")
        else:
            logger.info("Self-checking passed.")
        self.init_kinematics_solver()
        
        self.default_pose_solver_  = PoseSolver(
                                            self.joint_names, 
                                            self.encoders_state, 
                                            self.config["encoders"], 
                                            self.config["joints"],
                                            pvc_states)
        self._abort_event = threading.Event()
        self._move_lock = threading.Lock()
        
    def init_encoders(self):
        """Initialize the encoders state."""
        encoders_state, integrality = self.connector.get_encoders_state()
        assert integrality, f"Error: Can not fetch the whole encoders_state."
        assert os.path.exists(self.encoders_state_path), f"Encoders state file[{self.encoders_state_path}] not founded."
        logger.info(f"Load encoders state from {self.encoders_state_path}")
        self.encoders_state = OmegaConf.load(self.encoders_state_path)
        if integrality:
            for name in encoders_state:
                angle = encoders_state[name].angle
                self.encoders_state[name]["poweron_pose"] = angle
            OmegaConf.save(self.encoders_state, self.encoders_state_path)
            logger.info(f"Encoders poweron state saved to {self.encoders_state_path}")
        logger.info("Encoders initialized.")
    
    def init_kinematics_solver(self):
        """Initialize the kinematics solver."""
        self.kinematics_solver = KinematicsSolver(self.config)
        logger.info("Kinematics solver initialized.")

    @property
    def joint_positions(self):
        """Get the current joint positions of the robot. The joint positions are in radians."""
        pvc_states, pvc_integrality = self.connector.get_pvc_states()
        assert pvc_integrality, f"Get all motor PVC states failed."
        joints_realtime_pose_angle = {key: pvc_states[key].position for key in pvc_states}
        joints_velocity            = {key: pvc_states[key].velocity for key in pvc_states}
        joints_current             = {key: pvc_states[key].current  for key in pvc_states}
        joints_pose, joints_velocity, joints_kinetic = self.default_pose_solver_.solve(joints_realtime_pose_angle, joints_velocity, joints_current)
        all_current_pos_deg = joints_pose
        return np.deg2rad(all_current_pos_deg)
    
    @property
    def joint_velocities(self):
        """Get the current joint velocities of the robot. The joint velocities are in radians/s."""
        pvc_states, pvc_integrality = self.connector.get_pvc_states()
        assert pvc_integrality, f"Get all motor PVC states failed."
        joints_realtime_pose_angle = {key: pvc_states[key].position for key in pvc_states}
        joints_velocity            = {key: pvc_states[key].velocity for key in pvc_states}
        joints_current             = {key: pvc_states[key].current  for key in pvc_states}
        joints_pose, joints_velocity, joints_kinetic = self.default_pose_solver_.solve(joints_realtime_pose_angle, joints_velocity, joints_current)
        all_current_vel_deg = joints_velocity
        return np.deg2rad(all_current_vel_deg)
    
    @property
    def num_joints(self):
        """Get the number of joints."""
        return len(self.joint_names)
    
    @property
    def is_moving(self):
        """Whether the robot is currently moving."""
        return self._move_lock.locked()

    @is_moving.setter
    def is_moving(self, value: bool):
        if value and not self._move_lock.locked():
            self._move_lock.acquire()
        elif not value and self._move_lock.locked():
            self._move_lock.release()

    def set_enable(self, enable:bool):
        """Enable or disable the motors."""
        if enable:
            self.connector.enable_joints()
        else:
            self.connector.disable_joints()
    
    def set_control_mode(self, joint_name: str, control_mode: ControlMode | None = None):
        """Set the control modes for all joints.

        Args:
            control_mode (ControlMode | list[ControlMode] | None, optional): ControlMode can be PD, VELOCITY, CURRENT. Defaults to None.
            group (BaseControlGroup | list | str, optional): The group of joints to set the control modes for. Defaults to GR1ControlGroup.ALL.
        """
        self.connector.set_control_mode(joint_name, control_mode)
    
    def set_control_modes(self, control_mode: ControlMode | list[ControlMode] | None = None):
        """Set the control modes for all joints.

        Args:
            control_mode (ControlMode | list[ControlMode] | None, optional): ControlMode can be PD, VELOCITY, CURRENT. Defaults to None.
            group (BaseControlGroup | list | str, optional): The group of joints to set the control modes for. Defaults to GR1ControlGroup.ALL.
        """
        if isinstance(control_mode, ControlMode):
            out_control_mode = [control_mode] * self.num_joints

        elif isinstance(control_mode, list) and len(control_mode) == self.num_joints:
            out_control_mode = [mode for mode in control_mode]
        
        else:
            raise FourierValueError(f"Invalid control mode input: {control_mode}")
        self.connector.set_control_modes(self.joint_names, out_control_mode)
    
    def get_control_modes(self):
        """Get the control modes for all joints."""
        # TODO: get control modes
        raise NotImplementedError("Method not implemented.")
    
    def get_group_position(self, group: BaseControlGroup):
        """Get the joint positions of a group."""
        return self.joint_positions[group.slice].copy()
    
    def get_group_position_by_name(self, name: str):
        """Get the joint positions of a group by its name.

        Args:
            name (str): The name of the group. Available group names: 'left_leg', 'right_leg', 'waist', 'head', 'left_arm', 'right_arm'.
        """

        try:
            group = self.control_group[name.upper()]
            return self.get_group_position(group)
        except KeyError as ex:
            raise FourierValueError(f"Unknown group name: {name}") from ex
    
    def enable(self):
        """Enable the motors."""
        self.set_enable(True)
    
    def disable(self):
        """Disable the motors."""
        self.set_enable(False)

    def set_gains(
            self,
            position_control_kp: list[float],
            velocity_control_kp: list[float],
            velocity_control_ki: list[float],
            joint_names: list[str] | None = None,
    ):
        """Set PID gains.
        
        Args:
            position_control_kp (list[float]): The proportional gains for position control. 
            velocity_control_kp (list[float]): The proportional gains for velocity control.
            velocity_control_ki (list[float]): The integral gains for velocity control.
            joint_names (list[str], optional): The names of the joints to set the gains for. Defaults to None, which sets all of the joints.
        """
        if joint_names is None:
            assert len(position_control_kp) == len(velocity_control_kp) == len(velocity_control_ki) == self.num_joints, "Invalid gains shape."
            joint_names = self.joint_names
        else:
            assert len(joint_names) == len(position_control_kp) == len(velocity_control_kp) == len(velocity_control_ki), "Invalid gains shape."
        self.connector.set_custom_joints_pid_param(position_control_kp, velocity_control_kp, velocity_control_ki, joint_names)

    def get_gains(self):
        # TODO: get gains
        """Get the control gains for all joints."""
        raise NotImplementedError("Method not implemented.")
    
    def move_joints(
            self,
            group: BaseControlGroup | list | str,
            positions: np.ndarray | list,
            duration: float = 0.0,
            degrees: bool = False,
            blocking: bool = True,
        ):
        """Move in joint space with time duration.

        Move in joint space with time duration in a separate thread. Can be aborted using `abort()`. Can be blocking.
        If the duration is set to 0, the joints will move in their maximum speed without interpolation.
        """
        positions = np.rad2deg(positions) if not degrees else np.asarray(positions)
        target_pos = np.rad2deg(self.joint_positions.copy())

        if isinstance(group, BaseControlGroup):
            if positions.shape != (group.num_joints,):
                raise ValueError(f"Invalid joint position shape: {positions.shape}, expected: {(group.num_joints,)}")
            target_pos[group.slice] = positions
        elif isinstance(group, list):
            if len(group) != len(positions):
                raise ValueError(f"Invalid joint position shape: {positions.shape}, expected: {(len(group),)}")
            target_pos[group] = positions
        elif isinstance(group, str):
            try:
                target_pos[BaseControlGroup[group.upper()].slice] = positions
            except KeyError as ex:
                raise FourierValueError(f"Unknown group name: {group}") from ex
        
        target_pos = self.default_pose_solver_.inverse(target_pos) # solve target pose
        current_pos = np.rad2deg(self.joint_positions.copy())
        current_pos = self.default_pose_solver_.inverse(current_pos) # solve current pose
        
        def pos2cmd(pos):
            return [(joint_name, pos[i]) for i, joint_name in enumerate(self.joint_names)]
        
        def task(current_pos, target_pos, duration):
            with self._move_lock:
                trajectory = Trajectory(
                    start=current_pos,
                    end=target_pos,
                    duration=duration,
                )

                start_time = time.time()
                while not (trajectory.finished(t := time.time() - start_time) or self._abort_event.is_set()):
                    pos = trajectory.at(t)
                    self.connector.move_joints(pos2cmd(pos))
                    time.sleep(1 / self.freq)

            self._abort_event.clear()
        
        if duration == 0.0:
            self.connector.move_joints(pos2cmd(target_pos))
        elif duration > 0.0 and blocking:
            task(current_pos, target_pos, duration)
        else:
            thread = threading.Thread(name="RobotController.move_joints", target=task)
            thread.daemon = True
            thread.start()
    
    def set_currents(self, group: BaseControlGroup | list | str, currents: np.ndarray | list):
        """Set the current of the robot."""
        currents = np.asarray(currents)
        target_currents = np.zeros(self.num_joints)
        if isinstance(group, BaseControlGroup):
            if currents.shape != (group.num_joints,):
                raise ValueError(f"Invalid joint position shape: {currents.shape}, expected: {(group.num_joints,)}")
            target_currents[group.slice] = currents
        elif isinstance(group, list):
            if len(group) != len(currents):
                raise ValueError(f"Invalid joint position shape: {currents.shape}, expected: {(len(group),)}")
            target_currents[group] = currents
        elif isinstance(group, str):
            try:
                target_currents[BaseControlGroup[group.upper()].slice] = currents
            except KeyError as ex:
                raise FourierValueError(f"Unknown group name: {group}") from ex

        def pos2cmd(currents):
            return [(joint_name, currents[i]) for i, joint_name in enumerate(self.joint_names)]
        self.connector.set_current(pos2cmd(target_currents))
    
    def forward_kinematics(self, chain_names: list[str], q: np.ndarray | None = None) -> list[np.ndarray]:
        """Get the end effector pose of the specified chains in base_link frame.

        !!! info
            The available chain names are: `head`, `left_arm`, `right_arm`, with corresponding end effector frames: `head_yaw_link`, `left_end_effector_link`, `right_end_effector_link`, and the transformation matrices are in the `base_link` frame.

        Args:
            chain_names (list[str]): The chains to get the end effector pose of. Available chain names: 'head', 'left_arm', 'right_arm'.
            q (np.ndarray, optional): The robot confiuration to do forward kinematics in. Defaults to None.

        Returns:
            list: The end effector pose is a list of 4x4 transformation matrices. The order of the matrices is the same as the order of the chain names. The transformation matrix is in the `base_link` frame.
        """
        res = []
        for chain in chain_names:
            if chain not in ["head", "left_arm", "right_arm"]:
                raise ValueError(f"Unknown chain name: {chain}")
            current_q = self.joint_positions.copy()
            if q is not None:
                self.kinematics_solver.set_joint_positions(self.joint_names, q)
            else:
                self.kinematics_solver.set_joint_positions(self.joint_names, current_q)
            res.append(self.kinematics_solver.get_transform(self.config.ee_link[chain], "base_link"))
            # Recover the original joint positions
            self.kinematics_solver.set_joint_positions(self.joint_names, current_q)
        return res

    def inverse_kinematics(
            self,
            chain_names: list[str],
            targets: list[np.ndarray],
            move=False,
            dt: float = 0.005,
            velocity_scaling_factor: float = 1.0,
            convergence_threshold: float = 1e-8,
    ) -> np.ndarray:
        """Get the joint positions for the specified chains to reach the target pose in base_link frame.

        Args:
            chain_names (list[str]): The chains to get the joint positions for. Available chain names: 'head', 'left_arm', 'right_arm'.
            targets (list[np.ndarray]): The target poses in base_link frame.
            move (bool, optional): Whether to move the robot to the target pose. Defaults to False.
            dt (float): The time step for the inverse kinematics.

        Returns:
            np.ndarray: The joint positions to reach the target pose (in radians).
        """

        left_target = None if "left_arm" not in chain_names else targets[chain_names.index("left_arm")]
        right_target = None if "right_arm" not in chain_names else targets[chain_names.index("right_arm")]
        head_target = None if "head" not in chain_names else targets[chain_names.index("head")]
        self.kinematics_solver.solve(left_target, right_target, head_target, dt)
        q = self.kinematics_solver.get_joint_positions(self.joint_names)
        if move:
            self.move_joints(GR1ControlGroup.ALL, q, duration=dt/velocity_scaling_factor*100, blocking=True)
            self.is_moving = False
        return q
        
    def end(self):
        """End the robot control."""
        self.connector.destroy()
    
