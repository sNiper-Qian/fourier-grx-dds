from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from robot_control.utils import BaseControlGroup, GR1ControlGroup, GR2ControlGroup, Trajectory
from robot_control.state_machine import DDSPipeline
from loguru import logger
from omegaconf import OmegaConf
from robot_control.parallel_joints_solver import PoseSolver
import os
import numpy as np
import time
from robot_control.exceptions import FourierValueError
from robot_control.kinematics import KinematicsSolver
import threading

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
        self.solver = KinematicsSolver(self.config)
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

    def set_gains(
            self,
            position_control_kp: list[float] | None = None,
            velocity_control_kp: list[float] | None = None,
            velocity_control_ki: list[float] | None = None,
            pd_control_kp: list[float] | None = None,
            pd_control_kd: list[float] | None = None,
    ):
        """Set PID gains."""
        raise NotImplementedError("Method not implemented.")

    def get_gains(self):
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
                self.solver.set_joint_positions(self.joint_names, q)
            else:
                self.solver.set_joint_positions(self.joint_names, current_q)
            res.append(self.solver.get_transform(self.config.ee_link[chain], "base_link"))
            # Recover the original joint positions
            self.solver.set_joint_positions(self.joint_names, current_q)
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
        self.solver.solve(left_target, right_target, head_target, dt)
        q = self.solver.get_joint_positions(self.joint_names)
        if move:
            self.move_joints(GR1ControlGroup.ALL, q, duration=dt/velocity_scaling_factor*100, blocking=True)
            self.is_moving = False
        return q
        
    def end(self):
        """End the robot control."""
        self.connector.destroy()
    
