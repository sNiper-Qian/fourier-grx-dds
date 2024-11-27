import itertools
import logging
from collections.abc import Sequence

import numpy as np
import pink
import pinocchio as pin
from omegaconf import DictConfig, OmegaConf

from pathlib import Path

logger = logging.getLogger(__name__)

class RobotWrapper:
    def __init__(self, config: DictConfig):
        self.config = config
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=str(self.config.urdf_path),
            package_dirs=[str(d) for d in self.config.urdf_package_dirs],
            root_joint=pin.JointModelFreeFlyer(),
        )
        self.full_model = self.robot.model # model with all joints
        self.joint_names = list(self.config.joints.keys())
        if self.config.joints_to_lock:
            self.disabled_joint_names = self.config.joints_to_lock
        else:
            self.disabled_joint_names = []
        # self.disabled_joint_names = set([joint_name for joint_name in self.config.joints if not config.joints[joint_name]["enable"]])
        if len(self.disabled_joint_names) > 0:
            logger.info(f"Locking joints: {self.disabled_joint_names}")
            self.robot = self.robot.buildReducedRobot(self.disabled_joint_names)

        for name, prop in self.config.joints.items():
            if name in self.disabled_joint_names:
                continue
            self.set_joint_limits(name, np.deg2rad(prop["min_pose_degree"]), np.deg2rad(prop["max_pose_degree"]))

        if self.config.self_collision.enable:
            for g1, g2 in itertools.combinations(self.config.self_collision.enabled_links, 2):
                logger.info(f"Adding collision pair: {g1} - {g2}")
                self.robot.collision_model.addCollisionPair(
                    pin.CollisionPair(
                        self.robot.collision_model.getGeometryId(g1 + "_0"),
                        self.robot.collision_model.getGeometryId(g2 + "_0"),
                    )
                )

        self.robot.rebuildData()

        self.robot.collision_data.enable_contact = True

        self.configuration = pink.Configuration(
            self.robot.model,
            self.robot.data,
            self.robot.q0,
            collision_model=self.robot.collision_model,
            collision_data=self.robot.collision_data,
        )
        self.configuration.update(self.robot.q0)

        self.viz = None
        if self.config.visualize:
            self.viz = pin.visualize.MeshcatVisualizer(
                self.robot.model, self.robot.collision_model, self.robot.visual_model
            )
            self.robot.setVisualizer(self.viz, init=False)
            self.viz.initViewer(open=False, loadModel=True)

            self.viz.displayCollisions(self.config.display_collisions)

            self.viz.displayFrames(
                True,
                [self.robot.model.getFrameId(f) for f in self.config.displayed_frames],
            )

            self.viz.display(self.configuration.q)

    @property
    def model(self):
        return self.robot.model

    @property
    def data(self):
        return self.robot.data

    @property
    def q0(self):
        return self.robot.q0

    @property
    def q_real(self):
        """Get q vector in the real robot convention (length is 32, order same as config.joint_names)."""
        q = []
        for name in self.joint_names:
            q.append(self.get_q_from_name(name))
        return np.array(q)

    @q_real.setter
    def q_real(self, q: np.ndarray):
        """Set from a q vector in the real robot convention (length is 32, order same as config.joint_names))."""
        self.set_joint_positions(self.joint_names, q)

    def q_real2pink(self, q: np.ndarray):
        """Convert q vector from real robot convention to pink convention."""
        q_pink = self.configuration.q.copy()
        for name, value in zip(self.joint_names, q, strict=True):
            q_pink[self.get_idx_q_from_name(name)] = value
        return q_pink

    def q_pink2real(self, q: np.ndarray):
        """Convert q vector from pink convention to real robot convention."""
        q_real = []
        for name in self.joint_names:
            q_real.append(q[self.get_idx_q_from_name(name)])
        return np.array(q_real)

    def dq_pink2real(self, dq_pin: np.ndarray):
        """Convert dq vector from pinocchio convention to real robot convention."""
        dq_real = []
        for name in self.joint_names:
            if name in self.disabled_joint_names:
                dq_real.append(0.0)
            else:
                dq_real.append(dq_pin[self.get_idx_v_from_name(name)])
        return np.array(dq_real)
        
    def dq_real2pink(self, dq_real: np.ndarray):
        """Convert dq vector from real robot convention to pinocchio convention."""
        dq_pin = self.configuration.data.dq_after.copy()
        for name, value in zip(self.joint_names, dq_real, strict=True):
            if name in self.disabled_joint_names:
                continue
            dq_pin[self.get_idx_v_from_name(name)] = value
        return dq_pin

    def get_joint_by_name(self, name: str):
        """Get joint object by its name."""
        try:
            joint_id = self.full_model.getJointId(name)
            joint = self.full_model.joints[joint_id]
            return joint
        except IndexError as err:
            raise IndexError(f"Joint {name} not found in robot model") from err

    def get_idx_q_from_name(self, name: str):
        """Get joint index in Pinocchio configuration vector by its name.

        Args:
            name (str): Name of the joint.
        """
        return self.get_joint_by_name(name).idx_q

    def get_q_from_name(self, name: str):
        """Get joint position in Pinocchio configuration vector by its name.

        Args:
            name (str): Name of the joint.
        """
        if name in self.disabled_joint_names:
            return 0.0
        return self.configuration.q[self.get_idx_q_from_name(name)]

    def get_idx_v_from_name(self, name: str):
        """Get joint index in Pinocchio velocity vector by its name.

        Args:
            name (str): Name of the joint.
        """
        return self.get_joint_by_name(name).idx_v

    def get_v_from_name(self, name: str):
        """Get joint velocity in Pinocchio velocity vector by its name.

        Args:
            name (str): Name of the joint.
        """
        if name in self.disabled_joint_names:
            return 0.0
        return self.configuration.data.dq_after[self.get_idx_v_from_name(name)]

    def add_frame(self, frame_name: str, parent: str, transform: pin.SE3 | None = None):
        """Add a dummy frame to the robot model.

        Args:
            frame_name (str): Name of the frame.
            parent (str): Name of the parent frame.
            transform (pin.SE3 | None, optional): Transform from parent frame to the new frame. Defaults to None. If None, identity transform is used.
        """
        if frame_name in self.model.names:
            logger.warning(f"Frame {frame_name} already exists in robot model")
            return
        if self.model.existFrame(parent) is not True:
            logger.warning(f"Parent frame {parent} not found in robot model")
            return

        if transform is None:
            transform = pin.SE3.Identity()
        self.model.addFrame(
            pin.Frame(
                frame_name,
                self.model.frames[self.model.getFrameId(parent)].parent,
                self.model.getFrameId(parent),
                transform,
                pin.FrameType.OP_FRAME,
            )
        )

    def set_joint_positions(
        self,
        joint_names: list[str],
        positions: np.ndarray,
        degrees: bool = False,
        clip: bool = True,
    ):
        """Set joint positions in the robot configuration.

        Args:
            joint_names (list[str]): Names of the joints.
            positions (np.ndarray): Joint positions, AKA `q` vector in the robot definition.
            degrees (bool, optional): If True, positions are in degrees. Defaults to False.
            clip (bool, optional): If True, clip joint limits. Defaults to True.
        """
        if degrees:
            positions = np.deg2rad(positions)
        current_q = self.configuration.q.copy()
        for joint_name, position in zip(joint_names, positions, strict=True):
            if joint_name in self.disabled_joint_names:
                continue
            q_idx = self.get_idx_q_from_name(joint_name)

            # clip joint limits
            if clip:
                lower = self.robot.model.lowerPositionLimit[q_idx]
                upper = self.robot.model.upperPositionLimit[q_idx]
                position = np.clip(position, lower, upper)

            current_q[q_idx] = position
        self.configuration.update(current_q)

    def get_joint_positions(self, joint_names: Sequence[str]) -> np.ndarray:
        """Get joint positions of given joints.

        Args:
            joint_names (Sequence[str]): list of joint names

        Returns:
            q (NDArray): joint positions
        """
        return np.array([self.get_q_from_name(name) for name in joint_names])

    def set_joint_limits(self, joint_name: str, lower: float, upper: float):
        """Set joint upper and lower limits. Note this does not update the robot data.

        Args:
            joint_name (str): Name of the joint.
            lower (float): Lower limit.
            upper (float): Upper limit.
        """
        if joint_name not in self.robot.model.names:
            logger.warning(f"Joint {joint_name} not found in robot model")
            return
        self.robot.model.lowerPositionLimit[self.get_idx_q_from_name(joint_name)] = lower
        self.robot.model.upperPositionLimit[self.get_idx_q_from_name(joint_name)] = upper

    def set_velocity_limit(self, joint_name: str, limit: float):
        """Set joint velocity limit. Note this does not update the robot data.

        Args:
            joint_name (str): Name of the joint.
            limit (float): Velocity limit.
        """
        if joint_name not in self.robot.model.names:
            logger.warning(f"Joint {joint_name} not found in robot model")
            return
        self.robot.model.velocityLimit[self.get_idx_v_from_name(joint_name)] = limit

    def frame_placement(self, q: np.ndarray, frame_name: str) -> pin.SE3:
        if len(q) == 32:
            q = self.q_real2pink(q)

        frame_idx = self.model.getFrameId(frame_name)
        return self.robot.framePlacement(q, frame_idx)

    def get_transforms(self, frame_names: Sequence[str]) -> list[pin.SE3]:
        return [self.configuration.get_transform_frame_to_world(frame_name).np for frame_name in frame_names]

    def update_display(self):
        """Display the current configuration in the visualizer."""
        if self.viz:
            self.viz.display(self.configuration.q)
