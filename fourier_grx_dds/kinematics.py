from omegaconf import DictConfig, OmegaConf
from typing import Literal

import numpy as np
import pink
import pinocchio as pin
import qpsolvers

from fourier_grx_dds.robot_wrapper import RobotWrapper
from pathlib import Path

class KinematicsSolver(RobotWrapper):
    def __init__(self, config: DictConfig):
        super().__init__(config)

        self.tasks = {}
        self.barriers = {}

        if self.viz:
            from itertools import product

            import meshcat.geometry as g

            if config.debug:
                self.viz.viewer["left_ee_target"].set_object(g.Box([0.1, 0.1, 0.1]))
                self.viz.viewer["right_ee_target"].set_object(g.Box([0.1, 0.1, 0.1]))
                self.viz.viewer["head"].set_object(g.Box([0.1, 0.1, 0.1]))

            if config.get("debug_hand", False):
                for side, finger in product(["left", "right"], range(26)):
                    if finger == 0:
                        self.viz.viewer[f"{side}_hand/{finger}"].set_object(g.Box([0.02, 0.02, 0.02]))
                    else:
                        self.viz.viewer[f"{side}_hand/{finger}"].set_object(g.Sphere(0.01))
                    if side == "left":
                        self.viz.viewer[f"{side}_hand/{finger}"].set_property("color", [1, 0, 0, 1])
            self.viz.display(self.configuration.q)

        self.build_tasks()

    def build_tasks(self):
        r_hand_task = pink.tasks.RelativeFrameTask(
            self.config.named_links.right_end_effector_link,
            self.config.named_links.root_link,
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        l_hand_task = pink.tasks.RelativeFrameTask(
            self.config.named_links.left_end_effector_link,
            self.config.named_links.root_link,
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        head_task = pink.tasks.RelativeFrameTask(
            self.config.named_links.head_link,
            self.config.named_links.root_link,
            position_cost=0.0,
            orientation_cost=1.0,
            gain=0.5,
            lm_damping=1e-1,
        )

        posture_task = pink.tasks.PostureTask(cost=1e-2)
        self.tasks = {
            "l_hand_task": l_hand_task,
            "r_hand_task": r_hand_task,
            "head_task": head_task,
            "posture_task": posture_task,
        }

        if self.config.self_collision.enable:
            collision_barrier = pink.barriers.SelfCollisionBarrier(
                n_collision_pairs=len(self.robot.collision_model.collisionPairs),
                gain=20.0,
                safe_displacement_gain=1.0,
                d_min=self.config.self_collision.min_distance,
            )

            self.barriers = {
                "collision_barrier": collision_barrier,
            }

    def set_posture_target_from_current_configuration(self):
        self.tasks["posture_task"].set_target_from_configuration(self.configuration)

    def solve(
        self,
        left_target: np.ndarray | None,
        right_target: np.ndarray | None,
        head_target: np.ndarray | None,
        dt: float,
    ):
        if left_target is None and right_target is None and head_target is None:
            return
        tasks = []
        if left_target is not None:
            left_target = pin.SE3(translation=left_target[:3, 3], rotation=left_target[:3, :3])
            if self.viz:
                self.viz.viewer["left_ee_target"].set_transform(left_target.homogeneous)
            self.tasks["l_hand_task"].set_target(left_target)
            tasks.append(self.tasks["l_hand_task"])

        if right_target is not None:
            right_target = pin.SE3(translation=right_target[:3, 3], rotation=right_target[:3, :3])
            if self.viz:
                self.viz.viewer["right_ee_target"].set_transform(right_target.homogeneous)
            self.tasks["r_hand_task"].set_target(right_target)
            tasks.append(self.tasks["r_hand_task"])
        
        if head_target is not None:
            head_target = pin.SE3(head_target[:3, :3], np.array([0.0, 0.0, 0.0]))
            if self.viz:
                self.viz.viewer["head"].set_transform(head_target.homogeneous)
            self.tasks["head_task"].set_target(head_target)
            tasks.append(self.tasks["head_task"])

        solver = qpsolvers.available_solvers[0]

        if "quadprog" in qpsolvers.available_solvers:
            solver = "quadprog"

        velocity = pink.solve_ik(
            self.configuration,
            tasks,
            dt,
            solver=solver,
            barriers=self.barriers.values(),
            safety_break=False,
        )
        self.configuration.integrate_inplace(velocity, dt)

    def get_transform(self,to_frame,from_frame):
        transform = self.configuration.get_transform(to_frame,from_frame).np
        return transform

    def so3_to_ortho6d(self,so3):
        return so3[:, :2].transpose().reshape(-1)

    def se3_to_xyzortho6d(self,se3):
        """
        Convert SE(3) to continuous 6D rotation representation.
        """
        so3 = se3[:3, :3]
        xyz = se3[:3, 3]
        ortho6d = self.so3_to_ortho6d(so3)
        return np.concatenate([xyz, ortho6d])
