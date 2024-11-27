from __future__ import annotations

import time
from collections import deque
from typing import TYPE_CHECKING, Any

import numpy as np
import pinocchio as pin
from loguru import logger
from omegaconf import DictConfig

from fourier_grx.sdk.utils import ControlMode

if TYPE_CHECKING:
    from fourier_grx.sdk.robot import RobotWrapper
    # from fourier_grx.sdk.server import RobotServer


class JointImpedanceController:
    def __init__(self, order: int, dim: int, k: np.ndarray, b: np.ndarray, m: np.ndarray, max_effort: np.ndarray):
        """_summary_

        Args:
            order (int): the order of impedance control calculation
            dim (int): the dimension of the parameter
            k (np.ndarray): rigidity coefficient
            b (np.ndarray): damping coefficient
            m (np.ndarray): mass coefficient
            max_effort (np.ndarray): the maximum torque of current output

        Raises:
            Exception: the parameter length does not match
        """
        self.__order = order
        self.__dim = dim
        self.__k = k
        self.__b = b
        self.__m = m
        self.__max_effort = max_effort
        if (
            (3 < self.__order or self.__order < 0)
            or self.__dim <= 0
            or len(self.__k) != self.__dim
            or len(self.__b) != self.__dim
            or len(self.__m) != self.__dim
            or len(self.__max_effort) != self.__dim
        ):
            raise Exception("init failed,check dimension .")
        self.__q_current = np.zeros(self.__dim)
        self.__qd_current = np.zeros(self.__dim)
        self.__qdd_current = np.zeros(self.__dim)
        self.__q_desired = np.zeros(self.__dim)
        self.__qd_desired = np.zeros(self.__dim)
        self.__qdd_desired = np.zeros(self.__dim)

    def set_impedance_coeff(self, impedance_k: np.ndarray, impedance_b: np.ndarray):
        if len(impedance_k) != self.__dim or len(impedance_b) != self.__dim:
            raise ValueError("Dimensional Error in the Impedance Coefficient Matrix ")
        self.__k = impedance_k.copy()
        self.__b = impedance_b.copy()

    def get_impedance_coeff(self, index=None):
        if index is not None:
            return self.__k[index], self.__b[index]
        else:
            return self.__k, self.__b

    def update_controller_state(self, q_current, qd_current, qdd_current, q_desired, qd_desired, qdd_desired):
        self.__q_current = q_current
        self.__qd_current = qd_current
        self.__qdd_current = qdd_current
        self.__q_desired = q_desired
        self.__qd_desired = qd_desired
        self.__qdd_desired = qdd_desired

    def clip_torque(self, output):
        """limit the max output of torque"""
        for i in range(self.__dim):
            if abs(output[i]) > self.__max_effort[i]:
                if output[i] > 0:
                    output[i] = self.__max_effort[i]
                else:
                    output[i] = -self.__max_effort[i]

    def get_output(self, feedforward):
        """calculate the torque of joint space impedance control

        Args:
            feedforward (_type_): gravity torque

        Returns:
            _type_: the torque of impedance control
        """
        if self.__order == 0:
            self.clip_torque(feedforward)
            return feedforward
        elif self.__order == 1:
            feedforward = feedforward - np.multiply(self.__k, (self.__q_current - self.__q_desired))
            self.clip_torque(feedforward)
            return feedforward
        elif self.__order == 2:
            feedforward = (
                feedforward
                - np.multiply(self.__k, (self.__q_current - self.__q_desired))
                - np.multiply(self.__b, (self.__qd_current - self.__qd_desired))
            )
            self.clip_torque(feedforward)
            return feedforward
        elif self.__order == 3:
            feedforward = (
                feedforward
                - np.multiply(self.__k, (self.__q_current - self.__q_desired))
                - np.multiply(self.__b, (self.__qd_current - self.__qd_desired))
                - np.multiply(self.__m, (self.__qdd_current - self.__qdd_desired))
            )
            self.clip_torque(feedforward)
            return feedforward
        else:
            pass


class UpbodyImpedanceController1(JointImpedanceController):
    def __init__(self):
        """initialize upbody parameters"""
        order = 2
        dim = 20
        k = np.array(
            [
                60.0,
                60.0,
                60.0,
                0.0,
                0.0,
                0.0,
                60.0,
                70.0,
                70.0,
                60.0,
                14.0,
                0.0,
                0.0,
                60.0,
                70.0,
                70.0,
                60.0,
                14.0,
                0.0,
                0.0,
            ]
        )
        b = np.array(
            [5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 1.0, 0.0, 0.0]
        )
        m = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        max_effort = np.array(
            [
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
                50.0,
            ]
        )
        super().__init__(order, dim, k, b, m, max_effort)


class UpbodyImpedanceController:
    def __init__(self, server: Any, config: DictConfig, dt: float | None = None):
        """initialize upbody parameters"""

        self.robot: RobotWrapper = server.robot
        self.server = server
        self.ctrl_idx = np.array(list(range(12, 32)))

        self.joint_names = [list(self.robot.config.joint_names)[i] for i in self.ctrl_idx]

        # self.idx_q_pin2real = np.array([self.robot.get_idx_q_from_name(name) for name in self.joint_names])
        # self.idx_qv_pin2real = np.array([self.robot.get_idx_v_from_name(name) for name in self.joint_names])

        self.current_to_torque = [
            2.68,
            2.68,
            2.68,
            1.0,
            1.0,
            1.0,
            5.46,
            5.46,
            6.828,
            6.828,
            1,
            1,
            1,
            5.46,
            5.46,
            6.828,
            6.828,
            1,
            1,
            1,
        ]
        self.config = config.get("upper_impedance_controller", {})

        self.position_history = deque(maxlen=20)
        self.dt = dt

        self.enabled = False
        order = 2
        dim = 20
        k = np.array(
            self.config.get(
                "k",
                [
                    60.0,
                    60.0,
                    60.0,
                    15.0,
                    15.0,
                    15.0,
                    60.0,
                    70.0,
                    70.0,
                    60.0,
                    15.0,
                    15.0,
                    15.0,
                    60.0,
                    70.0,
                    70.0,
                    60.0,
                    15.0,
                    15.0,
                    15.0,
                ],
            )
        )
        b = np.array(
            self.config.get(
                "b",
                [5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
            )
        )
        m = np.array(
            self.config.get(
                "m",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        )
        max_effort = np.array(
            self.config.get(
                "max_effort",
                [
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                    50.0,
                ],
            )
        )
        self.controller = JointImpedanceController(order, dim, k, b, m, max_effort)

    def enable(self):
        if self.enabled:
            return
        self.enabled = True
        self.position_history.clear()
        self._set_mixed_mode_gains()

    def disable(self):
        if not self.enabled:
            return
        self.position_history.clear()
        self.enabled = False
        self._restore_origin_control_config()

    def _record_origin_control_config(self):
        self.original_ctrl_mode = self.server.control_commands["control_mode"]
        self.origin_gains = {
            "position_control_kp": self.server.control_commands["position_control_kp"],
            "velocity_control_kp": self.server.control_commands["velocity_control_kp"],
            "velocity_control_ki": self.server.control_commands["velocity_control_ki"],
            "pd_control_kp": self.server.control_commands["pd_control_kp"],
            "pd_control_kd": self.server.control_commands["pd_control_kd"],
        }

    def _restore_origin_control_config(self):
        self.server.set_control_modes(self.original_ctrl_mode)
        self.server.set_gains(self.origin_gains)

    def _set_mixed_mode_gains(self):
        """Set the control mode and corresponding PID parameters"""

        self._record_origin_control_config()

        control_mode = (
            [ControlMode.PD] * (32 - len(self.ctrl_idx))
            + [ControlMode.CURRENT] * 3
            + [ControlMode.PD] * 3
            + [ControlMode.CURRENT] * 4
            + [ControlMode.PD] * 3
            + [ControlMode.CURRENT] * 4
            + [ControlMode.PD] * 3
        )  # set lower as none and upbody as PD
        self.server.set_control_modes(control_mode)

        # fmt: off
        kp = np.array([
            1.0, 1.0, 1.0, 1.0, 0.1, 0.1,  # left leg
            1.0, 1.0, 1.0, 1.0, 0.1, 0.1,  # right leg
            0.5, 0.5, 0.5,  # waist
            10, 10, 10,  # head
            0.5, 0.5, 0.5, 0.5, 112.06, 10, 10,  # left arm
            0.5, 0.5, 0.5, 0.5, 112.06, 10, 10,  # right arm
        ])

        kd = np.array([
            0.4, 0.2, 0.3, 0.4, 0.06, 0.06,  # left leg
            0.4, 0.2, 0.3, 0.4, 0.06, 0.06,  # right leg
            0.3, 0.3, 0.3,  # waist
            0.5, 0.5, 0.5,  # head
            0.05, 0.05, 0.05, 0.05, 3.7, 0.5, 0.5,  # left arm
            0.05, 0.05, 0.05, 0.05, 3.7, 0.5, 0.5,  # right arm
        ])

        ki = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # left leg
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # right leg
            0.0, 0.0, 0.0,  # waist
            0.0, 0.0, 0.0,  # head
            0.0001, 0.0001, 0.0001, 0.0001, 0.0, 0.0, 0.0,  # left arm
            0.0001, 0.0001, 0.0001, 0.0001, 0.0, 0.0, 0.0,  # right arm
        ])

        #change kp and kd of head, left arm, right arm
        kp[15:18] = np.array(self.config.get("head_kp",[10, 10, 10]))
        kd[15:18] = np.array(self.config.get("head_kd",[0.5, 0.5, 0.5]))

        kp[22:25] = np.array(self.config.get("left_wrist_kp",[112.06, 10, 10]))
        kd[22:25] = np.array(self.config.get("left_wrist_kd",[3.7, 0.5, 0.5]))

        kp[29:32] = np.array(self.config.get("right_wrist_kp",[112.06, 10, 10]))
        kd[29:32] = np.array(self.config.get("right_wrist_kd",[3.7, 0.5, 0.5]))

        # fmt: on
        self.server.set_gains(
            {
                "position_control_kp": kp,
                "velocity_control_kp": kd,
                "velocity_control_ki": ki,
                "pd_control_kp": kp,
                "pd_control_kd": kd,
            }
        )

    def run(
        self,
        p: np.ndarray | None = None,
        v: np.ndarray | None = None,
        acc: np.ndarray | None = None,
        enable_track: bool = False,
    ):
        # start_time = time.perf_counter()
        if p is not None:
            now = time.perf_counter()
            if len(self.position_history) == 0:
                self.position_history.append((now, p[self.ctrl_idx]))
            else:
                if now - self.position_history[-1][0] > 1.5 / 120:  # minimum frequency 120Hz
                    logger.warning("[ImpedanceController] The recommended control frequency is above 120Hz.")
                    self.position_history.clear()
                self.position_history.append((now, p[self.ctrl_idx]))

            # estimate velocity from position history and dt
            if v is None:
                v = np.zeros_like(p)
                # here we have ahistory of timesatmped positions, if history exists, calculate velocity by averaging velocities between all points, dt is the time between the two points in the history
                try:
                    if len(self.position_history) >= 10:
                        dts = np.asarray([t for t, _ in self.position_history])
                        ps = np.asarray([p for _, p in self.position_history])
                        dt = np.diff(dts).reshape(-1, 1)
                        dp = np.diff(ps, axis=0)
                        v[self.ctrl_idx] = np.sum(dp / dt, axis=0) / len(dt)
                except Exception as e:
                    logger.error(f"Error in estimating velocity: {e}")

        q_real = self.server.states.position
        qd_real = self.server.states.velocity
        qdd_real = np.zeros_like(qd_real)

        q_pin = self.robot.q_real2pin(q_real)

        gravity = self.robot.dq_pin2real(pin.computeGeneralizedGravity(self.robot.model, self.robot.data, q_pin))[-20:]

        if enable_track is True and p is not None:
            # if p is None:
            #     p = np.zeros(len(self.ctrl_idx))  # rad, length = 20
            if v is None:
                v = np.zeros_like(p)
            if acc is None:
                acc = np.zeros_like(p)

            self.controller.update_controller_state(
                q_real[self.ctrl_idx],
                qd_real[self.ctrl_idx],
                qdd_real[self.ctrl_idx],
                p[self.ctrl_idx],
                v[self.ctrl_idx],
                acc[self.ctrl_idx],
            )

            impedance_torque = self.controller.get_output(gravity)  # length: 20

            # Tracking the desired position trajectory and implementing a simple controller
            impedance_current = [
                index1 / index2 for index1, index2 in zip(impedance_torque, self.current_to_torque, strict=True)
            ]

            curr_cmd = np.zeros(32)
            curr_cmd[self.ctrl_idx] = impedance_current
            self.server._set_command("current", curr_cmd)

            pos_cmd = q_real.copy()
            pos_cmd[[15, 16, 17, 22, 23, 24, 29, 30, 31]] = p[[15, 16, 17, 22, 23, 24, 29, 30, 31]]
            self.server._set_command("position", pos_cmd, clip_positions=True)

        else:
            # Zero-force drag: Gravity compensation
            gravity_control_current = [
                index1 / index2 for index1, index2 in zip(gravity, self.current_to_torque, strict=True)
            ]
            curr_cmd = np.zeros(32)
            curr_cmd[self.ctrl_idx] = gravity_control_current
            self.server._set_command("current", curr_cmd)

        # end_time = time.perf_counter()
        # elapsed_time = end_time - start_time
        # # # 200HZ
        # sleep_time = 1 / 160
        # # print(elapsed_time)
        # if elapsed_time > sleep_time:
        #     pass
        #     # print("time error...........................................")
        # else:
        #     time.sleep(max(0, sleep_time - elapsed_time))
