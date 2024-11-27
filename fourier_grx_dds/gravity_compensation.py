from __future__ import annotations

import time
from collections import deque
from typing import TYPE_CHECKING, Any

import numpy as np
import pinocchio as pin
from loguru import logger
from omegaconf import DictConfig
from pathlib import Path

from fourier_grx_dds.utils import ControlMode, BaseControlGroup, Trajectory
from fourier_grx_dds.exceptions import FourierValueError
from fourier_grx_dds.controller import RobotController
from fourier_grx_dds.robot_wrapper import RobotWrapper

import threading
from scipy.interpolate import PchipInterpolator

class JointImpedanceSolver:
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
            raise FourierValueError("Dimensional Error in the Impedance Coefficient Matrix ")
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

class GravityCompensator(RobotController):
    def __init__(self, cfg_path: Path, dt: float | None = None, target_hz: int = 200):
        """initialize upbody parameters"""
        super().__init__(cfg_path=cfg_path)
        self.config = self.config
        self.ctrl_idx = self.control_group.UPPER_EXTENDED.list
        self.target_hz = target_hz
        self.joint_names = list(self.config.joints.keys())

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
        self.config = self.config.get("impedance_controller", {})
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
        self.impedance_solver = JointImpedanceSolver(order, dim, k, b, m, max_effort)

    def enable_gc(self):
        if self.enabled:
            return
        self.enabled = True
        self.position_history.clear()
        self._set_mixed_mode_gains()

    def disable_gc(self):
        if not self.enabled:
            return
        self.position_history.clear()
        self.enabled = False
        self.set_control_modes([ControlMode.POSITION]*self.num_joints)
        time.sleep(0.1)
        # self._restore_origin_control_config()
    
    def disable(self):
        self.disable_gc()
        super().disable()

    def _record_origin_control_config(self):
        self.original_ctrl_mode = self.get_control_modes()
        # self.origin_gains = {
        #     "position_control_kp": self.server.control_commands["position_control_kp"],
        #     "velocity_control_kp": self.server.control_commands["velocity_control_kp"],
        #     "velocity_control_ki": self.server.control_commands["velocity_control_ki"],
        #     "pd_control_kp": self.server.control_commands["pd_control_kp"],
        #     "pd_control_kd": self.server.control_commands["pd_control_kd"],
        # }
        self.original_gain = self.get_gains()

    def _restore_origin_control_config(self):
        self.set_control_modes(self.original_ctrl_mode)
        self.set_gains(self.original_gain)

    def _set_mixed_mode_gains(self):
        """Set the control mode and corresponding PID parameters"""

        # self._record_origin_control_config()
        control_mode = (
            [ControlMode.POSITION] * (32 - len(self.ctrl_idx))
            + [ControlMode.CURRENT] * 3
            + [ControlMode.POSITION] * 3
            + [ControlMode.CURRENT] * 4
            + [ControlMode.POSITION] * 3
            + [ControlMode.CURRENT] * 4
            + [ControlMode.POSITION] * 3
        )  # set lower as none and upbody as PD
        self.set_control_modes(control_mode)
    
    def pause(self):
        curr_cmd = np.zeros(32)
        self.set_currents(self.control_group.ALL, curr_cmd)

        q_real = self.joint_positions.copy()
        pos_cmd = q_real.copy()
        self.move_joints(self.control_group.ALL, pos_cmd)
    
    def stop(self):
        q_real = self.joint_positions.copy()
        pos_cmd = q_real.copy()
        self.run(p=pos_cmd)

    def run(
        self,
        p: np.ndarray | list | None = None,
        v: np.ndarray | list | None = None,
        acc: np.ndarray | list| None = None,
        enable_track: bool = False,
    ):
        # start_time = time.perf_counter()
        p = np.asarray(p) if p is not None else None
        v = np.asarray(v) if v is not None else None
        acc = np.asarray(acc) if acc is not None else None
        if isinstance(p, np.ndarray) and p.shape[0] != self.num_joints:
            raise FourierValueError("The length of the position vector must match the number of joints.")
        if isinstance(v, np.ndarray) and v.shape[0] != self.num_joints:
            raise FourierValueError("The length of the velocity vector must match the number of joints.")
        if isinstance(acc, np.ndarray) and acc.shape[0] != self.num_joints:
            raise FourierValueError("The length of the acceleration vector must match the number of joints.")

        if p is not None:
            now = time.perf_counter()
            if len(self.position_history) == 0:
                self.position_history.append((now, p[self.ctrl_idx]))
            else:
                if now - self.position_history[-1][0] > 1.5 / self.target_hz:  # minimum frequency 
                    logger.warning(f"[ImpedanceController] The recommended control frequency is above {self.target_hz}.")
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

        q_real = self.joint_positions.copy()
        qd_real = self.joint_velocities.copy()
        qdd_real = np.zeros_like(qd_real)

        q_pin = self.kinematics_solver.q_real2pink(q_real)

        gravity = self.kinematics_solver.dq_pink2real(pin.computeGeneralizedGravity(self.kinematics_solver.model, self.kinematics_solver.data, q_pin))[-20:]
        if enable_track is True and p is not None:
            # if p is None:
            #     p = np.zeros(len(self.ctrl_idx))  # rad, length = 20
            if v is None:
                v = np.zeros_like(p)
            if acc is None:
                acc = np.zeros_like(p)

            self.impedance_solver.update_controller_state(
                q_real[self.ctrl_idx],
                qd_real[self.ctrl_idx],
                qdd_real[self.ctrl_idx],
                p[self.ctrl_idx],
                v[self.ctrl_idx],
                acc[self.ctrl_idx],
            )

            impedance_torque = self.impedance_solver.get_output(gravity)  # length: 20

            # Tracking the desired position trajectory and implementing a simple controller
            impedance_current = [
                index1 / index2 for index1, index2 in zip(impedance_torque, self.current_to_torque, strict=True)
            ]
            try:
                curr_cmd = np.zeros(32)
                curr_cmd[self.ctrl_idx] = impedance_current * self.default_pose_solver_.joints_direction[self.ctrl_idx]
                self.set_currents(self.control_group.ALL, curr_cmd)

                pos_cmd = q_real.copy()
                pos_cmd[[15, 16, 17, 22, 23, 24, 29, 30, 31]] = p[[15, 16, 17, 22, 23, 24, 29, 30, 31]]
                super().move_joints(self.control_group.ALL, pos_cmd)
                return True
            except Exception as e:
                logger.error(f"Error during impedance control: {e}")
                return False
        else:
            # Zero-force drag: Gravity compensation
            gravity_control_current = [
                index1 / index2 for index1, index2 in zip(gravity, self.current_to_torque, strict=True)
            ]
            curr_cmd = np.zeros(32)
            curr_cmd[self.ctrl_idx] = gravity_control_current
            self.set_currents(self.control_group.ALL, curr_cmd)
            return True
    
    def move_joints(
            self,
            group: BaseControlGroup | list | str,
            positions: np.ndarray | list,
            duration: float = 0.0,
            degrees: bool = False,
            blocking: bool = True,
            gravity_compensation: bool = False,
        ):
        """Move in joint space with time duration.

        Move in joint space with time duration in a separate thread. Can be aborted using `abort()`. Can be blocking.
        If the duration is set to 0, the joints will move in their maximum speed without interpolation.
        """
        if not gravity_compensation:
            if self.enabled:
                self.disable_gc()
            super().move_joints(group, positions, duration, degrees, blocking)
        else:
            if not self.enabled:
                self.enable_gc()
            positions = np.asarray(positions) if not degrees else np.deg2rad(positions)
            target_pos = self.joint_positions.copy()

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
            self.run(p=target_pos, enable_track=True)

class CommandHistory:
    def __init__(self, target_hz: int = 200, horizon: float = 0.1):
        self.horizon = horizon
        history_length = int(horizon * target_hz * 1.5)
        self.history = deque(maxlen=history_length)
        self.new_command_event = threading.Event()

    def put(self, command: np.ndarray):
        ts = time.monotonic()
        self.history.append((ts, command))
        self._discard_old(ts)
        self.new_command_event.set()

    def _discard_old(self, current_ts: float):
        while self.history and self.history[0][0] < current_ts - self.horizon:
            self.history.popleft()

    def get(self):
        if len(self.history) < 1:
            return None, None

        if not self.new_command_event.is_set():
            return None, None

        self.new_command_event.clear()

        ts = np.array([t for t, _ in self.history])
        commands = np.array([c for _, c in self.history])

        return ts, commands

def pchip_interpolate(timestamps: np.ndarray, commands: np.ndarray, target_hz: int = 200) -> np.ndarray:
    if len(timestamps) < 2:
        raise ValueError("At least two timestamps are required for interpolation")

    elapsed_time = timestamps[-1] - timestamps[-2]
    num_steps = int(elapsed_time * target_hz)

    if num_steps < 2:
        return commands

    pchip = PchipInterpolator(timestamps, commands, axis=0, extrapolate=True)

    x_interp = np.linspace(timestamps[-2], timestamps[-1], num_steps)

    output = pchip(x_interp)

    return output

class Upsampler(threading.Thread):
    def __init__(
        self,
        controller: GravityCompensator,
        dimension: int = 32,
        initial_command: np.ndarray | None = None,
        gravity_compensation: bool = False,
    ):
        self.controller = controller
        self.controller.enable()
        self.dimension = dimension
        self.target_hz = controller.target_hz
        self.target_dt = 1 / self.target_hz
        self.gravity_compensation = gravity_compensation
        self.command_history = CommandHistory()
        if initial_command is not None and len(initial_command) == dimension:
            self.last_command = initial_command
        else:
            self.last_command = None
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.paused = False
        self._cmd_lock = threading.Lock()
        super().__init__()

    def pause(self):
        self.pause_event.set()

    def unpause(self):
        logger.info("Upsampler unpaused.")
        self.last_command = None
        self.command_history = CommandHistory()
        self.pause_event.clear()
        self.paused = False

    def put(self, command: np.ndarray):
        if self.pause_event.is_set():
            logger.info("Upsampler paused, unpausing.")
            self.unpause()
        self.command_history.put(command)

    def get(self):
        timestamps, commands = self.command_history.get()
        if timestamps is None or commands is None or len(timestamps) < 2:
            with self._cmd_lock:
                if self.last_command is None:
                    return None
                return np.array([self.last_command])

        commands = pchip_interpolate(timestamps, commands, self.target_hz)
        return commands

    def stop(self):
        self.controller.disable()
        self.stop_event.set()

    def run(self):
        logger.info("Upsampler started")

        while True:
            if self.pause_event.is_set() and not self.paused:
                self.paused = True
                logger.info("Upsampler paused.")
                # self.server.move_joints(
                #     ControlGroup.ALL, self.client.joint_positions, degrees=False, gravity_compensation=False
                # )
                self.controller.pause()
                self.last_command = None
                self.command_history = CommandHistory()
                continue
            if self.stop_event.is_set():
                logger.info("Upsampler stopped.")
                with self._cmd_lock:
                    if self.controller is not None:
                        if self.last_command is not None:
                            # self.client.move_joints(
                            #     ControlGroup.ALL, self.last_command, degrees=False, gravity_compensation=False
                            # )
                            self.controller.run(p=self.last_command, enable_track=True)
                        else:
                            # self.client.move_joints(
                            #     ControlGroup.ALL, self.client.joint_positions, degrees=False, gravity_compensation=False
                            # )
                            self.controller.stop()
                    break
            commands = self.get()
            if commands is None:
                with self._cmd_lock:
                    if self.last_command is None:
                        time.sleep(self.target_dt)
                        continue
                    self.controller.run(self.last_command, enable_track=True)
                time.sleep(self.target_dt)
                continue
            for command in commands:
                if self.controller.run(p=command, enable_track=True):
                    with self._cmd_lock:
                        self.last_command = command
                time.sleep(self.target_dt)
