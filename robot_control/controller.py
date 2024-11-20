from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from loguru import logger
from omegaconf import DictConfig
from robot_control.utils import GR1ControlGroup, GR2ControlGroup
from robot_control.state_machine import DDSPipeline

class BaseController:
    def __init__(self, 
                 controller_cfg: DictConfig
                 ):
        self.frequency = controller_cfg.frequency
        dds_cfg = controller_cfg.dds
        encoder_cfg = controller_cfg.encoder
        self.joints = dds_cfg.joints
        self.joints_name = list(self.joints.keys())
        self.enabled_joints_name = set([joint_name for joint_name in self.joints if self.joints[joint_name]["enable"]]) # TODO: fix joint names
        self.robot_type = controller_cfg.robot.upper()
        if self.robot_type.startswith("GR1"):
            self.control_group = GR1ControlGroup
        elif self.robot_type.startswith("GR2"):
            self.control_group = GR2ControlGroup
            raise NotImplementedError("GR2 robot type is not supported yet.")
        else:
            raise ValueError(f"Unknown robot type: {self.robot_type}")

        self.connector = DDSPipeline(self.joints, dds_cfg["encoders"], dds_cfg["imu"], dds_cfg["freq"], controller_cfg.use_imu)
        
        