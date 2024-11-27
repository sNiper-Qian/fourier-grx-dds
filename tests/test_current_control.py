'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup, ControlMode
from fourier_grx_dds.controller import RobotController
import time

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    controller = RobotController(args.config)
    controller.enable()
    # controller.set_control_mode("right_wrist_yaw_joint", ControlMode.CURRENT)
    controller.set_control_modes([ControlMode.CURRENT]*32)
    currents = [0, 0, 0, 0, 0.3, 0, 0]
    controller.set_currents(GR1ControlGroup.LEFT_ARM, currents)
    time.sleep(0.5)
    controller.set_currents(GR1ControlGroup.LEFT_ARM, [0.0]*7)
    # Disable all of the motors
    controller.disable()
    # Destroy the controller
    controller.end()

if __name__ == "__main__":
    main()
