'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup
from fourier_grx_dds.controller import RobotController
import time

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    controller = RobotController(args.config)
    controller.enable()
    # Disable all of the motors
    controller.disable()
    # Destroy the controller
    controller.end()

if __name__ == "__main__":
    main()
