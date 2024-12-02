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
    # Test each joint on head
    
    head_position = [0.0, 0.0, 0.0]
    for i in range(3):
        head_position[i] = 0.5
        controller.move_joints(GR1ControlGroup.HEAD, head_position, duration=2.0)
        
    controller.disable()
    # Destroy the controller
    controller.end()

if __name__ == "__main__":
    main()
