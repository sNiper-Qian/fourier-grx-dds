'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup
from fourier_grx_dds.controller import RobotController
from fourier_grx_dds.gravity_compensation import GravityCompensator
import time
import numpy as np

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    # controller = RobotController(args.config)
    controller = GravityCompensator(args.config)
    controller.enable()
    # Move both arms to a specific position
    head_and_waist_position = [0.0]*controller.control_group.HEAD.num_joints + [0.0]*controller.control_group.WAIST.num_joints
    left_arm_position = [
                            0, 0, 0, 
                            -np.pi/2, 0, 0, 0
                        ] 
    right_arm_position = [
                            0, 0, 0, 
                            -np.pi/2, 0, 0, 0
                        ]
    position = head_and_waist_position + left_arm_position + right_arm_position
    controller.move_joints(controller.control_group.UPPER_EXTENDED, position, duration=2.0) 
    # Perform forward kinematics and get the SE3 representation of the end effectors
    res = controller.forward_kinematics(chain_names=["left_arm", "right_arm"])
    print("SE3 of left ee:", res[0])
    print("SE3 of right ee:", res[1])
    # Perform inverse kinematics and move the arms to the calculated position
    controller.move_joints(controller.control_group.UPPER, [0.0]*14, duration=2.0)
    controller.inverse_kinematics(["left_arm", "right_arm"], res, move=True, velocity_scaling_factor=0.1)
    time.sleep(1)
    # Move the arms back to the initial position
    controller.move_joints(controller.control_group.UPPER, [0.0]*14, duration=2.0)
    time.sleep(1)
    # Disable all of the motors
    controller.disable()
    # Destroy the controller
    controller.end()

if __name__ == "__main__":
    main()
