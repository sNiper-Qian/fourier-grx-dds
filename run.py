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
    # Move both arms to a specific position
    left_arm_position = [
                            -0.10834163755741072, -0.07329939774949822, 0.06528929994794762, 
                            -1.4866168673727456, -0.15687147335078633, -0.13071683482883256, 
                            -0.17893611111972085  
                        ] 
    right_arm_position = [
                            0.10834163755741072, -0.07329939774949822, 0.06528929994794762, 
                            -1.4866168673727456, -0.15687147335078633, -0.13071683482883256, 
                            0.17893611111972085  
                        ]
    arm_position = left_arm_position + right_arm_position
    controller.move_joints(GR1ControlGroup.UPPER, arm_position, duration=2.0)    
    # Perform forward kinematics and get the SE3 representation of the end effectors
    res = controller.forward_kinematics(chain_names=["left_arm", "right_arm"])
    print("SE3 of left ee:", res[0])
    print("SE3 of right ee:", res[1])
    # Perform inverse kinematics and move the arms to the calculated position
    controller.move_joints(GR1ControlGroup.UPPER, [0.0]*14, duration=2.0)
    controller.inverse_kinematics(["left_arm", "right_arm"], res, move=True, velocity_scaling_factor=0.1)
    time.sleep(1)
    # Move the arms back to the initial position
    controller.move_joints(GR1ControlGroup.UPPER, [0.0]*14, duration=2.0)
    time.sleep(1)
    # Disable all of the motors
    controller.disable()
    # Destroy the controller
    controller.end()

if __name__ == "__main__":
    main()
