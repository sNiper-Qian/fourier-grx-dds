import yaml
import argparse
import sys
sys.path.append("../")
from robot_control.utils import GR1ControlGroup
from robot_control.controller import RobotController
import time

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    controller = RobotController(args.config)
    controller.enable()
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
    controller.set_gains([0], [0], [0], joint_names=["right_elbow_pitch_joint"])
    controller.move_joints(GR1ControlGroup.UPPER, arm_position, duration=2.0)
    
    # res = controller.forward_kinematics(chain_names=["left_arm", "right_arm"])
    # controller.move_joints(GR1ControlGroup.UPPER, [0.0]*14, duration=2.0)
    # controller.inverse_kinematics(["left_arm", "right_arm"], res, move=True)
    # time.sleep(3)
    controller.move_joints(GR1ControlGroup.UPPER, [0.0]*14, duration=2.0)
    controller.end()

if __name__ == "__main__":
    main()
