'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup, ControlMode
from fourier_grx_dds.controller import RobotController
from fourier_grx_dds.gravity_compensation import GravityCompensation, Upsampler
import time
import math

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    controller = RobotController(args.config)
    controller.enable()
    gravity_compensator = GravityCompensation(controller, target_hz=500)
    upsampler = Upsampler(gravity_compensator,
                          gravity_compensation=True,
                          )
    
    target_position = [0.0]*32
    # target_position[21] = 0.3
    start = time.time()
    k = 0
    upsampler.start()
    while True:
        target_position[18] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[19] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[20] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[21] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[22] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[23] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[24] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        # target_position = controller.joint_positions
        # gravity_compensator.run(target_position, enable_track=True)
        upsampler.put(target_position)
        if time.time() - start > 5:
            break
        k += 1
        time.sleep(1/60)
    upsampler.stop()
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
    # Disable all of the motors
    controller.disable()
    # Destroy the controller
    controller.end()

if __name__ == "__main__":
    main()
