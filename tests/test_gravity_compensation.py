'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup, ControlMode
from fourier_grx_dds.controller import RobotController
from fourier_grx_dds.gravity_compensation import GravityCompensator, Upsampler
import time
import math

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    # controller = RobotController(args.config)
    # controller.enable()
    gravity_compensator = GravityCompensator(args.config, target_hz=200)
    gravity_compensator.enable()
    gravity_compensator.enable_gc()
    
    target_position = [0.0]*32
    # target_position[21] = 0.3
    start = time.time()
    k = 0
    while True:
        target_position[18] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[19] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[20] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[21] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[22] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[23] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[24] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        # target_position = controller.joint_positions
        gravity_compensator.run(target_position, enable_track=True)
        if time.time() - start > 10:
            break
        k += 1
        time.sleep(1/200)
    # Disable all of the motors
    gravity_compensator.disable()
    # Destroy the controller
    gravity_compensator.end()

if __name__ == "__main__":
    main()
