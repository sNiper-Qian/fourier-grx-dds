'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup, ControlMode
from fourier_grx_dds.gravity_compensation import GravityCompensator, Upsampler
import time
import math

def main() -> None:
    # Initialize your controller with the loaded configurations
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    controller_gc = GravityCompensator(args.config)
    controller_gc.enable()
    
    target_position = [0.0]*32
    # target_position[21] = 0.3
    start = time.time()
    k = 0
    enable_gc = True    
    horizon = 3
    while True:
        target_position[18] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[19] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[20] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[21] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[22] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[23] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[24] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        controller_gc.move_joints(GR1ControlGroup.ALL, positions=target_position, gravity_compensation=enable_gc)
        if time.time() - start > horizon:
            enable_gc = not enable_gc
            horizon += 3
        if time.time() - start > 15:
            break
        k += 1
        time.sleep(1/500)
    # Disable all of the motors
    controller_gc.disable()
    # Destroy the controller
    controller_gc.end()

if __name__ == "__main__":
    main()
