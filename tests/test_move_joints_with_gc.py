'''
A simple example to demonstrate the usage of the robot controller.
'''
import argparse
from fourier_grx_dds.utils import GR1ControlGroup, ControlMode, GR2ControlGroup
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
    
    target_position = [0.0]*controller_gc.num_joints
    start = time.time()
    k = 0
    last = time.time()
    while True:
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+0] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+1] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+2] = (0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+3] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+4] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+5] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        target_position[controller_gc.control_group.LEFT_ARM.value[0]+6] = -(0.3 * math.sin(0.01 * k - math.pi / 2) + 0.3)
        print("Control Frequency: ", 1/(time.time() - last))
        last = time.time()
        time.sleep(max(1/200 - (time.time() - last), 0))
        controller_gc.move_joints(controller_gc.control_group.ALL, positions=target_position, gravity_compensation=True)
        if time.time() - start > 15:
            break
        k += 1
        # time.sleep(1/500)
    # Disable all of the motors
    controller_gc.disable()
    # Destroy the controller
    controller_gc.end()

if __name__ == "__main__":
    main()
