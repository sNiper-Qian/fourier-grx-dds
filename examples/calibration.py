from robot_control.state_machine import DDSPipeline
import json
import os
import argparse
from omegaconf import OmegaConf

class Calibrator:
    def __init__(self, config_path):
        self.config = OmegaConf.load(config_path)
        joints = self.config["joints"]
        self.connector = DDSPipeline(
            joints, self.config["encoders"], self.config["imu"], self.config["frequency"]
        )

    def init_encoders(self):
        encoders_state, integrality = self.connector.get_encoders_state()
        assert integrality, f"Error: Can not fetch the whole encoders_state."

        encoders_state_path = self.config["encoders_state_path"]
        assert os.path.exists(encoders_state_path), f"Not found encoders state file[{encoders_state_path}]. Calibration data must be exist."

        print(f"Load encoders state from {encoders_state_path}")
        self.encoders_state = OmegaConf.load(encoders_state_path)

        if integrality:
            print(f"Save encoders calibration state to file {encoders_state_path}.")
            for name in encoders_state:
                angle = encoders_state[name].angle
                self.encoders_state[name]["calibration_pose"] = angle

            OmegaConf.save(self.encoders_state, encoders_state_path)

    def start(self):
        self.init_encoders()
        
    def stop(self):
        self.connector.destroy()

def main(config_path):
    """
    Calibration process.
    """
    calibrator = Calibrator(config_path)
    calibrator.start()
    calibrator.stop()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file")
    args = parser.parse_args()
    main(args.config)
