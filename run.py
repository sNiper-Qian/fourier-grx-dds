# run.py
import hydra
from omegaconf import DictConfig

# Import your Controller class here
from robot_control.controller import BaseController

@hydra.main(config_path="config/controller", config_name="default")
def main(cfg: DictConfig) -> None:
    # Initialize your controller with the loaded configurations
    controller = BaseController(cfg)

if __name__ == "__main__":
    main()
