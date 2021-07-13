import os.path
import numpy as np

from robot_properties_solo.config import Solo12Config

from solo.solo12_hwp_cpp import Solo12HWP

if __name__ == "__main__":
    solo12 = Solo12HWP()

    solo12.initialize(os.path.join(
        os.path.dirname(Solo12Config.dgm_yaml_path), 'dgm_parameters_solo12_nyu.yaml'))

    solo12.run()

    input("Press enter to start calibration.")

    solo12.calibrate_from_yaml()
