import json
import os
import time
from datetime import datetime

OUT_DIR = "/home/gianfi/KvaserCar_outdoor_test/test_behaviour/"


class TestExpLog:
    def __init__(self, vehicle_log, add_debug, termination_cause):
        self.vehicle_log = vehicle_log
        self.term_cause = termination_cause
        self.add_debug = add_debug
        self.log_dir = self.create_log_dir()
        self.log_name = self.create_log_file_name()

    def create_log_dir(self):
        # Base folder to store rosbag files
        bag_base_dir = os.path.expanduser(OUT_DIR)
        # Generate date-based subfolder (e.g., 0513)
        date_str = datetime.now().strftime('%m%d')
        log_subdir = os.path.join(bag_base_dir, date_str)
        # Ensure directory exists
        os.makedirs(log_subdir, exist_ok=True)

        return log_subdir
    
    def create_log_file_name(self):
        now = datetime.now()
        name = (
            f"{now.day:02d}"
            f"{now.hour:02d}"
            f"{now.minute:02d}"
            f"{now.second:02d}"
            f"{now.microsecond:06d}"
        )
        file_name = os.path.join(self.log_dir, name + ".json")

        return file_name


    def write_to_file(self):
        timestamp = time.time()

        data = {"exp_name":timestamp,
                "term_cause":self.term_cause,
                "ego": self.vehicle_log,
                "debug": self.add_debug}

        with open(self.log_name, 'w') as f:
            json.dump(data, f)