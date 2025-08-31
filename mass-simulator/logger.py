import os
from agent import Agent
import json


class Logger():
    """ Logger class for the MASS simulator
    """

    def __init__(self,
                 save_dir):
        self.save_path = self._get_next_file_name(save_dir)

        # Initialise the log dictionary
        self.log_dict = []
        self.n = -1

    def _get_next_file_name(self,
                            save_dir):
        # create the directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        # find the next file name
        i = 0
        while os.path.exists(os.path.join(save_dir,
                                          f"log_{i}.json")):
            i += 1
        return os.path.join(save_dir, f"log_{i}.json")

    def add_time(self,
                 t):
        # add the time to the log
        self.log_dict[self.n]['time'] = t

    def add_speed_req(self,
                      speed_req):
        self.log_dict[self.n]['speed_req'] = speed_req

    def add_course_req(self,
                       course_req):
        self.log_dict[self.n]['course_req'] = course_req

    def add_waypoint_req(self,
                         wp_req):
        self.log_dict[self.n]['wp_req'] = wp_req

    def log_vessel(self,
                   vessel: Agent):
        # add the agent position
        self.log_dict[self.n]['vessels'].append({'vessel_id': vessel.vessel_id,
                                                 'xy': vessel.xy,
                                                 'course_deg': vessel.course_deg,
                                                 'speed_kn': vessel.speed_kn,
                                                 'speed_mps': vessel.speed_mps,
                                                 'waypoints': vessel.waypoints})

    def next_step(self):
        self.log_dict.append({"vessels": []})
        self.n += 1

    def add_termination_reason(self,
                               termination_reason: str):
        self.log_dict[self.n]['termination_reason'] = termination_reason

    def add_performance_score(self,
                              perf_summary):
        self.log_dict[self.n]['performance_summary'] = perf_summary

    def save_log_file(self):
        # write the test dictionary to the file
        with open(self.save_path, 'w') as f:
            # write the dict to a json file
            json.dump(self.log_dict,
                      f,
                      indent=4)

    def reset(self):
        pass
