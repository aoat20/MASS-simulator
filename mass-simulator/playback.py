import json
import numpy as np
from agent import Agent


class Playback():
    """ Playback class for the MASS simulator
    """

    def __init__(self,
                 log_file):
        # load the log file
        with open(log_file, 'r') as f:
            self.log_dict = json.load(f)

    def get_time_req(self,
                     n):
        # get the n step
        t = self.log_dict[n+1]['time'] - self.log_dict[n]['time']
        speed_req = None
        course_req = None
        if 'speed_req' in self.log_dict[n]:
            speed_req = self.log_dict[n]['speed_req']
        if 'course_req' in self.log_dict[n]:
            course_req = self.log_dict[n]['course_req']

        return t, speed_req, course_req

    def get_xy_lims(self):
        # get the xy limits
        xy_lims = [np.inf, -np.inf, np.inf, -np.inf]
        for n in self.log_dict:
            for v in n['vessels']:
                xy_lims[0] = min(xy_lims[0], v['xy'][0])
                xy_lims[1] = max(xy_lims[1], v['xy'][0])
                xy_lims[2] = min(xy_lims[2], v['xy'][1])
                xy_lims[3] = max(xy_lims[3], v['xy'][1])

        return xy_lims

    def get_vessels(self, n):
        # get the initial vessels details
        vessels = []
        for v in self.log_dict[n]['vessels']:
            v_temp = Agent(vessel_id=v['vessel_id'],
                           xy_init=v['xy'],
                           speed_kn=v['speed_kn'],
                           speed_mps=v['speed_mps'],
                           waypoints=v['waypoints'])
            for d_n in self.log_dict[0:n]:
                v_temp.xy_hist.append(v_temp.xy_hist,
                                      [v_n['xy']for v_n in d_n['vessels']
                                       if v_n['vessel_id'] == v['vessel_id']])
            v_temp.xy_hist = v_temp.xy_hist[1:]
            vessels.append(v_temp)
        return vessels
