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
            log_dict = json.load(f)

        self.setup = log_dict['setup']
        self.log_dict = log_dict['log']
        self.t_max = self.log_dict[-1]['time']
        self.N = len(self.log_dict)
        self.n = 0

    def get_setup(self):
        return self.setup

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

    def next_step(self):
        self.n += 1

    def set_t(self, t):
        t_vals = [t['time'] for t in self.log_dict]
        n = t_vals.index(t)
        self.n = np.clip(n, 1, self.N-1)

    def get_current_step(self):
        n = self.n
        t = self.log_dict[n]['time']

        # get the initial vessels details
        vessels = {}
        for v in self.log_dict[n]['vessels']:
            v_temp = Agent(vessel_id=v['vessel_id'],
                           xy_init=v['xy'],
                           speed_kn=v['speed_kn'],
                           speed_mps=v['speed_mps'],
                           waypoints=v['waypoints'])
            v_temp.course_deg = v['course_deg']
            for d_n in self.log_dict[0:n]:
                v_temp.xy_hist.append([v_n['xy']for v_n in d_n['vessels']
                                       if v_n['vessel_id'] == v['vessel_id']][0])
            v_temp.xy_hist = v_temp.xy_hist[1:]
            vessels[v['vessel_id']] = v_temp
        return t, vessels
