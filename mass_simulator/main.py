from mass_simulator.agent import Agent
from mass_simulator.world import World
from mass_simulator.plotter import Plotter
from mass_simulator.logger import Logger
from mass_simulator.playback import Playback
import os
import json
from mass_simulator.general import *
import pyproj
from time import time


class MASSsim():
    def __init__(self,
                 scenario: int | str = "",
                 mode: str = "manual",
                 plotter: bool = True,
                 log_dir: str = "logs/",
                 log_file: int | str = ""):

        self._episode_finished = False
        self.termination_reason = ""

        if mode == "manual":
            self._start_manual(scenario=scenario,
                               log_dir=log_dir)
        elif mode == "test":
            self._start_test(scenario=scenario,
                             log_dir=log_dir,
                             plotter=plotter)
        elif mode == "playback":
            self._start_playback(log_file=log_file)

    def _start_manual(self, scenario, log_dir):
        conf = self._get_scenario(scenario=scenario)
        scen_conf, params, self._vessels, xy_lim = self._setup_scene(conf)
        self._world = World(params['t_step'])
        self._logger = Logger(log_dir, scen_conf)
        self._plotter = Plotter(self._vessels,
                                xy_lim,
                                control=True)
        self._manual_plotter_loop()

    def _start_test(self, scenario, log_dir, plotter):
        conf = self._get_scenario(scenario=scenario)
        scen_conf, params, self._vessels, xy_lim = self._setup_scene(conf)
        self._world = World(params['t_step'])
        self._logger = Logger(log_dir, scen_conf)
        if plotter:
            self._plotter = Plotter(self._vessels,
                                    xy_lim,
                                    control=True)

    def _start_playback(self,
                        log_file):
        log_path = self._get_log_path(log_file=log_file)
        self._playback = Playback(log_file=log_path)
        conf = self._playback.get_setup()
        scen_conf, params, self._vessels, xy_lim = self._setup_scene(conf)
        self._world = World(params['t_step'])
        self._plotter = Plotter(self._vessels,
                                xy_lim,
                                control=True)
        self._plotter.add_time_scrubber(self._playback.t_max)
        self._playback_plotter_loop()

    def _manual_plotter_loop(self):
        t_0 = time()
        while self._plotter.is_plotter_running():
            play = self._plotter.play
            if play:
                if self.is_episode_running():
                    if time() - t_0 >= self._world.t_step/(self._plotter.playspeed):
                        self._manualtest_next_step()
                        t_0 = time()
                else:
                    self._plotter.set_play(False)
            self._update_plotter()

        if hasattr(self, "_logger"):
            self._logger.save_log_file()

    def _playback_plotter_loop(self):
        t_0 = time()
        while self._plotter.is_plotter_running():
            # do stuff
            play = self._plotter.play
            t = self._plotter.get_time()
            t_n = np.clip(t-1, 0, self._playback.N)
            if t_n != self._playback.n and self._plotter.play:
                self._plotter.set_play(False)

            if play:
                if self.is_episode_running():
                    if time() - t_0 > self._world.t_step/(self._plotter.playspeed):
                        self._playback_next_step()
                        t_0 = time()
                else:
                    self._plotter.set_play(False)
            else:
                self._playback_n_step()

            self._update_plotter()

    def _playback_n_step(self):
        v: Agent
        t = self._plotter.get_time()
        self._plotter.set_time(t)
        self._playback.set_t(t)
        self._world.set_t(t)
        t, self._vessels = self._playback.get_current_step()
        for v in self._vessels.values():
            v.update_other_vessels(self._vessels)

    def _episode_finish_check(self):
        # Check if all vessels have reached their final waypoint
        v: Agent
        reached = []
        for v in self._vessels.values():
            reached.append(v._final_waypoint_reached)
        if all(reached):
            self._episode_finished = True

        # If it's in playback mode, check if it's hit the final time step
        if hasattr(self, '_playback'):
            if self._playback.n == self._playback.N-1:
                self._episode_finished = True
            else:
                self._episode_finished = False

    def is_episode_running(self):
        self._episode_finish_check()
        if hasattr(self, "_plotter"):
            if not self._plotter.is_plotter_running():
                self._plotter.tidy_up()
        return not self._episode_finished

    def _manualtest_next_step(self):
        self._world.next_step()
        if hasattr(self, '_logger'):
            self._logger.next_step(self._world.t_elapsed)

        v: Agent
        for v in self._vessels.values():
            v.next_step(self._world.t_step)
            if hasattr(self, '_logger'):
                self._logger.log_vessel(v)
            v.update_other_vessels(self._vessels)

    def _playback_next_step(self):
        v: Agent
        self._playback.next_step()
        t, self._vessels = self._playback.get_current_step()
        self._plotter.set_time(t)
        for v in self._vessels.values():
            v.update_other_vessels(self._vessels)

    def next_step(self):
        self._episode_finish_check()
        if not self._episode_finished:
            self._manualtest_next_step()
        if hasattr(self, '_plotter'):
            self._update_plotter()

    def get_obs(self):
        obs_dict = {}
        obs_dict['time_s'] = self._world.t_elapsed
        obs_dict['vessels'] = self._vessels
        return obs_dict

    def set_waypoints(self, vessel_id, waypoints_utm):
        self._vessels[vessel_id].update_waypoints(waypoints_utm)

    def _update_plotter(self):
        v: Agent
        self._plotter.update_time(self._world.t_elapsed)
        self._plotter.update_vessels(vessels=self._vessels)
        wp = self._plotter.get_waypoint_updates()
        if wp:
            for v in wp:
                self.set_waypoints(v, wp[v])

    def _get_scenario(self, scenario):
        if scenario == "":
            raise ValueError("scenario_n argument not set." +
                             "Must either be the number of the " +
                             "desired scenario or a str containing" +
                             " the path to the custom scenario json file.")
        elif isinstance(scenario, int):
            conf_loc = os.path.join(os.path.dirname(__file__), '..')
            scen_pth = os.path.join(conf_loc, 'scenarios',
                                    f"scenario{scenario}.json")
        elif isinstance(scenario, str):
            scen_pth = scenario

        # Open and load the config file
        with open(scen_pth) as f:
            conf = json.load(f)
        return conf

    def _setup_scene(self, conf):
        # Set up utm conversion
        p = pyproj.Proj(proj='utm',
                        zone=30,
                        ellps='WGS84',
                        preserve_units=False)

        params = conf['params']

        # initialise the other vessels
        vessels = {}
        xy_lim_tmp = []
        for v in conf['vessel_details']:
            # Get the vessel details
            way_points = []
            for wp in v["waypoints"]:
                wp_temp = list(p(convert_dms_to_dec(wp[1]),
                                 convert_dms_to_dec(wp[0])))
                # If there's a speed change, add an extra element
                if len(wp) == 3:
                    wp_temp.append(wp[2])
                way_points.append(wp_temp)
                # get waypoints for the computation of the limits
                xy_lim_tmp.append(wp_temp[0:2])

            if "speed_mps" in v:
                speed_mps = v['speed_mps']
            else:
                speed_mps = 0.5144*v["speed_kn"]
            course = compute_bearing(way_points[0],
                                     way_points[1])

            vessels[v['vessel']] = Agent(vessel_id=v['vessel'],
                                         xy_init=way_points[0],
                                         speed_mps=speed_mps,
                                         waypoints=way_points)

        xy_lim_tmp_np = np.array(xy_lim_tmp)
        # Get limits of travel for vessel travel
        xy_lim = [xy_lim_tmp_np[:, 0].min(),
                  xy_lim_tmp_np[:, 0].max(),
                  xy_lim_tmp_np[:, 1].min(),
                  xy_lim_tmp_np[:, 1].max()]
        return conf, params, vessels, xy_lim

    def _get_log_path(self, log_file):
        if log_file == "":
            raise ValueError("scenario_n argument not set." +
                             "Must either be the number of the " +
                             "desired scenario or a str containing" +
                             " the path to the custom scenario json file.")
        elif isinstance(log_file, int):
            conf_loc = os.path.join(os.path.dirname(__file__), '..')
            log_path = os.path.join(conf_loc, 'logs',
                                    f"log_{log_file}.json")
        elif isinstance(log_file, str):
            log_path = log_file

        return log_path


def main():
    mass_sim = MASSsim(scenario=3,
                       mode='manual',
                       plotter=True)
    mass_sim = MASSsim(log_file=37,
                       mode='playback',
                       plotter=True)
    mass_sim = MASSsim(scenario=3,
                       mode='test',
                       plotter=False)
    while mass_sim.is_episode_running():
        mass_sim.next_step()
        obs = mass_sim.get_obs()
        if obs['time_s'] == 200:
            mass_sim.set_waypoints('agent',
                                   [[430000,
                                    5555000]])
    print('finished')


if __name__ == '__main__':
    main()
