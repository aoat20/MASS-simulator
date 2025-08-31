from agent import Agent
from world import World
from plotter import Plotter
import os
import json
from general import *
import pyproj
from time import time


class MASSsim():
    def __init__(self,
                 scenario: int | str = "",
                 mode: str = "manual",
                 plotter: bool = True,
                 log_dir: str = "logs/"):
        self._playspeed = 100
        self.mission_finished = False
        self.termination_reason = ""

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

        params, self._vessels, xy_lim = self._load_scene(scen_pth)
        self._world = World(params['t_step'])
        self._plotter = Plotter(self._vessels,
                                xy_lim)

        self.plotter_loop()

    def plotter_loop(self):
        t_0 = time()
        while self._plotter.is_plotter_running():
            # do stuff
            if time() - t_0 >= self._world.t_step/(self._plotter.playspeed):
                self.next_step()
                t_0 = time()
            self._update_plotter()

    def next_step(self):
        self._world.next_step()

        v: Agent
        for v in self._vessels.values():
            v.next_step(self._world.t_step)
            v.update_other_vessels(self._vessels)

    def get_obs(self):
        pass

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

    def _load_scene(self,
                    config_file):
        # Set up utm conversion
        p = pyproj.Proj(proj='utm',
                        zone=30,
                        ellps='WGS84',
                        preserve_units=False)

        # Open and load the config file
        f = open(config_file)
        conf = json.load(f)

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
        f.close()

        xy_lim_tmp_np = np.array(xy_lim_tmp)
        # Get limits of travel for vessel travel
        xy_lim = [xy_lim_tmp_np[:, 0].min(),
                  xy_lim_tmp_np[:, 0].max(),
                  xy_lim_tmp_np[:, 1].min(),
                  xy_lim_tmp_np[:, 1].max()]
        return params, vessels, xy_lim


def main():
    mass_sim = MASSsim(4,
                       'manual',
                       True)


if __name__ == '__main__':
    main()
