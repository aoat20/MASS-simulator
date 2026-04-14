import numpy as np
from logic_bridge.bin_constants import bin_constants
from mass_simulator import general, agent
import matplotlib.pyplot as plt


class logic_bridge():

    def __init__(self):
        self.log = [[]]
        self.n = 0

    def add_to_log(self, **kwargs):
        log_entry = []

        if "time_s" in kwargs:
            self.log[self.n].append(f"clock({kwargs['time_s']})")

        if "course_deg" in kwargs:
            v_id = kwargs['vessel']
            course_seg = self.bearing_to_segment(kwargs['course_deg'])
            log_entry = f"course({v_id},{course_seg})"

        if "speed_kn" in kwargs:
            v_id = kwargs['vessel']
            speed_kn = round(kwargs['speed_kn'])
            log_entry = f"speed({v_id}, {speed_kn})"

        if "waypoint" in kwargs:
            v_id = kwargs['vessel']
            ref_id = kwargs['reference']
            ref_xy = kwargs['ref_xy']
            wp = kwargs['waypoint']
            wp_n = kwargs['wp_n']
            brg = general.compute_bearing(ref_xy,
                                          wp)
            brg_bin = self.bearing_to_segment(brg-kwargs['course_deg'])
            d = general.compute_distance(wp,
                                         ref_xy)
            d_bin = self.range_to_bins(d)
            log_entry = f"add_waypoint_bin({v_id},waypoint{wp_n},{ref_id},{brg_bin},{d_bin})"

        if "bearing_deg" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            segment = self.bearing_to_segment(kwargs['bearing_deg'])
            log_entry = f"bearing({v_id1},{v_id2},{segment})"

        if "range_m" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            range_bin = self.range_to_bins(kwargs['range_m'])
            log_entry = f"distance({v_id1},{v_id2},{range_bin})"

        if "tcpa_s" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            tcpa_bin = self.tcpa_to_bin(kwargs['tcpa_s'])
            clo_or_op = self.closing_or_opening(kwargs['tcpa_s'])
            log_entry = f"tcpa({v_id1},{v_id2},{tcpa_bin})" \
                        + f""
        if "cpa_m" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            cpa_bin = self.cpa_to_bin(kwargs['cpa_m'])
            log_entry = f"cpa({v_id1},{v_id2},{cpa_bin})"

        if "action" in kwargs:
            pass

        if log_entry:
            self.log[self.n].append(log_entry)

    def add_obs_to_log(self, obs):
        # Add time
        self.add_to_log(time_s=obs['time_s'])

        v_n = 0
        # Go through each vessel and add each observation
        for key1, value1 in obs['vessels'].items():
            self.add_to_log(vessel=key1,
                            course_deg=value1.course_deg)
            self.add_to_log(vessel=key1,
                            speed_kn=value1.speed_kn)

            # Go through each other vessel
            for key2, value2 in value1.other_vessels.items():
                # Add bearings
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                bearing_deg=value2.bearing_deg)
                # Add the range
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                range_m=value2.range_m)
                # Add the cpa
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                cpa_m=value2.cpa_m)
                # Add the tcpa
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                tcpa_s=value2.tcpa_s)

                if value1.waypoints_updated == 1:
                    n = 0
                    for wp in value1.waypoints[1:]:
                        if not np.isnan(wp[0]):
                            self.add_to_log(vessel=key1,
                                            reference=key1,
                                            ref_xy=obs["vessels"][key1].xy,
                                            waypoint=wp,
                                            wp_n=f"{v_n}_{n}",
                                            course_deg=obs["vessels"][key1].course_deg)
                            self.add_to_log(vessel=key1,
                                            reference=key2,
                                            ref_xy=obs["vessels"][key2].xy,
                                            waypoint=wp,
                                            wp_n=f"{v_n}_{n}",
                                            course_deg=obs["vessels"][key2].course_deg)

                        n += 1
            v_n += 1

    def next_step(self):
        self.n += 1
        self.log.append([])
        return self.log[self.n-1]

    def range_to_bins(self, range_m):
        # bin_sel = [r[0] for r in bin_constants.RANGE_BINS
        #            if (range_m > r[1] and range_m < r[2])][0]
        bin_sel = int(np.ceil(np.interp(range_m,
                                        np.linspace(0, 182.5*50, 50),
                                        np.arange(0, 50),
                                        right=50)))
        return bin_sel

    def closing_or_opening(self, tcpa_s):
        pass

    def tcpa_to_bin(self, tcpa_s):
        bin_sel = [r[0] for r in bin_constants.TCPA_BINS
                   if (tcpa_s > r[1] and tcpa_s < r[2])][0]
        return bin_sel

    def cpa_to_bin(self, cpm_m):
        bin_sel = [r[0] for r in bin_constants.CPA_BINS
                   if (cpm_m > r[1] and cpm_m < r[2])][0]
        return bin_sel

    def bearing_to_segment(self, bearing_deg):
        brg_tmp = (bearing_deg) % 360
        return int(np.ceil(np.interp(brg_tmp,
                                     np.linspace(5.625, 354.375, 32),
                                     np.arange(0, 32),
                                     right=0)))

    def bearing_to_sector(self, bearing_deg):
        seg = self.bearing_to_segment(bearing_deg=bearing_deg)
        brg_sect = [s[0] for s in bin_constants.SECTOR
                    if seg >= s[1] and seg <= s[2]][0]
        return brg_sect

    def segment_to_bearing(self, segment):
        brg_upper = np.interp(segment,
                              np.arange(0, 32),
                              np.linspace(5.625, 354.375, 32))
        brg_lower = (brg_upper - 11.25) % 360
        return brg_lower, brg_upper

    def remove_duplicates(self, log):
        # Remove duplicate logs until they change
        log_tmp = [log[0]]
        for n in range(len(log)-1):
            log_entry = []
            # Check if each entry is in the previous timestep
            for entry in log[(n+1)]:
                if entry not in log[n]:
                    log_entry.append(entry)

            # Check if there's anything missing in the next timestep
            for entry in log[n]:
                if entry not in log[n+1] \
                        and any(x in entry for x in ["arc_overtaking"]):
                    log_entry.append(f";{entry}")

            # If there more than just the clock add the entry
            if len(log_entry) > 1 and any("clock" in x for x in log_entry):
                log_tmp.append(log_entry)
            if log[n-1][1:] != [] and log[n][1:] == []:
                log_tmp.append(log[n])
        return log_tmp

    def add_wp_area(self, xy0, course_deg, r1, r2, theta_1, theta_2):
        mg_flat = np.concatenate([[self.xy_mg[0].flatten()],
                                 [self.xy_mg[1].flatten()]],
                                 axis=0)
        xy0_np = np.array(xy0).reshape(2, 1)

        x_diff = xy0[0]-mg_flat[0, :]
        y_diff = xy0[1]-mg_flat[1, :]

        d_tmp = np.array(np.sqrt(x_diff**2 + y_diff**2))
        theta_tmp = (
            np.array(np.rad2deg(np.atan2(x_diff, y_diff))-180-course_deg)) % 360

        theta_tmp2 = (theta_tmp - theta_1) % 360
        theta1_tmp = 0
        theta2_tmp = (theta_2 - theta_1) % 360

        d_sect = (d_tmp > r1) & (d_tmp < r2)
        th_sect = (theta_tmp2 > theta1_tmp) & (theta_tmp2 < theta2_tmp)

        area_mask = d_sect & th_sect

        # plt.figure
        # plt.imshow(theta_tmp.reshape(self.xy_mg[0].shape),
        #            extent=[np.min(self.xy_mg[0]), np.max(self.xy_mg[0]),
        #                    np.min(self.xy_mg[1]), np.max(self.xy_mg[1])],
        #            origin='lower')
        # plt.scatter(xy0[0], xy0[1])
        # plt.show()
        self.wp_area = self.wp_area & area_mask

    def waypoint_logic_to_coordinates(self,
                                      waypoint_logic: list,
                                      vessels: list[agent.Agent]):

        # Compute xy_lims
        lim_span = 10000
        xy = []
        for v in vessels.values():
            xy.append(v.xy)
        xy_min = np.min(np.array(xy), 0)
        xy_max = np.max(np.array(xy), 0)
        xy_lims = [xy_min[0]-lim_span, xy_max[0]+lim_span,
                   xy_min[1]-lim_span, xy_max[1]+lim_span]

        # Set up the space
        res = 100
        self.xy_mg = np.meshgrid(np.arange(xy_lims[0], xy_lims[1], res),
                                 np.arange(xy_lims[2], xy_lims[3], res))
        self.wp_area = np.ones(self.xy_mg[0].flatten().shape, dtype=bool)

        wp: str
        for wp in waypoint_logic:
            wp_list = wp.strip("add_waypoint").strip("()").split(",")
            print(wp_list)
            v_0 = wp_list[0]
            v_1 = wp_list[2]
            v_sector = wp_list[3]
            v_range = wp_list[4]

            sector_lims = [x[1:] for x in bin_constants.SECTOR
                           if x[0] == v_sector][0]
            s_1_1, s_1_2 = self.segment_to_bearing(sector_lims[0])
            s_2_1, s_2_2 = self.segment_to_bearing(sector_lims[1])

            range_lims = [x[1:] for x in bin_constants.RANGE_BINS
                          if x[0] == v_range][0]

            self.add_wp_area(vessels[v_1].xy,
                             vessels[v_1].course_deg,
                             range_lims[0],
                             range_lims[1],
                             s_1_1,
                             s_2_2)
        # plt.figure
        # plt.imshow(self.wp_area.reshape(self.xy_mg[0].shape),
        #            extent=[np.min(self.xy_mg[0]), np.max(self.xy_mg[0]),
        #                    np.min(self.xy_mg[1]), np.max(self.xy_mg[1])],
        #            origin='lower')
        # plt.show()
        x_out = np.mean(self.xy_mg[0].flatten()[self.wp_area])
        y_out = np.mean(self.xy_mg[1].flatten()[self.wp_area])

        return [x_out, y_out]

    def output_to_txt(self,
                      save_loc="log.txt",
                      log=[]):

        if log == []:
            log_tmp = self.remove_duplicates(log=self.log)
        else:
            log_tmp = self.remove_duplicates(log=log)

        if save_loc[-4:] != ".txt":
            save_loc = save_loc+".txt"

        # Save logs to txt
        with open(save_loc, 'a') as output:
            for entry in log_tmp:
                entry_str = ", ".join(str(x) for x in entry) + "."
                output.write(entry_str+"\n")
