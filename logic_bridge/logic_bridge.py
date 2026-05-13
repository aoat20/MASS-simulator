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
            v1_id = kwargs['v1_id']
            v2_id = kwargs['v2_id']
            v1_xy = kwargs['vessel1'].xy
            v2_xy = kwargs['vessel2'].xy
            v1_course = kwargs['vessel1'].course_deg
            v2_course = kwargs['vessel2'].course_deg
            v2_course_rad = np.deg2rad(v2_course)
            v1_speed_mps = kwargs['vessel1'].speed_mps
            v2_speed_mps = kwargs['vessel2'].speed_mps
            goal_wp = kwargs['vessel1'].goal_waypoint
            wp = kwargs['waypoint']
            # Compute DCPA and TCPA for the first leg of the diversion
            course0_new = 90-np.rad2deg(np.atan2(wp[1]-v1_xy[1],
                                                 wp[0]-v1_xy[0]))
            cpa1_m, _, tcpa1_s = general.compute_cpa(v1_xy, course0_new, v1_speed_mps,
                                                     v2_xy, v2_course, v2_speed_mps)
            tcpa1_bin = self.tcpa_to_bin(tcpa1_s)
            cpa1_bin = self.cpa_to_bin(cpa1_m)

            # Compute DCPA and TCPA for the second leg of the diversion
            # Distance from agent pos to wp
            r = general.compute_distance(v1_xy, wp)
            # Travel time to wp
            travel_time = r/v1_speed_mps
            # Heading from waypoint to resumption point
            course1_new = 90-np.atan2(goal_wp[1]-wp[1],
                                      goal_wp[0]-wp[0])
            v2_xy_new = v2_xy + v2_speed_mps*travel_time*np.array([np.sin(v2_course_rad),
                                                                   np.cos(v2_course_rad)])
            cpa2_m, _, tcpa2_s = general.compute_cpa(wp, course1_new, v1_speed_mps,
                                                     v2_xy_new, v2_course, v2_speed_mps)
            cpa2_bin = self.cpa_to_bin(cpa2_m)
            tcpa2_bin = self.tcpa_to_bin(tcpa2_s)

            turn_sign = np.sign(v1_course-course0_new)
            if turn_sign == 1:
                turn_direction = "port"
            else:
                turn_direction = "starboard"
            turn_mag_bin = self.turn_magnitude_to_bin(
                np.abs(v1_course-course0_new))
            log_entry = f"add_waypoint({v1_id},{v2_id}," \
                        + f"{cpa1_bin},{tcpa1_bin}," \
                        + f"{cpa2_bin},{tcpa2_bin}," \
                        + f"{turn_direction},{turn_mag_bin})"

        if "bearing_deg" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            sector, arc_overtaking = self.bearing_to_sector(
                kwargs['bearing_deg'])
            log_entry = f"sector({v_id1},{v_id2},{sector})"
            if arc_overtaking:
                self.log[self.n].append(log_entry)

                log_entry = f"arc_overtaking({v_id2},{v_id1})"

        if "range_m" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            range_bin = self.range_to_bins(kwargs['range_m'])
            log_entry = f"range({v_id1},{v_id2},{range_bin})"

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
            log_entry = f"dcpa({v_id1},{v_id2},{cpa_bin})"

        if "action" in kwargs:
            pass

        if "resume" in kwargs:
            log_entry = f"resume({kwargs['vessel']})"

        if log_entry:
            self.log[self.n].append(log_entry)

    def add_obs_to_log(self, obs):
        # If the vessel is turning don't add to log

        # Add time
        self.add_to_log(time_s=obs['time_s'])

        v_n = 0
        # Go through each vessel and add each observation
        for key1, value1 in obs['vessels'].items():
            # Go through each other vessel
            for key2, value2 in value1.other_vessels.items():

                if value1.resuming_mission == True:
                    self.add_to_log(resume=True,
                                    vessel=key1)
                    value1.resuming_mission = False
                elif value1.waypoints_updated == 1:
                    n = 0
                    for wp in value1.waypoints[1:]:
                        if not np.isnan(wp[0]):
                            self.add_to_log(v1_id=key1,
                                            v2_id=key2,
                                            vessel1=value1,
                                            vessel2=value2,
                                            waypoint=wp)

                        n += 1

                # Add bearings
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                bearing_deg=value2.bearing_deg)
                # Add the range
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                range_m=value2.range_m)

                # If either vessel is turning don't record any of the other stuff
                if value1.turning or obs['vessels'][key2].turning:
                    continue

                # Add the cpa
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                cpa_m=value2.cpa_m)
                # Add the tcpa
                self.add_to_log(vessel1=key1,
                                vessel2=key2,
                                tcpa_s=value2.tcpa_s)

            v_n += 1

    def next_step(self):
        self.n += 1
        self.log.append([])
        return self.log[self.n-1]

    def range_to_bins(self, range_m):
        bin_sel = [r[0] for r in bin_constants.RANGE_BINS
                   if (range_m > r[1] and range_m < r[2])][0]
        # bin_sel = int(np.ceil(np.interp(range_m,
        #                                 np.linspace(0, 182.5*50, 50),
        #                                 np.arange(0, 50),
        #                                 right=50)))
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
        if seg >= 10 and seg <= 22:
            is_arc_overtaking = True
        else:
            is_arc_overtaking = False
        return brg_sect, is_arc_overtaking

    def segment_to_bearing(self, segment):
        brg_upper = np.interp(segment,
                              np.arange(0, 32),
                              np.linspace(5.625, 354.375, 32))
        brg_lower = (brg_upper - 11.25) % 360
        return brg_lower, brg_upper

    def turn_magnitude_to_bin(self, turn_magntiude):
        bin_sel = [r[0] for r in bin_constants.TURN_MAGNITUDES
                   if (turn_magntiude > r[1] and turn_magntiude < r[2])][0]
        return bin_sel

    def remove_duplicates(self, log):

        log_init = [n for n in log[0] if "add_waypoint" not in n]
        # Remove duplicate logs until they change
        log_tmp = [log_init]
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
                    log_entry.append(f"!{entry}")

            # If there more than just the clock add the entry
            if len(log_entry) > 1 and any("clock" in x for x in log_entry):
                log_tmp.append(log_entry)
            if log[n-1][1:] != [] and log[n][1:] == []:
                log_tmp.append(log[n])

        # Remove reciprocal relationships
        log_out = []
        for log_entry in log_tmp:
            log_entry_out = []
            for n in log_entry:
                n_comparison = [sorted(n) == sorted(n2)
                                for n2 in log_entry_out]
                if not any(n_comparison):
                    log_entry_out.append(n)
            log_out.append(log_entry_out)
        return log_out

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
        x_out = np.mean(self.xy_mg[0].flatten()[self.wp_area]).tolist()
        y_out = np.mean(self.xy_mg[1].flatten()[self.wp_area]).tolist()

        return v_0, [x_out, y_out]

    def cpa_waypoint_logic_to_coordinates(self,
                                          waypoint_logic: list[str],
                                          vessels: list[agent.Agent]):
        pass

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
