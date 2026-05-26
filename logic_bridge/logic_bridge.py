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
            cpa2_m, _, tcpa2_s = general.compute_future_cpas(v1_xy, v1_speed_mps,
                                                             v2_xy, v2_course, v2_speed_mps,
                                                             wp, goal_wp)
            cpa2_bin = self.cpa_to_bin(cpa2_m)
            tcpa2_bin = self.tcpa_to_bin(tcpa2_s)

            cpa_side, cpa_end = general.compute_cpa_side_end(tcpa_s=tcpa1_s,
                                                             xy1=v1_xy,
                                                             course1=v1_course,
                                                             speed_mps1=v1_speed_mps,
                                                             xy2=v2_xy,
                                                             course2=v2_course,
                                                             speed_mps2=v2_speed_mps)

            turn_mag_bin = self.turn_magnitude_to_bin(
                np.abs(v1_course-course0_new))
            log_entry = f"add_waypoint({v1_id},{v2_id}," \
                        + f"{cpa1_bin},{tcpa1_bin}," \
                        + f"{cpa2_bin},{tcpa2_bin}," \
                        + f"{cpa_side},{cpa_end}," \
                        + f"{turn_mag_bin})"

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

    def cpa_to_bin(self, cpa_m):
        bin_sel = [r[0] for r in bin_constants.CPA_BINS
                   if (cpa_m > r[1] and cpa_m < r[2])][0]
        return bin_sel

    def bin_to_tcpa(self, tcpa_bin):
        tcpa_bin_lims = [x[1:3] for x in bin_constants.TCPA_BINS
                         if x[0] == tcpa_bin][0]
        return tcpa_bin_lims

    def bin_to_dcpa(self, dcpa_bin):
        dcpa_bin_lims = [x[1:3] for x in bin_constants.CPA_BINS
                         if x[0] == dcpa_bin][0]
        return dcpa_bin_lims

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

    def turn_magnitude_to_bin(self, turn_magnitude):
        turn_magnitude = turn_magnitude % 180
        bin_sel = [r[0] for r in bin_constants.TURN_MAGNITUDES
                   if (turn_magnitude >= r[1] and turn_magnitude < r[2])][0]
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
                    log_entry.append(f"not({entry})")

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
                if "range" in n \
                        or "dcpa" in n \
                        or "tcpa" in n:
                    if not any(n_comparison):
                        log_entry_out.append(n)
                else:
                    log_entry_out.append(n)
            log_out.append(log_entry_out)
        return log_out

    def add_wp_area_range_brg(self, xy0, course_deg, r1, r2, theta_1, theta_2):
        mg_flat = np.concatenate([[self.xy_mg[0].flatten()],
                                 [self.xy_mg[1].flatten()]],
                                 axis=0)

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

        return area_mask

    def add_wp_area_cpa(self,
                        xy_mg,
                        dcpa_bin1,
                        tcpa_bin1,
                        dcpa_bin2,
                        tcpa_bin2,
                        vessel: agent.Agent,
                        v2_id,
                        cpa_side,
                        cpa_end):

        # Get the minimum allowable dcpa and tcpa for diversion leg and resume leg
        dcpa1_lim = self.bin_to_dcpa(dcpa_bin=dcpa_bin1)
        tcpa1_lim = self.bin_to_tcpa(tcpa_bin=tcpa_bin1)
        dcpa2_lim = self.bin_to_dcpa(dcpa_bin=dcpa_bin2)
        tcpa2_lim = self.bin_to_tcpa(tcpa_bin=tcpa_bin2)

        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)

        xy1 = vessel.xy
        speed1 = vessel.speed_mps
        xy2 = vessel.other_vessels[v2_id].xy
        course2 = vessel.other_vessels[v2_id].course_deg
        speed2 = vessel.other_vessels[v2_id].speed_mps
        goal_wp = vessel.goal_waypoint

        mask_out = []
        cpa_side_end_mask = []
        for wp in mg_flat.T:
            course1 = general.compute_bearing(xy1,
                                              wp)

            dcpa1_m, _, tcpa1_s = general.compute_cpa(xy1=xy1,
                                                      course1=course1,
                                                      speed_mps1=speed1,
                                                      xy2=xy2,
                                                      course2=course2,
                                                      speed_mps2=speed2)
            dcpa2_m, _, tcpa2_s = general.compute_future_cpas(xy1=xy1,
                                                              speed_mps1=speed1,
                                                              xy2=xy2,
                                                              course2=course2,
                                                              speed_mps2=speed2,
                                                              wp=wp,
                                                              goal_wp=goal_wp)
            mask_out.append(dcpa1_m > dcpa1_lim[0]
                            and (tcpa1_s > tcpa1_lim[0] or tcpa1_s <= 0)
                            and dcpa2_m > dcpa2_lim[0]
                            and (tcpa2_s > tcpa2_lim[0] or tcpa2_s <= 0))
            cpa_side_tmp, cpa_end_tmp = general.compute_cpa_side_end(tcpa_s=tcpa1_s,
                                                                     xy1=xy1,
                                                                     course1=course1,
                                                                     speed_mps1=speed1,
                                                                     xy2=xy2,
                                                                     course2=course2,
                                                                     speed_mps2=speed2)

            cpa_side_end_mask.append((cpa_side == cpa_side_tmp or cpa_side == '')
                                     and (cpa_end == cpa_end_tmp or cpa_end == ''))
        return mask_out, cpa_side_end_mask

    def add_wp_area_turn(self,
                         xy_mg,
                         xy0,
                         course_deg,
                         turn_mag):
        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)

        x_diff = mg_flat[0, :]-xy0[0]
        y_diff = mg_flat[1, :]-xy0[1]

        theta_tmp = (
            np.array(90-np.rad2deg(np.atan2(y_diff, x_diff))-course_deg)) % 360

        mag_min = [x[1:3] for x in bin_constants.TURN_MAGNITUDES
                   if x[0] == turn_mag][0]

        mask_out = np.abs((theta_tmp+180) % 360 - 180) > mag_min[0]
        return mask_out

    def _setup_blank_wp_area(self,
                             xy_list,
                             lim_span=10000,
                             res=100):
        # Compute xy_lims
        xy_min = np.min(np.array(xy_list), 0)
        xy_max = np.max(np.array(xy_list), 0)
        xy_lims = [xy_min[0]-lim_span, xy_max[0]+lim_span,
                   xy_min[1]-lim_span, xy_max[1]+lim_span]

        # Set up the space
        xy_mg = np.meshgrid(np.arange(xy_lims[0], xy_lims[1], res),
                            np.arange(xy_lims[2], xy_lims[3], res))
        wp_area = np.ones(xy_mg[0].flatten().shape, dtype=bool)
        return wp_area, xy_mg

    def waypoint_logic_to_coordinates(self,
                                      waypoint_logic: list,
                                      vessels: list[agent.Agent]):

        xy_list = [v.xy for v in vessels.values()]
        wp_area, xy_mg = self._setup_blank_wp_area(xy_list)

        wp: str
        for wp in waypoint_logic:
            wp_list = wp.strip("add_waypoint").strip("()").split(",")
            v0 = vessels[wp_list[0]]
            v1_id = wp_list[1]
            div_dcpa = wp_list[2]
            div_tcpa = wp_list[3]
            res_dcpa = wp_list[4]
            res_tcpa = wp_list[5]
            cpa_side = wp_list[6]
            cpa_end = wp_list[7]
            turn_mag = wp_list[8]
            cpa_mask, side_end_mask = self.add_wp_area_cpa(xy_mg=xy_mg,
                                                           dcpa_bin1=div_dcpa,
                                                           tcpa_bin1=div_tcpa,
                                                           dcpa_bin2=res_dcpa,
                                                           tcpa_bin2=res_tcpa,
                                                           vessel=v0,
                                                           v2_id=v1_id,
                                                           cpa_side=cpa_side,
                                                           cpa_end=cpa_end)
            turn_mask = self.add_wp_area_turn(xy_mg=xy_mg,
                                              xy0=v0.xy,
                                              course_deg=v0.course_deg,
                                              turn_mag=turn_mag)

            # plt.figure
            # plt.subplot(311)
            # plt.imshow(np.array(cpa_mask).reshape(xy_mg[0].shape),
            #            extent=[np.min(xy_mg[0]), np.max(xy_mg[0]),
            #                    np.min(xy_mg[1]), np.max(xy_mg[1])],
            #            origin='lower')
            # plt.scatter(v0.xy[0], v0.xy[1])

            # plt.subplot(312)
            # plt.imshow(np.array(side_end_mask).reshape(xy_mg[0].shape),
            #            extent=[np.min(xy_mg[0]), np.max(xy_mg[0]),
            #                    np.min(xy_mg[1]), np.max(xy_mg[1])],
            #            origin='lower')
            # plt.scatter(v0.xy[0], v0.xy[1])
            # plt.subplot(313)
            # plt.imshow(np.array(turn_mask).reshape(xy_mg[0].shape),
            #            extent=[np.min(xy_mg[0]), np.max(xy_mg[0]),
            #                    np.min(xy_mg[1]), np.max(xy_mg[1])],
            #            origin='lower')
            # plt.scatter(v0.xy[0], v0.xy[1])

            wp_area = wp_area & cpa_mask & turn_mask & side_end_mask

        # get the closest allowable waypoint to the goal
        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)
        mg_flat_wp_area = mg_flat[:, wp_area]
        distances = np.linalg.norm(mg_flat_wp_area - np.reshape(v0.goal_waypoint,
                                                                [2, 1]), axis=0)

        if len(distances) != 0:
            min_i = np.argmin(distances)
            xy_out = mg_flat_wp_area[:, min_i].tolist()
        else:
            xy_out = []
            print("##################################################################\n"
                  + "################ WARNING! No valid wp location. ##################\n"
                  + "##################################################################")

        return v0.vessel_id, xy_out

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
