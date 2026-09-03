import numpy as np
from logic_bridge.bin_constants import bin_constants
from mass_simulator import general, agent
import matplotlib.pyplot as plt


class logic_bridge():

    def __init__(self,
                 turn_logic_flag=False):
        self.log = [[]]
        self.n = 0
        self.turn_logic_flag = turn_logic_flag

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

            roc1_bin = self.cpa_to_roc_bin(cpa1_m, tcpa1_s)

            # Compute DCPA and TCPA for the second leg of the diversion
            cpa2_m, _, tcpa2_s = general.compute_future_cpas(v1_xy, v1_speed_mps,
                                                             v2_xy, v2_course, v2_speed_mps,
                                                             wp, goal_wp)

            roc2_bin = self.cpa_to_roc_bin(cpa2_m, tcpa2_s)

            cpa_side, cpa_end = general.compute_cpa_side_end(tcpa_s=tcpa1_s,
                                                             xy1=v1_xy,
                                                             course1=course0_new,
                                                             speed_mps1=v1_speed_mps,
                                                             xy2=v2_xy,
                                                             course2=v2_course,
                                                             speed_mps2=v2_speed_mps)

            turn_mag_bin = self.turn_magnitude_to_bin(
                np.abs(v1_course-course0_new))
            # log_entry = f"waypoint({v1_id},{v2_id}," \
            #             + f"{roc1_bin},".replace("'", "").replace(" ", "") \
            #             + f"{roc2_bin},".replace("'", "").replace(" ", "") \
            #             + f"{cpa_side},{cpa_end}," \
            #             + f"{turn_mag_bin})"

            log_entry = [f"take_action({v1_id})"]
            log_entry.append(f"turn({v1_id},{turn_mag_bin})")
            log_entry.append(f"avoid({v1_id},{v2_id},"
                             + f"{roc1_bin}".replace("'", "").replace(" ", "")+")")
            log_entry.append(f"resume({v1_id},{v2_id},"
                             + f"{roc2_bin}".replace("'",
                                                     "").replace(" ", "") + ")")
            log_entry.append(f"side({v1_id},{v2_id},"
                             + f"{cpa_side})")

        if "bearing_deg" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            sector, arc_overtaking = self.bearing_to_sector(
                kwargs['bearing_deg'])
            log_entry = [f"sector({v_id1},{v_id2},{sector})"]
            if arc_overtaking:
                log_entry.append(f"arc_overtaking({v_id2},{v_id1})")

        if "range_m" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            range_bin = self.range_to_bins(kwargs['range_m'])
            log_entry = [f"range({v_id1},{v_id2},{range_bin})"]

        if "tcpa_s" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            tcpa_bin = self.tcpa_to_bin(kwargs['tcpa_s'])
            clo_or_op = self.closing_or_opening(kwargs['tcpa_s'])
            log_entry = [f"tcpa({v_id1},{v_id2},{tcpa_bin})"]

        if "cpa_m" in kwargs:
            v_id1 = kwargs['vessel1']
            v_id2 = kwargs['vessel2']
            cpa_bin = self.cpa_to_bin(kwargs['cpa_m'])
            log_entry = [f"dcpa({v_id1},{v_id2},{cpa_bin})"]

        if "resuming_mission" in kwargs:
            log_entry = [f"resuming_mission({kwargs['vessel']})"]

        if "waypoint_reached" in kwargs:
            log_entry = [f"waypoint_reached({kwargs['vessel']})"]

        if log_entry:
            self.log[self.n].extend(log_entry)

    def add_obs_to_log(self, obs):

        # Add time
        self.add_to_log(time_s=obs['time_s'])

        v_n = 0
        # Go through each vessel and add each observation
        for key1, value1 in obs['vessels'].items():
            # Go through each other vessel
            for key2, value2 in value1.other_vessels.items():

                # Skip the waypoints if it's the first time step
                if self.n != 0:
                    if value1.resuming_mission == True:
                        self.add_to_log(resuming_mission=True,
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

                if value1.turning or value1._final_waypoint_reached:
                    if key1 == "agent":
                        self.add_to_log(vessel=key1,
                                        waypoint_reached=True)

                # If either vessel is turning don't record any of the other stuff
                if not self.turn_logic_flag:
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

    def add_failed_waypoint_logic(self, wp_logic):
        self.log[self.n].append(f"FAILED({wp_logic})")

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

    def cpa_to_roc_bin(self, dcpa, tcpa):
        # Check the cpa for each risk of collision and find which bin is appropriate
        roc_out = []
        for roc in bin_constants.RISK_OF_COLLISION:
            dcpa_lims, tcpa_lims = self.roc_bin_to_cpa_lims_spec(roc[0])
            for n in range(len(dcpa_lims)):
                if dcpa > dcpa_lims[n][0] and dcpa < dcpa_lims[n][1] \
                        and tcpa > tcpa_lims[n][0] and tcpa < tcpa_lims[n][1]:
                    roc_out = roc[0]
        if roc_out == "opening" or roc_out == "dcpa_acceptable":
            roc_out = "no_risk"

        return roc_out

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

    def bearing_to_sector_segment(self, bearing_deg):
        seg = self.bearing_to_segment(bearing_deg=bearing_deg)
        brg_sect = [s[0] for s in bin_constants.SECTOR
                    if seg >= s[1] and seg <= s[2]][0]
        if seg >= 10 and seg <= 22:
            is_arc_overtaking = True
        else:
            is_arc_overtaking = False
        return brg_sect, is_arc_overtaking

    def bearing_to_sector(self, bearing_deg):
        brg_tmp = (bearing_deg) % 360

        sector_n = int(np.ceil(np.interp(brg_tmp,
                                         np.linspace(11.25, 348.75, 16),
                                         np.arange(0, 16),
                                         right=0)))
        brg_sect = bin_constants.SECTOR[sector_n][0]

        if brg_tmp >= 112.5 and brg_tmp <= 247.5:
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

    def roc_bin_to_cpa_lims(self, roc_bin):
        if roc_bin == "no_risk":
            roc_bin = ['opening', 'dcpa_acceptable']
        else:
            bin_i = [i for i, b in enumerate(
                bin_constants.RISK_OF_COLLISION) if b[0] == roc_bin][0]
            roc_bin = [b[0] for i, b in enumerate(bin_constants.RISK_OF_COLLISION)
                       if i <= bin_i]

        dcpa_lims = []
        tcpa_lims = []
        for roc_bin_tmp in roc_bin:
            dcpa_tcpa = [x[1:5] for x in bin_constants.RISK_OF_COLLISION
                         if x[0] == roc_bin_tmp][0]
            dcpa_lims.append([self.bin_to_dcpa(dcpa_tcpa[0])[0],
                              self.bin_to_dcpa(dcpa_tcpa[1])[1]])
            tcpa_lims.append([self.bin_to_tcpa(dcpa_tcpa[2])[0],
                              self.bin_to_tcpa(dcpa_tcpa[3])[1]])

        return dcpa_lims, tcpa_lims

    def roc_bin_to_cpa_lims_spec(self, roc_bin):
        if roc_bin == "no_risk":
            roc_bin = ['opening', 'dcpa_acceptable']
        else:
            roc_bin = [roc_bin]

        dcpa_lims = []
        tcpa_lims = []
        for roc_bin_tmp in roc_bin:
            dcpa_tcpa = [x[1:5] for x in bin_constants.RISK_OF_COLLISION
                         if x[0] == roc_bin_tmp][0]
            dcpa_lims.append([self.bin_to_dcpa(dcpa_tcpa[0])[0],
                              self.bin_to_dcpa(dcpa_tcpa[1])[1]])
            tcpa_lims.append([self.bin_to_tcpa(dcpa_tcpa[2])[0],
                              self.bin_to_tcpa(dcpa_tcpa[3])[1]])

        return dcpa_lims, tcpa_lims

    def remove_duplicates(self, log):

        log_init = [n for n in log[0] if "waypoint" not in n]
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

            # Remove duplicates
            log_entry = list(set(log_entry))
            log_entry.sort()

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
        # The area mask using the bearing and range to the waypoint
        # relative to some datum

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
        # The old version using the dcpa and tcpa for each legs of the diversion

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

    def add_wp_area_riskofcollision(self,
                                    xy_mg,
                                    cpa_roc_1,
                                    cpa_roc_2,
                                    vessel: agent.Agent,
                                    v2_id,
                                    cpa_side,
                                    cpa_end):
        # The more simple version using risks of collision based CPAs
        [dcpa1_lims, tcpa1_lims] = self.roc_bin_to_cpa_lims(roc_bin=cpa_roc_1)
        [dcpa2_lims, tcpa2_lims] = self.roc_bin_to_cpa_lims(roc_bin=cpa_roc_2)

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

            # Make the risk of collision mask
            mask_point = []
            for n in range(len(dcpa1_lims)):
                for m in range(len(dcpa2_lims)):
                    mask_point.append(dcpa1_m > dcpa1_lims[n][0]
                                      and dcpa1_m < dcpa1_lims[n][1]
                                      and tcpa1_s > tcpa1_lims[n][0]
                                      and tcpa1_s < tcpa1_lims[n][1]
                                      and dcpa2_m > dcpa2_lims[m][0]
                                      and dcpa2_m < dcpa2_lims[m][1]
                                      and tcpa2_s > tcpa2_lims[m][0]
                                      and tcpa2_s < tcpa2_lims[m][1])
            mask_out.append(any(mask_point))

            cpa_side_tmp, cpa_end_tmp = general.compute_cpa_side_end(tcpa_s=tcpa1_s,
                                                                     xy1=xy1,
                                                                     course1=course1,
                                                                     speed_mps1=speed1,
                                                                     xy2=xy2,
                                                                     course2=course2,
                                                                     speed_mps2=speed2)

            # Make the side end mask
            cpa_side_end_mask.append((cpa_side == cpa_side_tmp or (cpa_side != "port" and cpa_side != "starboard"))
                                     and (cpa_end == cpa_end_tmp or (cpa_end != "forward" and cpa_end != "aft")))
        return mask_out, cpa_side_end_mask

    def add_wp_area_turn(self,
                         xy_mg,
                         xy0,
                         course_deg,
                         turn_mag):
        # Generate mask for turn magnitudes
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
                             res=500):
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

    def add_wp_area_cpa_side(self,
                             xy_mg,
                             vessel: agent.Agent,
                             v2_id,
                             cpa_side):
        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)

        xy1 = vessel.xy
        speed1 = vessel.speed_mps
        xy2 = vessel.other_vessels[v2_id].xy
        course2 = vessel.other_vessels[v2_id].course_deg
        speed2 = vessel.other_vessels[v2_id].speed_mps
        cpa_side_mask = []
        for wp in mg_flat.T:
            course1 = general.compute_bearing(xy1,
                                              wp)
            _, _, tcpa1_s = general.compute_cpa(xy1=xy1,
                                                course1=course1,
                                                speed_mps1=speed1,
                                                xy2=xy2,
                                                course2=course2,
                                                speed_mps2=speed2)
            cpa_side_tmp, _ = general.compute_cpa_side_end(tcpa_s=tcpa1_s,
                                                           xy1=xy1,
                                                           course1=course1,
                                                           speed_mps1=speed1,
                                                           xy2=xy2,
                                                           course2=course2,
                                                           speed_mps2=speed2)
            cpa_side_mask.append(cpa_side == cpa_side_tmp)
        return cpa_side_mask

    def add_wp_area_avoid(self,
                          xy_mg,
                          cpa_roc_avoid,
                          vessel,
                          v2_id):
        [dcpa1_lims, tcpa1_lims] = self.roc_bin_to_cpa_lims(
            roc_bin=cpa_roc_avoid)

        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)

        xy1 = vessel.xy
        speed1 = vessel.speed_mps
        xy2 = vessel.other_vessels[v2_id].xy
        course2 = vessel.other_vessels[v2_id].course_deg
        speed2 = vessel.other_vessels[v2_id].speed_mps

        mask_out = []
        for wp in mg_flat.T:
            course1 = general.compute_bearing(xy1,
                                              wp)
            dcpa1_m, _, tcpa1_s = general.compute_cpa(xy1=xy1,
                                                      course1=course1,
                                                      speed_mps1=speed1,
                                                      xy2=xy2,
                                                      course2=course2,
                                                      speed_mps2=speed2)

            # Make the risk of collision mask
            mask_point = []
            for n in range(len(dcpa1_lims)):
                mask_point.append(dcpa1_m > dcpa1_lims[n][0]
                                  and dcpa1_m < dcpa1_lims[n][1]
                                  and tcpa1_s > tcpa1_lims[n][0]
                                  and tcpa1_s < tcpa1_lims[n][1])
            mask_out.append(any(mask_point))
        return mask_out

    def add_wp_area_resume(self,
                           xy_mg,
                           cpa_roc_resume,
                           vessel,
                           v2_id):
        [dcpa2_lims, tcpa2_lims] = self.roc_bin_to_cpa_lims(
            roc_bin=cpa_roc_resume)

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
        for wp in mg_flat.T:
            course1 = general.compute_bearing(xy1,
                                              wp)
            dcpa2_m, _, tcpa2_s = general.compute_future_cpas(xy1=xy1,
                                                              speed_mps1=speed1,
                                                              xy2=xy2,
                                                              course2=course2,
                                                              speed_mps2=speed2,
                                                              wp=wp,
                                                              goal_wp=goal_wp)

            # Make the risk of collision mask
            mask_point = []
            for m in range(len(dcpa2_lims)):
                mask_point.append(dcpa2_m > dcpa2_lims[m][0]
                                  and dcpa2_m < dcpa2_lims[m][1]
                                  and tcpa2_s > tcpa2_lims[m][0]
                                  and tcpa2_s < tcpa2_lims[m][1])
            mask_out.append(any(mask_point))

        return mask_out

    def mask_to_wp(self,
                   xy_mg,
                   wp_area,
                   vessel):
        # get the closest allowable waypoint to the goal
        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)
        mg_flat_wp_area = mg_flat[:, wp_area]

        wp_to_final = np.linalg.norm(mg_flat_wp_area - np.reshape(vessel.goal_waypoint,
                                                                  [2, 1]), axis=0)
        if len(vessel.waypoints) == 3:
            pos_to_wp = np.linalg.norm(mg_flat_wp_area - np.reshape(vessel.waypoints[1],
                                                                    [2, 1]), axis=0)
        else:
            pos_to_wp = np.linalg.norm(mg_flat_wp_area - np.reshape(vessel.xy,
                                                                    [2, 1]), axis=0)

        distances = pos_to_wp + wp_to_final

        if len(distances) != 0:
            min_i = np.argmin(distances)
            xy_out = mg_flat_wp_area[:, min_i].tolist()
        else:
            xy_out = []
            print("##################################################################\n"
                  + "################ WARNING! No valid wp location. ##################\n"
                  + "##################################################################")
        return xy_out

    def waypoint_logic_to_coordinates(self,
                                      waypoint_logic: list,
                                      vessels: list[agent.Agent]):

        # Go from waypoint logic with avoid, resume, cpa_side and turn
        xy_list = [v.xy for v in vessels.values()]
        wp_area, xy_mg = self._setup_blank_wp_area(xy_list)

        wp: str
        for wp in waypoint_logic:
            wp_list = wp.replace('(', ',').strip(")").split(",")
            v0 = vessels[wp_list[1]]
            if wp_list[0] == "turn":
                # turn(vessel, magnitude)
                wp_area_tmp = self.add_wp_area_turn(xy_mg=xy_mg,
                                                    xy0=v0.xy,
                                                    course_deg=v0.course_deg,
                                                    turn_mag=wp_list[2])
            elif wp_list[0] == "avoid":
                # avoid(vessel0, vessel1, cpa_risk)
                wp_area_tmp = self.add_wp_area_avoid(xy_mg=xy_mg,
                                                     cpa_roc_avoid=wp_list[3],
                                                     vessel=v0,
                                                     v2_id=wp_list[2])
            elif wp_list[0] == "resume":
                # resume(vessel0, vessel1, cpa_risk)
                wp_area_tmp = self.add_wp_area_resume(xy_mg=xy_mg,
                                                      cpa_roc_resume=wp_list[3],
                                                      vessel=v0,
                                                      v2_id=wp_list[2])
            elif wp_list[0] == "cpa_side":
                # cpa_side(vessel0, vessel1, port_or_starboard)
                cpa_side = wp_list[3]
                if cpa_side != "port" and cpa_side != "starboard":
                    cpa_side = ""
                wp_area_tmp = self.add_wp_area_cpa_side(xy_mg=xy_mg,
                                                        vessel=v0,
                                                        v2_id=wp_list[2],
                                                        cpa_side=cpa_side)

            # plt.figure
            # plt.imshow(np.array(wp_area_tmp).reshape(xy_mg[0].shape),
            #            extent=[np.min(xy_mg[0]), np.max(xy_mg[0]),
            #                    np.min(xy_mg[1]), np.max(xy_mg[1])],
            #            origin='lower')
            # plt.scatter(v0.xy[0], v0.xy[1])
            # plt.show()

            wp_area = wp_area & wp_area_tmp

        xy_out = self.mask_to_wp(xy_mg,
                                 wp_area,
                                 v0)

        return v0.vessel_id, xy_out

    def waypoint_logic_to_coordinates_wp(self,
                                         waypoint_logic: list,
                                         vessels: list[agent.Agent]):

        xy_list = [v.xy for v in vessels.values()]
        wp_area, xy_mg = self._setup_blank_wp_area(xy_list)

        wp: str
        for wp in waypoint_logic:
            #  waypoint(V0,V1,avoid,resume,cpa_side,cpa_end,turn_mag)
            wp_list = wp.strip("waypoint").strip("()").split(",")
            v0 = vessels[wp_list[0]]
            v1_id = wp_list[1]
            cpa_roc_1 = wp_list[2]
            cpa_roc_2 = wp_list[3]
            cpa_side = wp_list[4]
            if cpa_side != "port" and cpa_side != "starboard":
                cpa_side = ""
            cpa_end = wp_list[5]
            if cpa_end != "forward" and cpa_end != "aft":
                cpa_end = ""
            turn_mag = wp_list[6]

            cpa_mask, side_end_mask = self.add_wp_area_riskofcollision(xy_mg=xy_mg,
                                                                       cpa_roc_1=cpa_roc_1,
                                                                       cpa_roc_2=cpa_roc_2,
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
            # plt.show()

            wp_area = wp_area & cpa_mask & turn_mask & side_end_mask

        # get the closest allowable waypoint to the goal
        mg_flat = np.concatenate([[xy_mg[0].flatten()],
                                  [xy_mg[1].flatten()]],
                                 axis=0)
        mg_flat_wp_area = mg_flat[:, wp_area]

        wp_to_final = np.linalg.norm(mg_flat_wp_area - np.reshape(v0.goal_waypoint,
                                                                  [2, 1]), axis=0)
        if len(v0.waypoints) == 3:
            pos_to_wp = np.linalg.norm(mg_flat_wp_area - np.reshape(v0.waypoints[1],
                                                                    [2, 1]), axis=0)
        else:
            pos_to_wp = np.linalg.norm(mg_flat_wp_area - np.reshape(v0.xy,
                                                                    [2, 1]), axis=0)

        distances = pos_to_wp + wp_to_final

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
