import numpy as np
from dataclasses import dataclass
from mass_simulator.general import *


class Agent():
    def __init__(self,
                 vessel_id,
                 xy_init,
                 waypoints,
                 draught_m=0,
                 speed_change_knps=1,
                 speed_max_kn=20,
                 turning_radius=100,
                 speed_kn: float = 0.,
                 speed_mps: float = 0.):
        # Agent parameters
        self.vessel_id = vessel_id
        self.draught_m = draught_m
        self.activity = ''
        self.speed_change_knps = speed_change_knps
        self.max_speed_kn = speed_max_kn
        self.turning_radius = turning_radius
        self.xy = xy_init
        self.xy_hist = [xy_init]

        # Set boat initial speed in either knots or m/s
        if speed_kn:
            self.speed_kn = speed_kn
            self.speed_mps = kn_to_mps(speed_kn)
        elif speed_mps:
            self.speed_mps = speed_mps
            self.speed_kn = mps_to_kn(speed_mps)
        else:
            raise ValueError(f"No speed specified for vessel {self.vessel_id}")

        self.other_vessels = {}
        self.waypoints = waypoints
        self.waypoint_n = 1
        self.goal_waypoint = waypoints[-1]
        self._final_waypoint_reached = False

        # compute course to first waypoint
        self.course_deg = compute_bearing(xy_init,
                                          self.waypoints[1])

        self._requested_speed_mps = speed_mps
        self._requested_course_deg = self.course_deg
        self._compute_xy_step()

    def next_step(self,
                  t_step):
        self.xy = [self.xy[0]+self.xy_step[0]*t_step,
                   self.xy[1]+self.xy_step[1]*t_step]
        self.xy_hist.append(self.xy)

        self._next_waypoint()

        if not self._final_waypoint_reached:
            self.update_course(compute_bearing(self.xy,
                                               self.waypoints[self.waypoint_n]))

        if self._requested_speed_mps != self.speed_mps:
            self._compute_new_speed(t_step)

        if self._requested_course_deg != self.course_deg:
            self._compute_new_course(t_step)

    def _compute_xy_step(self):
        # Compute the step in x and y based on the course and speed
        course_rad = np.deg2rad(self.course_deg)
        self.xy_step = [self.speed_mps*np.sin(course_rad),
                        self.speed_mps*np.cos(course_rad)]

    def update_waypoints(self,
                         waypoints):
        waypoints.append(self.goal_waypoint)
        self.waypoints = waypoints
        self.waypoint_n = 0
        brg = compute_bearing(self.xy,
                              waypoints[0])
        self.update_course(brg)

    def _next_waypoint(self):
        # While not past the final waypoint
        if self.waypoint_n < len(self.waypoints):
            # Get distance to current waypoint
            d = compute_perp_distance(self.xy,
                                      self.xy_hist[-2],
                                      self.waypoints[self.waypoint_n])
            if d < 50:
                if self.waypoint_n < len(self.waypoints)-1:
                    self.waypoint_n += 1
                else:
                    self._final_waypoint_reached = True

    def update_course(self,
                      course_deg):
        self._requested_course_deg = course_deg

    def update_speed(self,
                     speed_mps):
        self._requested_speed_mps = speed_mps

    def _compute_new_speed(self,
                           t_step):
        speed_change_tmp = t_step*mps_to_kn(self.speed_change_knps)
        if self._requested_speed_mps < self.speed_mps:
            self.speed_mps = np.clip(self.speed_mps-speed_change_tmp,
                                     self._requested_speed_mps,
                                     None)
        elif self._requested_speed_mps > self.speed_mps:
            self.speed_mps = np.clip(self.speed_mps+speed_change_tmp,
                                     None,
                                     self._requested_speed_mps)
        self.speed_kn = mps_to_kn(self.speed_mps)
        self._compute_xy_step()

    def _compute_new_course(self,
                            t_step):
        course_change_tmp = t_step * \
            np.rad2deg(self.speed_mps/self.turning_radius)
        course_diff = (self._requested_course_deg-self.course_deg-180) % 360
        print(course_diff)
        print(self.course_deg)
        if course_diff < 180:
            self.course_deg = np.clip(self.course_deg-course_change_tmp,
                                      self._requested_course_deg,
                                      None)
        elif course_diff > 180:
            self.course_deg = np.clip(self.course_deg+course_change_tmp,
                                      None,
                                      self._requested_course_deg)
        self._compute_xy_step()

    def update_other_vessels(self,
                             other_vessels: dict):
        v: Agent
        for v in other_vessels.values():
            if v.vessel_id != self.vessel_id:
                cpa_m, cpa_yds, tcpa_s = self.compute_cpa_tcpa(v.xy,
                                                               v.speed_mps,
                                                               v.course_deg)
                range_m = compute_distance(self.xy,
                                           v.xy)
                range_yds = m_to_yds(range_m)
                bearing_deg = compute_bearing(self.xy,
                                              v.xy)
                self.other_vessels[v.vessel_id] = OtherVessel(cpa_m=cpa_m,
                                                              cpa_yds=cpa_yds,
                                                              tcpa_s=tcpa_s,
                                                              range_m=range_m,
                                                              range_yds=range_yds,
                                                              bearing_deg=bearing_deg)

    def compute_cpa_tcpa(self, xy, speed_mps, course_deg):
        # Compute differences
        dv_x = self.speed_mps*np.sin(np.deg2rad(self.course_deg)) - \
            speed_mps*np.sin(np.deg2rad(course_deg))
        dv_y = self.speed_mps*np.cos(np.deg2rad(self.course_deg)) - \
            speed_mps*np.cos(np.deg2rad(course_deg))
        dx = self.xy[0] - xy[0]
        dy = self.xy[1] - xy[1]

        # CPA and TCPA
        cpa_m = np.abs(dv_y*dx - dv_x*dy)/np.sqrt(dv_x**2 + dv_y**2)
        cpa_yds = m_to_yds(cpa_m)
        tcpa_s = - (dv_x*dx + dv_y*dy)/(dv_x**2 + dv_y**2)
        if tcpa_s < 0:
            tcpa_s = 0
        return cpa_m, cpa_yds, tcpa_s


@dataclass
class OtherVessel:
    cpa_m: float
    cpa_yds: float
    tcpa_s: float
    range_m: float
    range_yds: float
    bearing_deg: float
