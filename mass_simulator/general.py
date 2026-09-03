import numpy as np
from scipy.interpolate import griddata
import math


def m_to_yds(m):
    return m*1.09361


def yds_to_m(yds):
    return yds*0.9144


def mps_to_kn(mps):
    return mps*1.94384


def kn_to_mps(kn):
    return kn*0.514444


def compute_bearing(xy1,
                    xy2):
    """Compute the bearing from the points xy1 to xy2"""
    bearing_deg = np.rad2deg(np.arctan2(xy2[0]-xy1[0],
                                        xy2[1]-xy1[1]))
    return bearing_deg


def compute_relative_bearing(xy1, heading1,
                             xy2):
    abs_brg = compute_bearing(xy1, xy2)
    return abs_brg-heading1


def compute_cpa(xy1, course1, speed_mps1,
                xy2, course2, speed_mps2):
    """Compute the cpa and tcpa between vessel1 and vessel2 with their
    respective positions, courses and speeds"""

    # Compute differences
    dv_x = speed_mps1*np.sin(np.deg2rad(course1)) - \
        speed_mps2*np.sin(np.deg2rad(course2))
    dv_y = speed_mps1*np.cos(np.deg2rad(course1)) - \
        speed_mps2*np.cos(np.deg2rad(course2))
    dx = xy1[0] - xy2[0]
    dy = xy1[1] - xy2[1]
    # CPA and TCPA
    cpa_m = np.abs(dv_y*dx - dv_x*dy)/np.sqrt(dv_x**2 + dv_y**2)
    cpa_yds = m_to_yds(cpa_m)
    tcpa_s = - (dv_x*dx + dv_y*dy)/(dv_x**2 + dv_y**2)

    return cpa_m, cpa_yds, tcpa_s


def compute_vessel_xy_cpa(tcpa_s,
                          xy1, course1, speed_mps1,
                          xy2, course2, speed_mps2):
    # Compute the vessel locations at the CPA
    course1_rad = np.deg2rad(course1)
    xy1_cpa = xy1 + speed_mps1*tcpa_s*np.array([np.sin(course1_rad),
                                                np.cos(course1_rad)])
    course2_rad = np.deg2rad(course2)
    xy2_cpa = xy2 + speed_mps2*tcpa_s*np.array([np.sin(course2_rad),
                                                np.cos(course2_rad)])
    return xy1_cpa, xy2_cpa


def compute_future_cpas(xy1, speed_mps1,
                        xy2, course2, speed_mps2,
                        wp, goal_wp):
    v2_course_rad = np.deg2rad(course2)
    # Distance from agent pos to wp
    r = compute_distance(xy1, wp)
    # Travel time to wp
    travel_time = r/speed_mps1
    # Heading from waypoint to resumption point
    course1_new = 90-np.rad2deg(np.atan2(goal_wp[1]-wp[1],
                                         goal_wp[0]-wp[0]))
    v2_xy_new = xy2 + speed_mps2*travel_time*np.array([np.sin(v2_course_rad),
                                                       np.cos(v2_course_rad)])
    cpa_m, cpa_yds, tcpa_s = compute_cpa(wp, course1_new, speed_mps1,
                                         v2_xy_new, course2, speed_mps2)
    return cpa_m, cpa_yds, tcpa_s


def compute_cpa_side_end(tcpa_s,
                         xy1, course1, speed_mps1,
                         xy2, course2, speed_mps2):
    course1_rad = np.deg2rad(course1)
    course2_rad = np.deg2rad(course2)
    # Compute the xy positions at the cpa
    xy1_cpa = xy1 + speed_mps1*tcpa_s*np.array([np.sin(course1_rad),
                                                np.cos(course1_rad)])
    xy2_cpa = xy2 + speed_mps2*tcpa_s*np.array([np.sin(course2_rad),
                                                np.cos(course2_rad)])
    # Compute bearing of wp relative to second vessel
    x_diff = xy2_cpa[0] - xy1_cpa[0]
    y_diff = xy2_cpa[1] - xy1_cpa[1]
    theta = (np.array(90-np.rad2deg(np.atan2(y_diff, x_diff))-course1)) % 360
    theta_180 = (theta+180) % 360 - 180
    if theta_180 < 0:
        cpa_side = "port"
    elif theta_180 > 0:
        cpa_side = "starboard"

    if np.abs(theta_180) <= 90:
        cpa_end = "forward"
    elif np.abs(theta_180) <= 180:
        cpa_end = "aft"

    return cpa_side, cpa_end


def compute_distance(xy1,
                     xy2):
    """Compute the distance between point xy1 and xy2"""
    return np.sqrt((xy1[0]-xy2[0])**2 + (xy1[1]-xy2[1])**2)


def convert_dms_to_dec(coord_dms):
    """Convert degree minute seconds coordinates to decimal degree coordinates"""
    multiplier = 1 if coord_dms[-1] in ['N', 'E'] else -1
    coord_dec = multiplier * \
        sum(float(x) / 60 ** n for n,
            x in enumerate(coord_dms[:-1].split('-')))
    return coord_dec


def compute_perp_distance(A,
                          B,
                          E):
    """Find the perpendicular distance between the point E and the line
    connecting A and B"""

    # vector AB
    AB = [None, None]
    AB[0] = B[0] - A[0]
    AB[1] = B[1] - A[1]

    # vector BP
    BE = [None, None]
    BE[0] = E[0] - B[0]
    BE[1] = E[1] - B[1]

    # vector AP
    AE = [None, None]
    AE[0] = E[0] - A[0]
    AE[1] = E[1] - A[1]

    # Variables to store dot product

    # Calculating the dot product
    AB_BE = AB[0] * BE[0] + AB[1] * BE[1]
    AB_AE = AB[0] * AE[0] + AB[1] * AE[1]

    # Minimum distance from
    # point E to the line segment
    d = 0

    # Case 1
    if (AB_BE > 0):

        # Finding the magnitude
        y = E[1] - B[1]
        x = E[0] - B[0]
        d = np.sqrt(x * x + y * y)

    # Case 2
    elif (AB_AE < 0):
        y = E[1] - A[1]
        x = E[0] - A[0]
        d = np.sqrt(x * x + y * y)

    # Case 3
    else:

        # Finding the perpendicular distance
        x1 = AB[0]
        y1 = AB[1]
        x2 = AE[0]
        y2 = AE[1]
        mod = np.sqrt(x1 * x1 + y1 * y1)
        d = abs(x1 * y2 - y1 * x2) / mod

    return d


def compute_interp_depth_map(depth_points):
    points = [d[0][::-1] for d in depth_points]
    values = [d[1] for d in depth_points]

    points_np = np.array(points)
    p_min = np.min(points_np, axis=0)
    p_max = np.max(points_np, axis=0)

    grid_x, grid_y = np.mgrid[p_min[0]:p_max[0]:100,
                              p_min[1]:p_max[1]:100]

    depth_map = griddata(points,
                         values,
                         (grid_x, grid_y),
                         method="linear",
                         fill_value=0)

    depth_map_norm = (depth_map-np.min(depth_map)) * \
        (1/(1-np.min(depth_map)))

    depth_map_norm_fl = np.flipud(depth_map_norm)

    return depth_map_norm_fl, p_min, p_max


def solve_heading_for_desired_cpa(
    x1, y1, speed1,
    x2, y2, speed2, heading2_deg,
    desired_cpa,
    tolerance=1e-3
):
    """
    Solve for Vessel 1 headings that produce a desired CPA.

    Marine headings:
        0° = North
        90° = East
        clockwise positive

    Returns:
        list of headings in degrees
    """

    # ------------------------------------------------------------
    # Convert marine heading to math coordinates
    # ------------------------------------------------------------
    def heading_to_velocity(speed, heading_deg):

        theta = math.radians(90.0 - heading_deg)

        vx = speed * math.cos(theta)
        vy = speed * math.sin(theta)

        return vx, vy

    # ------------------------------------------------------------
    # Compute CPA for a candidate heading
    # ------------------------------------------------------------
    def compute_cpa(heading1_deg):

        v1x, v1y = heading_to_velocity(speed1, heading1_deg)
        v2x, v2y = heading_to_velocity(speed2, heading2_deg)

        # Relative position
        rx = x2 - x1
        ry = y2 - y1

        # Relative velocity
        vrx = v2x - v1x
        vry = v2y - v1y

        vr2 = vrx**2 + vry**2

        # Same velocity -> constant separation
        if vr2 < 1e-12:
            return math.hypot(rx, ry)

        # Time to CPA
        tcpa = -(rx * vrx + ry * vry) / vr2

        # Closest point
        cx = rx + vrx * tcpa
        cy = ry + vry * tcpa

        return math.hypot(cx, cy)

    # ------------------------------------------------------------
    # Search all headings
    # ------------------------------------------------------------
    headings = []

    scan = np.linspace(0, 360, 3601)

    errors = []

    for h in scan:

        cpa = compute_cpa(h)

        errors.append(cpa - desired_cpa)

    # Detect zero crossings
    for i in range(len(scan) - 1):

        e1 = errors[i]
        e2 = errors[i + 1]

        if abs(e1) < tolerance:
            headings.append(scan[i])

        elif e1 * e2 < 0:

            # Bisection refinement
            lo = scan[i]
            hi = scan[i + 1]

            for _ in range(40):

                mid = 0.5 * (lo + hi)

                emid = compute_cpa(mid) - desired_cpa

                if abs(emid) < tolerance:
                    break

                if e1 * emid < 0:
                    hi = mid
                    e2 = emid
                else:
                    lo = mid
                    e1 = emid

            headings.append(mid)

    # Remove duplicates
    unique = []

    for h in headings:

        h = h % 360.0

        if not any(abs(h - u) < 0.1 for u in unique):
            unique.append(90-h)

    return sorted(unique)
