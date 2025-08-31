import numpy as np


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
    if tcpa_s < 0:
        tcpa_s = -1
    return cpa_yds, tcpa_s


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
