import numpy as np


class bin_constants:
    RANGE_BINS = [["very_near", 0, 1*1852],
                  ["near", 1*1852, 2*1852],
                  ["middle", 2*1852, 3*1852],
                  ["far", 3*1852, 5*1852],
                  ["very_far", 5*1852, np.inf]]

    SECTOR = [["ahead", 0, 0],
              ["starboard_bow_forward", 1, 3],
              ["starboard_bow_broad", 4, 4],
              ["starboard_beam_forward", 5, 7],
              ["starboard_beam", 8, 8],
              ["starboard_beam_aft", 9, 11],
              ["starboard_quarter_broad", 12, 12],
              ["starboard_quarter_aft", 13, 15],
              ["astern", 16, 16],
              ["port_quarter_aft", 17, 19],
              ["port_quarter_broad", 20, 20],
              ["port_beam_aft", 21, 23],
              ["port_beam", 24, 24],
              ["port_beam_forward", 25, 27],
              ["port_bow_broad", 28, 28],
              ["port_bow_forward", 29, 31]
              ]

    CPA_BINS = [["critical", 0, 0.1*1852],
                ["very_close", 0.1*1852, 0.25*1852],
                ["close", 0.25*1852, 0.5*1852],
                ["marginal", 0.5*1852, 1*1852],
                ["safe", 1*1852, np.inf]]

    TCPA_BINS = [["opening", -np.inf, 0],
                 ["imminent", 0, 3*60],
                 ["short", 3*60, 12*60],
                 ["medium", 12*60, 18*60],
                 ["long", 18*60, 24*60],
                 ["very_long", 24*60, np.inf]]

    TURN_MAGNITUDES = [["insubstantial", 0, 20],
                       ["small", 20, 30],
                       ["moderate", 30, 40],
                       ["large", 40, 60],
                       ["very_large", 60, 180]]

    # roc, dcpa1, dcpa2, tcpa1, tcpa2
    RISK_OF_COLLISION = [["opening", "critical", "safe",        # no_risk
                                     "opening", "opening"],
                         ["dcpa_acceptable", "marginal", "safe",    # no_risk
                                             "imminent", "very_long"],
                         ["risk_developing", "critical", "close",
                          "long", "very_long"],
                         ["imminent_critical", "critical", "critical",
                                               "imminent", "imminent"],
                         ["imminent_veryclose", "very_close", "very_close",
                                                "imminent", "imminent"],
                         ["imminent_close", "close", "close",
                                            "imminent", "imminent"],
                         ["short_critical", "critical", "critical",
                                            "short", "short"],
                         ["short_veryclose", "very_close", "very_close",
                                             "short", "short"],
                         ["short_close", "close", "close",
                                         "short", "short"],
                         ["medium_critical", "critical", "critical",
                                             "medium", "medium"],
                         ["medium_veryclose", "very_close", "very_close",
                                              "medium", "medium"],
                         ["medium_close", "close", "close",
                                          "medium", "medium"]]
