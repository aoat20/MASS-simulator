from mass_simulator import MASSsim
import yaml

train_scen = ['crossing1',  # early action
              'crossing1',  # appropriate action time
              'crossing1',  # late action
              'crossing2',  # early action
              'crossing2',  # appropriate action time
              'crossing2',  # late action
              'crossing2',  # even later
              'headon1',  # early action
              'headon1',  # appropriate action time
              'headon1',  # late action
              'overtaking1',  # early action
              'overtaking1',  # appropriate action time
              'overtaking1',  # late action
              'passing1',
              ]

train_scen = ['headon1',  # even later
              ]
for t in train_scen:
    mass_sim = MASSsim(scenario=t,
                       mode='test',
                       plotter=True)

    take_action = False

    wp_pos = dict()
    p = 0
    wp_neg = dict()
    n = 0

    # Run until mission is finished
    while mass_sim.is_episode_running():
        # Advance to the next time step
        if mass_sim.next_step():
            obs, obs_logic = mass_sim.get_obs()

            if any(["take_action" in ol for ol in obs_logic]) and obs['time_s'] != 1:

                print("take_action, waypoint")
                x = input()

                if not take_action:
                    facts = [ob for ob in obs_logic_prev
                             if "clock" not in ob]
                    if x[0] == 'p':
                        act_pn = 'pos'
                    elif x[0] == 'n':
                        act_pn = 'neg'
                    take_action = True

                wp = [ob for ob in obs_logic
                      if all([s not in ob for s in ['range',
                                                    'sector',
                                                    'clock',
                                                    'waypoint_reached',
                                                    "take_action",
                                                    "arc_overtaking"]])]

                if len(x) >= 2:
                    if x[1] == "p":
                        wp_pos[p] = wp
                        p += 1
                    elif x[1] == "n":
                        wp_neg[n] = wp
                        n += 1

                if len(x) > 2 or len(x) == 0:
                    data = [dict(facts=facts,
                                 take_action=act_pn,
                                 waypoints=dict(pos=wp_pos,
                                                neg=wp_neg))]

                    with open(f"examples/{t}.yaml", "a+") as f:
                        yaml.dump(data, f, default_flow_style=False)
                    mass_sim.end_episode()
                mass_sim._plotter.set_play(False)

        obs_logic_prev = obs_logic
