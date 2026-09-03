from mass_simulator import MASSsim

# To run scenario 4 in manual mode
mass_sim = MASSsim(scenario=4,
                   mode='manual',
                   log_dir='/home/user/test_folder')

# To set up scenario 4 in test mode
mass_sim = MASSsim(scenario=2,
                   mode='test',
                   plotter=True)

# Run until mission is finished
while mass_sim.is_episode_running():
    # Advance to the next time step and get observations
    if mass_sim.next_step():
        obs, obs_logic = mass_sim.get_obs()

        # DO PROLOG STUFF HERE

        # Add a waypoint at an arbitrary location and time
        if obs['time_s'] == 250:
            mass_sim.set_waypoints('agent',
                                   [[430_000,
                                    5_555_000]])

        # Add a waypoint at an arbitrary time and a location specified by some logical statements
        if obs['time_s'] == 300:
            mass_sim.send_waypoint_logic(
                ["cpa_side(agent,cruiseliner1,starboard)",
                 "avoid(agent,cruiseliner1,no_risk)",
                 "resume(agent,cruiseliner1,no_risk)",
                 "turn(agent,small)"])
mass_sim.save_episode()


# Playback one of the log files
MASSsim(mode='playback',
        log_file='logs/log_0.json')
MASSsim(mode='playback',
        log_file=0)
