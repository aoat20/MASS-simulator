from mass_simulator import MASSsim

# To run scenario 4 in manual mode
mass_sim = MASSsim(scenario=4,
                   mode='manual')

# To set up scenario 4 in test mode
mass_sim = MASSsim(scenario=4,
                   mode='test',
                   plotter=True)

# Run until mission is finished
while mass_sim.is_episode_running():
    # Advance to the next time step
    mass_sim.next_step()

    # Get the observations
    obs = mass_sim.get_obs()

    if obs['time_s'] == 250:
        mass_sim.set_waypoints('agent',
                               [[430_000,
                                 5_555_000]])


# Playback one of the log files
MASSsim(mode='playback',
        log_file='logs/log_0.json')
MASSsim(mode='playback',
        log_file=0)
