# MASS-simulator

A package to simulate multiple vessels travelling between waypoints. 

## Example Usage (included in example_script.py)

```python
from mass_simulator import MASSsim

# To run scenario 4 in manual mode
mass_sim = MASSsim(scenario=4,
                   mode='manual',
                   log_dir='/home/user/test_log_folder')

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

```
## General
- "scenario" is either be a number referring to the numbered scenarios in the scenarios folder, or a string with the path to a file containing a scenario.
- "mode" is a string that is either "manual", "test", or "playback".
- "plotter" is a bool and is ignored for all modes except "test" where it can be set to True to watch the episode.
- "log_file" is only used in playback mode and is either a number referring to the log files in the "logs" folder, or a string with the path to a log file.
- "log_dir" is a string that specifies the path to the directory where log files will be saved in manual or test mode. It defaults to /logs.
- In each mode there will be an "AIS Data" window which will show the current time into the episode and the CPA, TCPA, Range and Bearing to all the other vessels relative to whichever vessel is in focus. Clicking on one of the other vessels will change the vessel in focus. 
<img width="441" height="173" alt="Screenshot from 2025-09-05 11-31-31" src="https://github.com/user-attachments/assets/2c06a034-ee87-4684-abb2-5b9e24f8a106" />

## Manual Mode 
To change the waypoint of a specific vessel, first click that vessel (defaults to agent), tick the "Change waypoints" box, and simply click the desired waypoints. Other vessels can then be clicked and their waypoints changed as desired. Once all the desired waypoints have been added, unclick the "Change waypoints" box and the vessels will now follow the new waypoints. If while changing waypoints you decide against them, click the "Clear waypoints" button to clear the waypoints.

## Test Mode 
Test mode allows programmatic interaction with the simulator. Follow the structure shown in the example script, using the "is_episode_running()" in a while loop. 
- "next_step()" advances a time step.
- "get_obs()" return the observation dictionary containing 'time' and 'vessels'. 'vessels' is a dictionary of the vessels in the episode as Agent objects which contain the attributes "course_deg", "speed_kn", "wayponts" and "xy". They also have a dictionary of the CPA, TCPA, Range and Bearing to each other vessel in "other_vessels".
- "set_waypoints" allows you to set waypoints for each of the vessels. Specify the vessel_id as the first argument and the waypoints in a list as the second.
- "save_episode()" will the save the episode log.

## Playback Mode 
Playback mode also adds a way of moving to specific points in time in the episode. 
