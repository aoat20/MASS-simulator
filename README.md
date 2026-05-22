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
    obs, obs_log = mass_sim.next_step()

    # Get the observations
    obs, obs_log = mass_sim.get_obs()

    # Do actions (the time is arbitrary)
    if obs['time_s'] == 250:
        # Set waypoint location
        mass_sim.set_waypoints('agent',
                               [[430_000,
                                 5_555_000]])
    
    if obs['time_s'] == 300:
        # Set waypoint location with a list of logical statements
        # add_waypoint(vessel1,vessel2,dcpa_avoid,tcpa_avoid,dcpa_resume,tcpa_resume,cpa_side,cpa_end,turn_mag)
        mass_sim.send_waypoint_logic(
            ["add_waypoint(agent,cruiseliner1,safe,short,safe,imminent,port,aft,large)"])

mass_sim.save_episode()

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
To change the waypoints of a specific vessel, first click that vessel (defaults to agent), right click along the path or on a waypoint and click "Change" or "Add" or "Remove". If you click along the path of the vessel, the waypoint will be added in between the two waypoints either side of the path you clicked on, otherwise it will be added at the end of the current path. Other vessels can then be clicked and their waypoints changed as desired. Once all the desired waypoints have been added/changed, right click anywhere and click "Send" and the vessels will now follow the new waypoints. 

Another method of adding waypoints is to left click on the vessel, then middle click all your desired waypoint locations, (reclicking a point will remove it), then middle click on the vessel to activate the waypoints. 

The speed can be adjusted by clicking on a vessel and dragging up to the desired speed.

## Test Mode 
Test mode allows programmatic interaction with the simulator. Follow the structure shown in the example script, using the "is_episode_running()" in a while loop. 
- "next_step()" advances a time step.
- "get_obs()" return the observation dictionary containing 'time' and 'vessels'. 'vessels' is a dictionary of the vessels in the episode as Agent objects which contain the attributes "course_deg", "speed_kn", "wayponts" and "xy". They also have a dictionary of the CPA, TCPA, Range and Bearing to each other vessel in "other_vessels". Example usage:
```python
# Get observation dict
obs = mass_sim.get_obs()

# Get the state of the vessel called boat1
obs['vessels']['boat1'].xy
obs['vessels']['boat1'].speed_kn
obs['vessels']['boat1'].course_deg
obs['vessels']['boat1'].waypoints

# Get relative state of another vessel called boat2
obs['vessels']['boat1'].other_vessels['boat2'].cpa_m
obs['vessels']['boat1'].other_vessels['boat2'].tcpa_s
obs['vessels']['boat1'].other_vessels['boat2'].range_m
obs['vessels']['boat1'].other_vessels['boat2'].bearing_deg
```
- "set_waypoints" allows you to set waypoints for each of the vessels. Specify the vessel_id as the first argument and the waypoints in a list as the second.
- "save_episode()" will the save the episode log.

## Playback Mode 
Playback mode also adds a way of moving to specific points in time in the episode. 

## Scenario Generator
To run the scenario generator and create custom scenarios, run scenario_generator.py in python:
```bash
python3 scenario_generator.py
```

This will open a map with a topography map (currently unused). To navigate the map, middle click and drag to pan and scroll to zoom in or out. Right click will open the following menu:

<img width="243" height="100" alt="Screenshot from 2025-10-30 08-36-43" src="https://github.com/user-attachments/assets/7b82f7a2-c04d-47d5-9fac-b79a7d8a13a1" />

This can be used to add vessels, adjust the topology of the map and save the scenario (recommended untick "Show depth").

### Adding vessels
To add a vessel, right click in the desired location and click "Add vessel here". This will place the vessel and then open the following menu which can be used to set the vessel parameters: 

<img width="323" height="169" alt="Screenshot from 2025-10-30 08-38-16" src="https://github.com/user-attachments/assets/a16fabe6-0315-4105-aba9-ad83760af275" />

Left clicking anywhere on the map will add a waypoint in that location. When you're done, right click again and click "Done adding vessel". Repeat for however many vessels are desired. 

### Topographical map
Currently unused, work in progress.

### Saving the scenario
When the desired scenario is defined, right click and click "Save scenario". This will bring up a file dialog box to get the desired save location. Enter a scenario name and press "Ok". This scenario is now ready to be used in the MASS simulator.
