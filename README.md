# Udacity CarND System Integration Project
**Team Members:**
- Medhat Omr (Team Leader) (medhat.omr@icloud.com)
- Ibrahim Almohandes (heema111@gmail.com)
- Dmitry Gavrilenko (dmitry.gavrilenko@icloud.com)
- Brian Meier (bjmeier@verizon.net)

### Submission checklist and requirements

- Smoothly follow waypoints in the simulator.
    - As shown in video linked below, the car smoothly follows waypoints in the simulator.
    
- Respect the target top speed set for the waypoints.
    - As shown in the video linked below, the car respects the 40 km/hr simulator speed limit.
    
- Stop at traffic lights when needed.
    - As shown in the video linked below, the car stops at trafffic lights when needed. 
  
- Stop and restart PID controllers depending on the state of ``/vehicle/dbw_enabled``.
    - Per lines 27 - 30 of twist_controller.py, the PID controller is reset ``if not dbw_enabled``.
    
- Publish throttle, steering, and brake commands at 50 hz.
    - Per line 7 of twist_controller.py, the CMD_RATE is set to 50 Hz.  This values is used by the dbw node to set the publish rate for the throttle, steering and brake commands.
    
- Launch correctly using the launch files provided in the capstone repo.
    - Program launches with ``launch/styx.launch`` and ``launch/site.launch`` for the simulator and car respectively.
    

[![Click to View Sumulator Video](https://img.youtube.com/vi/GP0a8tXMdgk/0.jpg)](https://www.youtube.com/watch?v=GP0a8tXMdgk "Simulator Video")

[Simulator Video](https://www.youtube.com/watch?v=GP0a8tXMdgk)
