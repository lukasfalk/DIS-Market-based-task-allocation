# Distributed Intelligent Systems ENG-466
Project 4 - Market-based task allocation with heterogeneous team of robots


Notes:
We have privileged knowledge of map (coordinates of wall), GNSS absolute robot location and tasks at all times. 
Time drain (2 minutes) is for moving to task, and completing task (waiting for x seconds depending on task and robot type).
The robots know the 1/3 + 2/3 for spawning a new task A or B.

Questions:
Do we use GNSS at the moment?
Should robots moving over a task that isnt assigned to them still trigger??? 
^right now we pass directly over tasks lmao, felt like it was different in the lab??

It looks like the robots are quite far away from the tasks when they are considered finished? Did i fuck up the code??
