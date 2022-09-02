# RobotLocalization
In this project, we used Particle Filter algorithm to localize our Vector robot both in simulation and real world maze. The code is organized as follows:
* controllerv1.py: Has the Robot class for the simulator robot and performs methods such as read_laser() to get the laser output of the robot and get_action() to get possible actions and move() to move the robot.
* utility.py: some useful functions including Line_initersection() gets two lines and returns their intersection point, if existed, and is used in Laser() to find the laser output of each particl.
* controllerv2.py: Has the Robot class with similar methods to the simulator class for the real robot.
* partile_filter.py: The particle filter class that initiates random particles, moves robot and particles, reads their laser, calculates particle weights, and resamples particles. For resampling we used augmented Monte Carlo for addressing kidnapping.
<br />Videos of results are uploaded to this repository.
