# Multi-Robot Home Surveillance

## Team Members
1. Jongsuh Lee (James)

## Video
[See the robots in action!](https://drive.google.com/file/d/1UgOknVUCbNmY5NiatFo7d72_akHZG5z7/view?usp=share_link)

## How To Run

There are two points of entry to the program:
1. `sim_main.launch`; and
2. `real_main.launch`.

The first runs the simulation of the program. The second is meant to work with the 4 turtlebot3s that are named `roba`, `robb`, `robc`, and `rafael` respectively.

'OAST' refers to 'On a separate terminal', and 'OANTW' means 'On a new terminal window' below.

For both branches:
1. OAST exectue `roscore`
2. OANTW execute `rosrun home_surveillance lea_display.py`

For the simulation branch:

3. OAST, execute `roslaunch home_surveillance sim_main.launch`.
4. You will see something like this on the window that is running `lea_display.py`: `leader: rafael // {'B': 'robc', 'C': 'roba', 'D': 'robb'}, 1`. It states that `rafael` is the leader robot, and that `robc` is patrolling zone `B`, `roba` is patrolling zone `C`, etc.
5. OANTW, execute `rosnode kill mp_<robot_name>` where `<robot_name>` stands for the name of the robot you want to kill. If you kill the leader, the robots will all stop and elect a new leader amongst themselves, who will then assign different zones to the rest of the robots. If you kill a follower, the killed robot will stop, and the system will maintain the invariant.

The invariant is: If we have n live robots, the first n zones in the list will be patrolled by at least one robot.

For the real branch:

3. OAST, ssh into each robot.
4. run `$(bru mode real) && $(bru name <robot_name> -m <ip_address_of_vpn_computer>) && multibringup`.
5. OAST execute `roslaunch home_surveillance real_main.launch`.
6. Follow step 5 of the simulation branch.

Note that for the system to work in real life: you need an area that is separated into four zones; the map file in `real_main.launch` has to set to a map of the area; the initial poses of the robots need to be properly set (you can do this via the launch file or the rviz interface); and the coordinates in `real_patrol`'s Zone class need to be properly set.
