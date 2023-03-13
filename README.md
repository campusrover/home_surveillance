# Robot Convoy

## Team Members
1. Jongsuh Lee (James)

## Project Description

On program execution, the robots will:
1. elect a leader amongst themselves (Leader Election Phase);
2. organize themselves in a line (Convoy Formation Phase); and
3. proceed in linear fashion to the destination (Traveling Phase).

Throughout the demonstration, various unscripted events may occur, forcing the robots to repeat steps 1-3, or some subset thereof. For instance:
1. the leader will be deemed unhealthy, through:
    1. real-time termination of relevant nodes, or
    2. the robot itself being shut-down;
2. the leader will be deemed unqualified, by:
    1. straying too far from the target destination (e.g., by being radically relocated by human intervention), or
    2. having too-low battery levels;
3. communication failure between robots will be simulated;
4. a member of the convoy will be deemed unhealthy or unqualified (much like the leader might in cases 1 and 2); and
5. a robot formerly ejected from the convoy will regain health or be re-qualified, such that it must be re-integrated back into the convoy, with a former leader coming back online being a special case.

The events will either trigger a repeat from the Leader Election Phase in the case of #1-#3, or from the Convoy Formation Phase, in the case of #4 and #5.

The MRS will account for a subset or proper superset of the events in #1-#4, depending on the difficulty of the project. The goal is to have a distributed MRS that is as
robust as possible and resilient to component failures, communication failures, etc.

## Learning Goals

1. Learn Raft's Leader Election Algorithm, and maybe proceed to log replication as well, and use it to effect coordinated behaviors among the various robots.
2. Develop a high degree of proficiency with robot localization and navigation using the tf2 package, map servers, etc.

## How I would like to be evaluated.

1. Success in leveraging ROS's distributed system of topics, services, and actions to implement Raft.
2. Usage of Raft's leader election and log replication algorithms to make the robots do something interesting.
3. Mastery of robot localization and navigation using the map_server, the tf package, AMCL, and etc.