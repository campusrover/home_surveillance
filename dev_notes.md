For leader resiliency:
1. keep track of the cur_leader for each node, as you did before.
2. have the patrol object subscribe to all topics of other_mps_leader_dead_publisher
3. if the patrol object receives the leader_dead message, it shuts down the action server.
4. if 1-3 works, remove the leader_shutdown event and do further cleanup


The leader will maintain two maps, a `guard_map` and `zone_map`. For any zone `z` and guard `g`, where `g` is responsible for `z`, `guard_map` will contain `<z,g>`, while `zone_map` will contain `<g,z>`. 

Whenever a follower robot `g` fails, the leader will check `z_missing = zone_map[g]` to see which zone the robot was responsible for. It will then iterate through `Zones.priority_list` in reverse order, and for the first zone `z_present` in the list for which `g == guard_map[z_present]` is not `None`, it will assign `g` to `z_missing`. 