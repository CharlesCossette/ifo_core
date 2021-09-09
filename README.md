# IFO Core
This repo consists of a collection of ROS packages running the core autonomy stack on Uvify IFO-S quadcopter. It is primarily intended to be contained with general-purpose functionality that is project-independent. This should be cloned on the agents themselves, as well as on a local computer when running SITL.

## Package Breakdown 

### controller
The primary interface with MAVROS. A user can send the controller waypoints to follow by publishing a `WaypointList` to the `controller/waypoints_in` topic.

### fsm (finite state machine)
Empty and unused, but we should use it when we get there.

### local_diagnostics
The purpose of this node is to provide aggregate diagnostic information from various nodes on a single agent. As such, this is where each node can report their overall health, whether they are initialized or not, and any other diagnostic information.

All nodes publish to the local_diagnostics/raw topic, which this node subcribesto. This node then publishes a summary of overall system health.

This node can be used as a mechanism to get nodes to "wait" for other nodes to be initialized before starting a procedure.

### ifo_common
This is primarily a python package which defined an abstract `IfoNode` that all nodes should inherit. An `IfoNode` provides an easy interface to the `local_diagnostics` node, and also automates periodic reporting.

## Tests to eventually implement somehow

- sudden mocap gap/failure
- trying to start a mission without mocap
- sudden discontinuity in mocap values