# Experimental Robotics Laboratory / assignment 1

The package contains the nodes and the simulation environment for simulating a Cluedo game in a limitied conditions.

The packege implement a simple architecture of the algorithm of this game.

## Description 
### Software Architecture
* This graph shows the Component Viewpoint of the software architecture to highlight atomic blocks interfaces.
![ROS nodes](../master/image/ros_diagram2.png)


### Temporal Diagram
* This sequence diagram is designed in order to show object interactions arranged in time sequence. It depicts the objects involved in the scenario and the sequence of messages exchanged between the objects needed to carry out the functionality of scenario.
![Sequential Graph](../master/image/sequence.png)


### State Diagram
* Finit State Machine is shown in the graph below.
* There are 3 states:
1. Check consistence (for hypothesis)
2. Validate Hypothesis
3. Stop

* Besides, there are 3 events:
1. start/stop
2. True
3. False

![State Graph](../master/image/FMS.png)

Note: 
* Inside the state 'Check consistence', there could be substates such as; 'goToPoint' and 'receiving Hints'.

## Instruction how to run the code:

1. Open a terminal (ROS sourced) then execute
```
roslaunch exp_rob_a1 sim.launch
```
2. After that run
```
rosrun exp_rob_a1 s_machine
```

Note: 
* I did not know why if I run the `state_machine` in the launch file the simulation fails.

## Working Hypothesis and environment
### System's feature
The system concerns of:
* the implementation of a behavioral architecture.
* the representation of a map with a suitable level of abstraction.
* the usage of the Cluedo ontology to manage hypothesis.
* the random-based generation of hints and the validation of hypothesis.
* the simulated motion in the environment.
* Robot is a point, rooms are also represented by point positions.

Note:
* The system works in ROS environment.
* Python must be installed.

### System's limitations
* There is no interface to interact with this system (from user's point of view), which means that the user cannot start, stop or restart the simulation once the nodes are executed.
* The database (hints) are limited. And it must inserted manually not by reading a file.
* The simulation environment is not clear. For example, the room divisions are not displayed in the 2d environment.

### Possible improvements
* Adding a user interface (start/stop/restart).
* Convert the `goPoint` service server to simple action server.
* Using ARMOR can give more flexibility for accessing and modifying ontological databases.
* Display rooms in 2d environment. 

