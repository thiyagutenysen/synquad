## Modules
There are three main modules in the code:
- Walking controller
- Robot module
- Communication


### Walking controller
This module generate the foot trajectories to follow based on the inputs to the system. The foot position
is provided in the Cartesian co-ordinates.

### Robot module
This module takes the input of the foot position and converts it to the joint angles to send to the robot.

### Communication module
This module handles the communication with the Tiva board. It handles the communication and data exchange 
protocol.


## Compilation

To comile the code run:
```
$ make
```
This will create an application named 'app'. Run this program using:
```
$ ./app
```



