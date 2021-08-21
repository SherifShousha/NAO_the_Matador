## Launch the code
1. Launch ROS master with `$ roscore`
2. Run camera load with `$ rosrun bull camera_node.py`
3. Because the LED module can only be accessed in sudo privilege. So the main node can only run as Super User. 
   In sudo privilege, run the following command:
    `$ source </home/pi/.bashrc> `
    `$ rosrun bull main.py`


## Reference
The module libraries are provided by the robot car manufacturer Freenove. A few modifications are made to fit into our project.
https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi