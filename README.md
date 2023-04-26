# UAM_FORCE_CONTROL
- ros package containing admittance node

## Docker container startup 

- before starting a docker container run following command -  command opens up the server to connections from any remote machine (X server is responsible for managing the graphical display) - without this you won't be able to see Gazebo simulation

````bash
xhost +
````
- start the docker container
````bash
docker start -i uav_ros_simulation_focal
````


## Usage

### Terminator

- access a running docker container and start an interactive bash session inside it in each terminal window

````bash
docker exec -it uav_ros_simulation_focal bash
````


````bash
# 1st terminal - simulation Startup
bash start.sh

# 2nd terminal - starting the force control node
cd
cd uav_ws/src/uam_force_control/launch
roslaunch admittance_node.launch

# 3rd terminal - starting the plotjuggler
rosrun plotjuggler plotjuggler

#4th terminal - publish position on red/tracker/input_pose
rostopic pub /red/tracker/input_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'base_link'
pose:
  position:
    x: 0.0
    y: -3.0
    z: 2.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
````

## Simulation parameters

- edit simulation parameters - [AdmittanceControl.cpp](https://github.com/Vujo82/uam_force_control/blob/master/src/AdmittanceControl.cpp)

*K* - stiffness coefficient

*M* - inertia coefficient

*D* - damping coefficient

*HISTORY_BUFFER_SIZE* - median/mean kernel size

## Apply changes

- position yourself in uav_ws
````bash
cd 
cd uav_ws
````
- run a catkin build
````bash
catkin build
````
- after successful run
````bash
source devel/setup.bash
````

