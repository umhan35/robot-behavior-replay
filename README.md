# Robot Behavior Replay Implementation

This repo contains sample code for the AI-HRI 2022 paper of "[Mixed-Reality Robot Behavior Replay: A System Implementation](https://zhaohanphd.com/publications/aihri22-mixed-reality-robot-behavior-replay-a-system-implementation/)" by Zhao Han, Tom Williams, and Holly A. Yanco.

This ROS package was tested with ROS Melodic on Ubuntu 18.04.

3.6 is the MongoDB version in Ubuntu 18.04's repo: https://packages.ubuntu.com/bionic/mongodb.

In addition to the MongoDB related code based on the [mongodb_store](https://github.com/strands-project/mongodb_store) project, you can find the code we used for Figure 2-4:

 - Code: [record_the_replay.cpp](record_the_replay.cpp)
 - Behavior trees:
   - [behavior trees/replay.xml]()
   - [behavior trees/go pick gearbox bottom.xml]()
   - [behavior trees/go place gearbox part.xml]()

## Requirement

 - Robot with ROS support (e.g., [Fetch](https://docs.fetchrobotics.com/))
- Ubuntu 18.04 ([Official Ubuntu wiki page](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview))
- ROS Melodic ([Install instruction](https://wiki.ros.org/melodic/Installation/Ubuntu))
- A catkin workspace ([Official ROS tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace))

## Setup

### Install the system mongodb 3.6 package
```bash
sudo apt-get install python-pymongo mongodb mongodb-dev
```

(There is also a Mongodb GUI software: Compass. You can download it here: https://www.mongodb.com/try/download/compass)

### Clone and build mongodb_store

Here we will use our fork to [strand project's `mongodb_store`](https://github.com/strands-project/mongodb_store), which is
https://github.com/uml-robotics/mongodb_store. Note that the changes including a fix that makes the work.

To clone it in your catkin workspace's source directory

First cd into your workspace.
```bash
roscd && cd ../src 
```
Then download the strands project's mongodb_store

```bash
git clone https://github.com/strands-project/mongodb_store
```

Install Python dependencies
```bash
pip install --user future
```

Run `catkin build` to build the ROS packages in the `mongodb_store` directory.
```bash
catkin build mongodb_log mongodb_store mongodb_store_msgs
```

## Running

Start `mongodb_store`
```bash
roslaunch mongodb_store mongodb_store.launch host:=localhost use_daemon:=true port:=27017
```

`mongodb` has started as a systemd service when you install it using apt.

To check the status of the service
```bash
sudo service mongodb status
```

To stop
```bash
sudo service mongodb stop
```

## Storing, Querying, and Playing arbitrary topics using the mongodb_log package

Store topics by running
```bash
rosrun mongodb_log mongodb_log.py /topic_name_one /topic_name_two
```
Or all topics 
```bash 
rosrun mongodb_log mongodb_log.py -a
```
For topics that do not update at a constant rate add the argument `--no-specific`
```bash
rosrun mongodb_log mongodb_log.py --no-specific /topic_name_one
``` 
The `--no-specific` argument disables a fast C++ logger needed for constantly updated topics. This logger might cause 
errors when running mongodb_log for topics that do not require a fast C++ logger, and as such it is recommend to disable
that option.  

Play back the topics by running
```bash
rosrun mongodb_store mongodb_play.py /topic_name_one /topic_name_two
```

Or all topics
```bash
rosrun mongodb_store mongodb_play.py
```

### Storing Topics via MongoDBLogger Action

The MongoDBLogger Action acts as a regular action. MongoDBLogger.action is composed of the following:
```text
# Takes in a string of topics to be logged separated by a colon ':', i.e. "/counter:/clock"
std_msgs/String topics
---
# The result will be all the topics logged during the duration of the action.
std_msgs/String topics_logged
---
# feedback will be composed of all the messages received
std_msgs/String msgs_received
```
To have the action server running run 
```bash
rosrun mongodb mongodb_logger_action_server.py
```
#### API

```C++
#include <mongodb/MongoDBLoggerAction.h>

typedef actionlib::SimpleActionClient<mongodb::MongoDBLoggerAction> Client;

 Client client("mongodb_logger", true);

 mongodb::MongoDBLoggerGoal goal;
 //...put data in goal...
 client.sendGoal(goal);
 //...play what needs to be recorded...
 client.cancelGoal();
```

#### Add it via cmake
You'll need to build mongodb to use the Action Server.
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  mongodb
)
```
If your ROS package is also a ROS library, you also need to add `mongodb` to `package.xml` and the `catkin_package` block in `CMakeLists.txt`.
 
### Play back messages recorded before or after a certain time
Note: messages play from **newest to oldest**

Usage: mongodb_play.py \[options] \[TOPICs...]

Options: -s for starting time, and -e for ending time

Time format: "day/month/year hour:minute"

#### Examples

To have the playback stop at August 7th, 2019 5:25 pm (plays every message recorded after that time)
```bash
rosrun mongodb_store mongodb_play.py -e "07/08/19 17:25" /topic_name_one /topic_name_two ...
```

To have the playback start at August 7th, 2019 5:25 pm (plays every message recorded before that time)
```bash
rosrun mongodb_store mongodb_play.py -s "07/08/19 17:25" /topic_name_one /topic_name_two ...
```
Can also be combined
```bash
rosrun mongodb_store mongodb_play.py -s "07/08/19 17:25" -e "01/08/19 09:30" /topic_name_one /topic_name_two ...
```

### [Example used in the paper] Play back Caddy Manipulation Motions in Gazebo

Note that example needs a proper setup of this repo: https://github.com/uml-robotics/fetchit2019.

For simulations specific for caddy_manipulation that require playing back topics stored in mongodb, please launch gazebo via this command

```bash
roslaunch mongodb caddy_manipulation_gazebo_remap.launch
```
This launch file remaps the `/clock` topic to `/clock_temp`. This allows for no clock synchronizing issues when it comes
to replaying previous topics. 

For any situation that requires the `/clock_temp` topic to be remapped back (such as setting up the caddy_detector) 
please run this on another terminal window:

```bash
rosrun mongodb undo_gazebo_clock_remap
```
Since replaying the nodes does not allow integrated measurements of certain parameters, we will have to set those
parameters during each run of mongodb_log.py and mongodb_play.py. Please launch the config manager with this command 
before running mongodb_log and mongodb_play. The mongodb_store server will have to be running in order for this to work.
```bash
rosrun mongodb_store config_manager.py _defaults_path:=pkg://mongodb/defaults
```

#### Examples

To store the caddy picking motion during simulation
```bash
rosrun mongodb_store config_manager.py _defaults_path:=pkg://mongodb/defaults
```
```bash
rosrun mongodb_log mongodb_log.py --no-specific /arm_with_torso_controller/follow_joint_trajectory/goal /gripper_controller/gripper_action/goal
```
To replay the caddy picking motion
```bash
roslaunch mongodb caddy_manipulation_gazebo_remap.launch
```
```bash
rosrun mongodb_store config_manager.py _defaults_path:=pkg://mongodb/defaults
```
```bash
rosrun mongodb_store mongodb_play.py /arm_with_torso_controller/follow_joint_trajectory/goal /gripper_controller/gripper_action/goal
```
