
# final_aerial_project
Eurecat UVIC master repository for the final delivery of the aerial project



## How to set up the project framework

### Install catkin tools

    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get install python-catkin-tools

### Install vcstool

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
    sudo apt-get update
    sudo apt-get install python3-vcstool


### Ros packages (Maybe not complete)
Install ros packages

    sudo apt-get install ros-<ros-distro>-lms1xx
    sudo apt-get install ros-<ros-distro>-controller-manager
    sudo apt-get install ros-<ros-distro>-base-local-planner
    sudo apt-get install ros-<ros-distro>-dwa-local-planner
    sudo apt-get install ros-<ros-distro>-joint-state-controller
    sudo apt-get install ros-<ros-distro>-gazebo-ros-control
    sudo apt-get install ros-<ros-distro>-diff-drive-controller
    sudo apt-get install ros-<ros-distro>-move-base
    sudo apt-get install ros-<ros-distro>-twist-mux
    sudo apt-get install ros-<ros-distro>-clear-costmap-recovery
    sudo apt-get install ros-<ros-distro>-nav-core
    sudo apt-get install ros-<ros-distro>-navfn
    sudo apt-get install ros-<ros-distro>-rotate-recovery
    sudo apt-get install ros-<ros-distro>-octomap
    sudo apt-get install ros-<ros-distro>-octomap-ros
    sudo apt-get install ros-<ros-distro>-octomap-msgs
    sudo apt-get install ros-<ros-distro>-geographic-msgs
    sudo apt-get install ros-<ros-distro>-mavlink
    sudo apt-get install ros-<ros-distro>-robot-localization
    
    sudo apt-get install libgeographic-dev
    sudo apt-get install libgoogle-glog-dev
    pip install future


###  ROS kinetic (and ubuntu 16.04 LTS):

Get the vcs repo file:

    wget https://raw.githubusercontent.com/AerialRobots/final_aerial_project/master/project_cfg/vcsFiles/kinetic.repos

Import the repos

    vcs import < kinetic.repos

Run the python script located in 

    python3 final_aerial_project/project_cfg/scripts/kineticUpdates.py


### Ros melodic (and ubuntu 18.04 LTS):

Get the vcs repo file:

    wget https://raw.githubusercontent.com/AerialRobots/final_aerial_project/master/project_cfg/vcsFiles/melodic.repos

Import the repos

    vcs import < melodic.repos

Run the python script located in 

    python3 final_aerial_project/project_cfg/scripts/melodicUpdates.py

### Compile everything

Inside the workspace folder run:

    catkin build






