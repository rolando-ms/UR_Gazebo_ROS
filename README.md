## UR Gazebo ROS

This project is used to control the ur-gazebo with ROS interface (beware that ur-gazebo uses python 2). It contains a feature to control the joints of the robot using keyboard inputs.

## Basic Dependencies:
* Ubuntu 18.04
* Anaconda for linux

## Anaconda for Linux

If Anaconda is not installed, you can just follow the [linux installation instructions](https://docs.conda.io/projects/conda/en/latest/user-guide/install/linux.html) for Anaconda.

## Creating a Conda Environment

A python 2 environment is required for some libraries. If Anaconda is already installed, you can just type in a terminal:

```
conda create --name py27_ros python=2.7
```

and once it is created, you can activate it with:

```
conda activate py27_ros
```

## Installing ROS Melodic

For Ubuntu 18.04, the supported version version of ROS is *melodic*. To install ros melodic follow the instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu) (Please do until step 1.6; In 1.1 choose preferably **Desktop-Full Install**).

Once ros melodic is installed, [follow the instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) (step 3) to create a ROS workspace.


Also, to work with python, install rospkg by typing in a terminal inside your conda environment the following:

```
pip install rospkg
```

## Clone ros_industrial Repository

UR-gazebo is contained in the [ros-industrial repository](https://github.com/ros-industrial/universal_robot) (follow the instructions of **Building from source**; use the **melodic** distro; remember to use your own created catkin workspace from the previous section).

You can test the installation by following the instructions of **Usage with Gazebo Simulation** in the aforementioned ros-industrial repository.

## Update publish rate values

The python scripts publish data via ROS at a fixed rate (60 hz). Thus, it is necessary to update some values for the publishers.

* Go to your-ros-workspace/src/universal\_robot/ur\_gazebo
* In controller/arm\_controller\_ur10.yaml, update the following:
  - arm\_controller: state\_publish\_rate: 60
  - joint\_group\_position\_controller: state\_publish\_rate: 60
* In controller/joint\_state\_controller.yaml, update the following:
  - joint\_state\_controller: publish\_rate: 60
* In launch/controller\_utils.launch, update the value entry in robot state publisher as follows:
```
<param name="publish_frequency" type="double" value="60.0" />
```

## Install Pycharm Community Edition (Optional)

Install following the [linux instructions](https://www.jetbrains.com/help/pycharm/installation-guide.html#requirements).


## Using this Repository

* Clone this repository.
* (Optional) Install pycharm community edition following the [linux instructions](https://www.jetbrains.com/help/pycharm/installation-guide.html#requirements). Then create a new project (choose the option *previously configured interpreter* and select your conda environment python interpreter located in ~anaconda3/envs/your-ros-env/bin/python2.7). Once created, add the repository folder to the project.
* Replace your own ros workspace name in the paths: source ~/your-ros-workspace/devel/setup.bash
* Replace your own conda environment path and name: source /your-path/anaconda3/bin/activate; conda activate py27\_ros;
* If pycharm is installed
  - Replace your own paths for pycharm in the ROSBash.sh file
  - In the root folder of this repo, type:
  ```
  ./ROSBash
  ```
  - In pycharm, open your project containing the folder of this repository.
  - Run move\_ur.py and select *network control* to move the robot to its initial position. Then stop the script.
  - If isaac gym is transmitting the joint target values, you can run Goal\_Update or Goal\_Update\_with\_Client to publish into gazebo. 
* If pycharm is **not** installed
  - Comment or delete all lines after *&& sleep 5 &&*
  - In the root folder of this repo, type:
  ```
  ./ROSBash
  ```
  - Open a new terminal in the root repository folder and type:
  ```
  source ~/your-ros-workspace/devel/setup.bash
  conda activate py27_ros
  python move_ur.py
  ```
  - Select *network control* to move the robot to its initial pose. Then stop the script.
  - In the same terminal, if isaac gym is transmitting data, type:
  ```
  python Goal_Update.py

