# AutonomousDrones  
This is a project to establish and execute a successful surveillance algorithm using multiple drones.  

## System and Software Versions -  
	OS: Ubuntu 18.04  
  	Gazebo: gazebo-9  
  	ROS: ROS melodic  
	GCS: QGroundControl  
  	Python: python3.6.9 
	
## Installations -  

1. [Ubuntu](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/)    

2. [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)  
	Please note that installation link might be updated to a later version of Gazebo!  
	
3. [ROS](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros.md)

4. [ArduPilot](https://github.com/Suchit153/ardupilot)  
	Recommended: Fork and clone the above repo!  	

5. [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)  
	Please note that we only require the installation for Ubuntu Linux!  

## Additional Setup -  
In order to support a multi-drone framework, we need to make some additional changes -  

1. setup qgroundcontrol (QGC) to accept multiple vehicles -  
	* Open QGC
	* navigate to the settings tab and click on Comm Links  
	* Click Add  
	* Fill in the vehicle TCP port as [shown](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/imgs/qg_comms.png)  
	* Repeat for adding new vehicles on different TCP ports  
	* __Note:__ TCP ports for each vehicle are separated by a value of 100. Therefore, vehicle1 = 8100, vehicle2 = 8200, vehicle3 = 8300 ...
	
## Simulation Prerequisite -

* Make a catkin workspace in your home directory -  
  > $ mkdir -p ~/catkin_proj/src  
  > $ cd ~/catkin_proj/src  
  > $ git clone https://github.com/rbccps-iisc/AutonomousDrones.git  
  > $ cd ~/catkin_proj  
  > $ catkin build --cmake-args -DPYTHON_VERSION=3.6  
	
## Simulation Setup -

* Models
  * The models used in the environment are located in [models](https://github.com/rbccps-iisc/AutonomousDrones/tree/master/Simulation/models)  

* Worlds
  * The world files used for using these models in gazebo is located in [worlds](https://github.com/rbccps-iisc/AutonomousDrones/tree/master/Simulation/worlds)
  
* Launch Files
  * The launch files used for using these models in gazebo is located in [launch files](https://github.com/rbccps-iisc/AutonomousDrones/tree/master/launch)
  
* Scripts
  * The scripts for running simulations are located in [scripts](https://github.com/rbccps-iisc/AutonomousDrones/tree/master/Simulation/scripts)  
  * The scripts for running on the drone's on-board computer are located in [scripts](https://github.com/rbccps-iisc/AutonomousDrones/tree/master/Drone/scripts)  
  * Description of the functionality of each script in this directory is mentioned as a comment at the beginning of the script file.  
 
## Simulation Usage - 
Run each of the following steps in separate terminals - 

1. launch gazebo world:  
	> $ cd ~/catkin_proj/src/AutonomousDrones  
	> $ LaunchAPM.sh  
	
2. start ardupilot for multiple drones:  
	> $ cd ~/ardupilot/Tools/autotest  
	> $ ./multi-ardupilot.sh  
	
3. launch mavros for multiple drones:  
	> $ roslaunch iq_sim multi-am.launch  
	
4. start QGC:  
	> $ ./QGroundControl.AppImage  
	* In QGC, navigate to the settings tab and click on Comm Links. Then, connect all three vehicles.  
	
5. Running Scripts:  
	* Terminal 1  
		> $ cd ~/catkin_proj/src/AutonomousDrones/Simulation/scripts   
		> $ python3 drone1_mission.py  
	* Terminal 2  
		> $ cd ~/catkin_proj/src/AutonomousDrones/Simulation/scripts   
		> $ python3 drone2_mission.py  
	* Terminal 3  
		> $ cd ~/catkin_proj/src/AutonomousDrones/Simulation/scripts   
		> $ python3 drone3_mission.py  
	* Terminal 4  
		> $ cd ~/catkin_proj/src/AutonomousDrones/Simulation/scripts   
		> $ python3 gcs_control.py    

	



 

