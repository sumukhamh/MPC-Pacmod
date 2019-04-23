# Autoware Pacmod Interface
The interface to convert the messages from the Autoware to messages understood by the PacMod. This is vehicle specific and done for the GEMe6.

# Instructions to build the package
1. Create a ros workspace 
```
source /opt/ros/lunar/setup.bash
mkdir -p ~/interface_ws/src
cd ~/interface_ws/
catkin_make
echo "source ~/interface_ws/devel/setup.bash --extend" >> ~/.bashrc
```
2. Clone the autoware_pacmod_interface repository
```
cd ~/interface_ws/src
git clone git@github.com:CogRob/autoware_pacmod_interface.git
```
3. Build the package
```
cd ~/interface_ws/
catkin_make
```

# Instructions to run the package
1. Launch the pacmod 
```
roslaunch pacmod_game_controller pacmod_game_controller.launch
```
2. Launch the interface node
```
roslaunch autoware_pacmod_interface autoware_pacmod_interface.launch
```
3. Start up Autoware, go to the Computing tab and switch on twist_filter node under waypoint_follower. 
4. Go to the interface tab and change the gears to see if the car is responding or do a ```rostopic echo```.
The commands should be sent to the PacMod.
