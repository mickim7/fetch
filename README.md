# Fetch Gazebo Demo
Link to our demo: https://youtu.be/FAeerBthu1k

### DISCLAIMERS
* Do NOT use virtual box with a Mac OS
  * Virtual box on mac laptops tends to crash when Ubuntu, ROS, and Gazebo are installed 
* Start with a wiped computer that can run Ubuntu OS
* We used Ubuntu 18.04.4
  * You might need to be in root mode to install most of the software
  * Run sudo su in the terminal to get to root mode 
  * Or use [this](https://phoenixnap.com/kb/how-to-create-sudo-user-on-ubuntu )
 tutorial if you want to create a new sudo user on Ubuntu

## INSTALLATION / SETUP 
1. ROS Melodic 
  * Use [this](http://wiki.ros.org/melodic/Installation/Ubuntu) tutorial to install 
2. Terminator (great tool for projects that require parallel running programs, use instead of Terminal)
  * Use [this](https://gnometerminator.blogspot.com/p/introduction.html) link to download Terminator 
  * Useful Shortcuts
    * ctrl+shift+o splits horizontally
    * ctrl+shift+e splits vertically
    * crtl+shift+w closes current terminal
    * crtl+shift+n switches to next terminal
3. ros_gazebo (Gazebo 9)
  * Run `sudo apt get install gazebo9`
  * Gazebo is a simulation environment compatible with many robot models including Fetch 
4. Catkin 
  * http://wiki.ros.org/catkin (sudo apt-get install ros-melodic-catkin)
  * http://wiki.ros.org/catkin/Tutorials/create_a_workspace 
  * http://wiki.ros.org/catkin/Tutorials/CreatingPackage
  * Catkin is the official build system of ROS 
  * Your packages will live in your catkin workspace (catkin_ws)
5. Packages
  * Each package should have EXACTLY one .xml file. 
  * A package CANNOT be created inside another package.
  * http://wiki.ros.org/catkin/Tutorials/CreatingPackage
6. Rviz
  * Rviz shows you what the robot is seeing
  * You can change way robot sees in Rviz (Image/Image Topic)
  * http://wiki.ros.org/rviz/UserGuide

 ## RECREATING OUR WORK
**replace package_name!**
Below is a set of instructions specifying exactly where you should clone files from our Github so that they are in the correct locations.  

1. Create a scripts file, which will hold the programs that control the robot
 * `cd ~/catkin_ws/src/package_name`
 * `mkdir scripts`
2. Copy files from our git repo into the different destinations
 * `cd ~/fetch`
 * `cp demo.py ~/catkin_ws/src/package_name/scripts/demo.py`
 * `cp odometry.py ~/catkin_ws/src/package_name/scripts/odometry.py`
 * `cp demo.launch ~/catkin_ws/src/package_name/launch/demo.launch`
 * `cp playground.launch /opt/ros/melodic/share/fetch_gazebo/launch/playground.launch`
 * `cp empty_world.launch /opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch`
 * `cp costmap_common.yaml /opt/ros/melodic/share/fetch_navigation/config/costmap_common.yaml`
 * `cp twotable_world.sdf  ~/catkin_ws/src/package_name/scripts/twotable_world.sdf`
3. To run a demo with a custom created cart in the world, copy these files`
 * `cp /cart/model.sdf ~/.gazebo/models/cart_front_steer/model.sdf`
 * `cp /cart/cart_demo.py ~/catkin_ws/src/package_name/scripts/demo.py`
 * `cp /cart/cart_world.sdf ~/catkin_ws/src/package_name/scripts/cart_world.sdf`
 * `cp /cart/playground.launch /opt/ros/melodic/share/fetch_gazebo/launch/playground.launch`
4. Make the files executable 
 * `chmod +x demo.py`
 * `chmod +x odometry.py`
5. Edit launch file
 * open `/opt/ros/melodic/share/fetch_gazebo/launch/playground.launch` and edit "scmk_demos" to your package name
6. Running our demo (run each command in new terminal)
 * `roslaunch fetch_gazebo playground.launch`
 * `rosrun package_name odometry.py`
 * `roslaunch package_name demo.launch`
 * `rosrun rviz rviz -d /opt/ros/melodic/share/fetch_gazebo_demo/config/demo.rviz`

## DESCRIPTIONS OF FILES
Below we have the directory paths for each of the files in our project, a brief description of the purpose of the files, and the specific modifications that we made for our project.

1. `~/catkin_ws/src/package_name/launch/demo.launch`
  * This is the [launch file](http://wiki.ros.org/roslaunch) for the demo. Running a launch file runs multiple commands at once. 
  * This launch file starts navigation, MoveIt, perception, and then our demo.py program.
  
2. `~/.gazebo/models/cart_front_steer/model.sdf`
 * Gazebo models are created using an [sdf file](http://sdformat.org/spec?ver=1.7&elem=model) 
 * For our demo, we edited the cart_front_steer model from gazebo to have a long rod rigidly attached to the front of the cart. 
 * To do this, we added a link and a joint, where the joint is of type "fixed".
 * Useful equations for [moments of inertia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia) 
3. `/opt/ros/melodic/share/fetch_gazebo/launch/playground.launch`
 * The playground.launch file starts gazebo and loads the specified world in the sdf file. 
 * We have edited the launch file to load cart_world.sdf, which is a world that has 2 cafe tables, a cube, and the cart that we created earlier.
4. `/opt/ros/melodic/share/fetch_navigation/config/costmap_common.yaml`
 * This file takes certain parameters for the navigation system. 
 * Here, we decreased the inflator radius so that the robot could get closer to obstacles without throwing a costmap warning.
5. `/opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch`
  * This is a launch file that is called by the playground.launch file (many launch files call other launch files, so you may need to follow the trail of launch files to see where things are executed). 
  * The empty_world.launch file sets many arguments for the gazebo world and creates the gazebo-related servers and clients.
 * In order to get ground-truth values for the position of the robot, we remapped  the "odom" topic to an unused "gazebo_odom" topic.  
 * This means that there is nothing publishing to the "odom" topic and we can now publish the ground-truth values through another program called odometry.py
6. `~/catkin_ws/src/scmk_demos/scripts/odometry.py`
 * We created our own odometry publisher to publish odometry messages to move_base because we wanted to use the Gazebo location data as ground truth instead of running localization.
7. `~/catkin_ws/src/scmk_demos/scripts/twotable_world.sdf`
 * We created and saved our own Gazebo world that has two tables and one cube
8. `~/catkin_ws/src/scmk_demos/scripts/cart_world.sdf`
 * We created and saved our own Gazebo world that has an instance of the cart with our added handle, two tables, and a cube
9. `~/catkin_ws/src/package_name/scripts/demo.py`
 * This is the main code for our demo. Here we are calling different functions and commanding the robot. 
 * Our robot travels to the first table, picks up the cube, travels to the second table, and drops the cube on it. 
 * Before running this demo, make sure the .sdf file linked in the playground.launch file is twotable_world.sdf 
10. `~/catkin_ws/src/package_name/scripts/cart_demo.py`
 * This is the main code for a demo we started but did not finish writing. 
 * Our goal was to have the robot pick up the cube from the first table, travel to the cart, put it on the cart, push the cart to the second table, and put the cube on the second table. 
**Before running this demo, change the .sdf file linked in playground.launch to cart_world.sdf
11. `~/catkin_ws/src/package_name/scripts/event_based.py`
 * A demo that moves different parts of the robot using event-based programming 


## TIPS AND TRICKS
Throughout our time working with ROS, Fetch, and Gazebo, we acquired a number of problem solving tools and tricks we used to solve certain issues we came across. Some of these tips are below: 

#### Resetting your world in gazebo 
When resetting your world inside the Gazebo interface, using file→ reset world might result in your Fetch robot doing a barrel roll and refusing to get up. Instead, use file→ reset model pose. This should bring the robot back to its original starting place. If you need to reset your ros environment because Gazebo glitches, run the command pkill -f ros in the terminal. 

#### Confused about which nodes are publishing where?
When in doubt, view the rqt graph! We found this to be one of our most useful debugging tools. While running your program, run the command rqt_graph which produces a dynamic graph mapping your system including all the nodes you are running and the topics/messages being passed between them. 

#### Odometry in Gazebo seems off 
You might want to try bypassing localization odometry and just use the Gazebo location data as ground truth. You will want to create a new node that publishes the Gazebo location data as odometry messages to the move_base node. In our github directory, the  odometry.py file is our node that publishes the Gazebo location data. If you do this and view the rqt graph, you might see that there are multiple odometry messages being published to move_base, so you now want to remap the gazebo odom messages to a useless topic so that the only odometry messages being published to move_base are the ones from the new node you created. You can do this in ~/opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch

#### Costmap warning issues
If you are receiving yellow warning messages about Costmaps such as “clearing both costmaps to unstuck robot,” you might be sending the robot to a “dangerous” location. The danger of a location is usually determined by its proximity to an object, door, or wall. Possible fixes include sending the robot to a less “costly” location (further away from objects) or decreasing the radius around the robot that is considered “costly.” This will allow the robot to get closer to objects without throwing warnings. You can do this by decreasing the inflator radius value in the file ~/fetch_nav/config/costmap_common.yaml

#### Orienting your robot’s base
When moving the robot, you should be specifying (x, y, theta) coordinates. In the playground.launch Gazebo environment, we found theta = 0 orients the robot towards the front wall, and theta = -pi/2 faces the robot to the right towards our second table. 

#### Manipulating the arm using clients 
There are multiple ways you can manipulate the arm of your Fetch robot. We suggest using the GraspingClient to locate/pick up objects and tuck. Additionally, you can use the GripperClient to open and close the gripper (good for dropping cubes). To use the former, you will need to import moveit_commander and sys at the top of your file. 

#### Understanding and modifying the structure of Gazebo objects
Objects in Gazebo such as tables, cubes, carts, etc. are composed of links and joints where the number of joints is always one less than the number of links, as links describe rigid bodies and joints join together links. For example, a cart with a flat chassis and four wheels has five links and four joints. You can add joints and links to objects, modifying an object’s .sdf file to suit your needs. See “Description of files” subsection “~/.gazebo/models/cart_front_steer/model.sdf” of this Markdown file for more information and tutorials on editing Gazebo objects. 

#### General Debugging Tips 
 1. Do not be afraid to modify the ROS/Gazebo environments. They are open source and not always perfect. 
 2. Run pwd in terminal to print the path of where you are 
3. If you don’t know where something is being executed (where Gazebo is being launched for example), go through the launch file. Launch files will call other launch files, so you may have to dig through multiple launch files to find the program that executes the functionality you want.
 4. As stated above, the rqt graph is your friend when you don’t know what’s going on.  
 5. Good places to start when stuck
  * http://docs.ros.org/ 
  * http://wiki.ros.org/ROS/Tutorials/
  * https://docs.fetchrobotics.com/
  * http://gazebosim.org/tutorials/?tut=ros_comm
 6. Robotics is hard and ROS is confusing. If you are out of options and have been stuck for a while, googling your error is not helping, don’t be afraid to email Professor Zucker. If you do this, describe your problem in detail along with the fixes you already tried. 




