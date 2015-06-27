# roomba_500driver_meiji
Adaptation of original [roomba_500driver_meiji](http://en.osdn.jp/projects/meiji-ros-pkg/) to ROS catkin system, originally developed by Autonomous Mobile Systems Laboratory at the Meiji University, Japan.

# Build
1. Clone this repository to your `src` directory of catkin workspace.
2. Install dependancy with `rosdep install roomba_500driver_meiji`
3. Execute `catkin_make` from the top of your catkin workspace.

# Usage
1. Start your ROS system (`roscore`).
2. Connect your computer and roomba with serial cable.
3. Execute `rosrun roomba_500driver_meiji roomba_500driver_meiji`. Status message will be shown.
