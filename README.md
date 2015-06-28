# roomba_500driver_meiji
Adaptation of original [roomba_500driver_meiji](http://en.osdn.jp/projects/meiji-ros-pkg/) to ROS catkin system, originally developed by Autonomous Mobile Systems Laboratory at the Meiji University.

# Build
1. Clone this repository to your `src` directory of catkin workspace.
2. Install dependancy with `rosdep install roomba_500driver_meiji`
3. Execute `catkin_make` from the top of your catkin workspace.

# Usage
1. Start your ROS system (`roscore`).
2. Connect your computer and roomba with serial cable.
3. Execute `rosrun roomba_500driver_meiji roomba_500driver_meiji`. Status message will be shown.

# Note
You can use this repository (together with [roomba_teleop_meiji](https://github.com/mthrok/roomba_teleop_meiji)) to control not only roomba 500 series
but also roomba 600 series and iRobot Create 2. (I have only tested with iRobot Create 2).
The API, however is not updated, and which means only
functionalities common to both 500 series and 600 series are currently available.

For the difference of 500 series and 600 series,
please refer to the Appendix of [iRobot® Create® 2 Open Interface (OI)](http://www.irobot.com/~/media/MainSite/PDFs/About/STEM/Create/create_2_Open_Interface_Spec.pdf)

# For the original adaptation of Roomba_500driver_meiji without any modification by me, checkout [`original` branch](https://github.com/mthrok/roomba_500driver_meiji/tree/original).