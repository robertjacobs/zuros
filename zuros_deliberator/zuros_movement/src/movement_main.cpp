#include "ros/ros.h"
#include "zuros_movement/movement.h"

int main(int argc, char **argv)
{
        /**
		* The ros::init() function needs to see argc and argv so that it can perform
		* any ROS arguments and name remapping that were provided at the command line. For programmatic
		* remappings you can use a different version of init() which takes remappings
		* directly, but for most command-line programs, passing argc and argv is the easiest
		* way to do it. The third argument to init() is the name of the node.
		*
		* You must call one of the versions of ros::init() before using any other
		* part of the ROS system.
		*/
        ros::init(argc, argv, "movement_node");

        /**
		* NodeHandle is the main access point to communications with the ROS system.
		* The first NodeHandle constructed will fully initialize this node, and the last
		* NodeHandle destructed will close down the node.
		*/
        ros::NodeHandle n;

        // create an instance of the subscriber class
        Movement movement(n);

        // initialize the subscribers (for details see comments in class)
        movement.init();

		// spin will enter a loop which keeps sending the joystick values while ROS is running
		movement.spin();

        return 0;
}
