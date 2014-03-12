#include "ros/ros.h"
#include "zuros_depth/depth.h"

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
    ros::init(argc, argv, "zuros_depth_depth_cpp");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */

    // This nodehandle functions in the local namespace (hence the tilde)
    ros::NodeHandle n("~");
    double min_range = -1;
    double max_range = -1;
    double h_degrees = -1;

    n.getParam("min_range", min_range);
    n.getParam("max_range", max_range);
    n.getParam("h_degrees", h_degrees);

    if(min_range == -1)
    {
      ROS_ERROR("[zuros_depth] : You did not set the min_range parameter. Using default xtion value (min_range: 0.8)");
      min_range = 0.8;
    }

    if(max_range == -1)
    {
      ROS_ERROR("[zuros_depth] : You did not set the max_range parameter. Using default xtion value (max_range: 3.5)");
      max_range = 3.5;
    }

    if(h_degrees == -1)
    {
      ROS_ERROR("[zuros_depth] : You did not set the h_degrees parameter. Using default xtion value (h_degrees: 58.)");
      h_degrees = 58.;
    }

    // create an instance of the subscriber class
    XtionToLaser xtionToLaser(n, min_range, max_range, h_degrees);

    // initialize the subscribers (for details see comments in class)
    xtionToLaser.init();

    /**
    * ros::spin() will enter a loop, pumping callbacks. With this version, all
    * callbacks will be called from within this thread (the main one). ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
    ros::spin();

    return 0;
}

