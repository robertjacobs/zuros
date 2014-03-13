// ROS includes
#include <ros/ros.h>
#include <zuros_depth/depth.h>
#include <limits>

XtionToLaser::XtionToLaser(ros::NodeHandle nh, double min_range, double max_range, double h_degrees)
{
	node_handle_ = nh;
  min_range_ = min_range;
  max_range_ = max_range;
  h_degrees_ = h_degrees;
}

void XtionToLaser::init()
{
  // Subscribe to depth points topic
  //points_subscriber_ = node_handle_.subscribe("/camera/depth_registered/points", 100, &inputCallback);
	 
  points_subscriber_ = node_handle_.subscribe("/camera/depth_registered/points", 100, &XtionToLaser::inputCallback, this);

	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*/
	laser_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("scan", 10);
}

// Handles callback
void XtionToLaser::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*pointcloud_msg, *cloud);
	
	// publish to laser scanner topic
	// laser scanner format: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
	// Asus xtion specs: http://www.asus.com/Multimedia/Xtion_PRO_LIVE/#specifications
	sensor_msgs::LaserScan msg;

	// Fill message with data
	msg.header = pointcloud_msg->header;
	msg.header.frame_id = "camera_link";

	// Asus xtion angles: 58Â° H, 45Â° V, 70Â° D (Horizontal, Vertical, Diagonal)
  // Calculate opening
  // if h_degrees = 58 (xtion) opening is -29 to 29 with 0 as center
  double opening = h_degrees_ / 2;
	msg.angle_min = -29/180.*CV_PI;			// -29 .. 0 .. 29 for xtion
	msg.angle_max = 29/180.*CV_PI;				// -29 .. 0 .. 29 for xtion

  
	msg.angle_increment = 0.090625/180.*CV_PI;  // 2*alpha/640

	msg.time_increment = 0;		// sensor is not moving so value is 0
	msg.scan_time = 1/30;

	// Set min and max range of the sensor
	msg.range_min = min_range_;		  // default 0.8 meters
	msg.range_max = max_range_;  		// default 3.5 meters

	msg.ranges.resize(cloud->width, 0);
	msg.intensities;		// If your device does not provide intensities, please leave the array empty.

	// range we want to look at is 238 - 242 (5 pixels with 240 as center)
    float total;
    int total_count;
    for (unsigned int u=0; u<cloud->width; ++u)
    {
        total = 0;
        total_count = 0;
        for (unsigned int v=238; v<243; ++v)
        {
            pcl::PointXYZRGB& point = cloud->at(u,v);
            //matrix indices: row y, column x!

            if(point.z == point.z && point.x == point.x)    //test not a number
            {
                total += sqrt(point.z*point.z + point.x * point.x);
                ++total_count;
            }
        }
        float distance = total/(float)total_count;
        if (distance < min_range_)
            msg.ranges[u]= -std::numeric_limits<double>::infinity();
        else
            msg.ranges[u]=distance;
    }

	//publish on topic
	laser_publisher_.publish(msg);
}

