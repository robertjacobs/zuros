/*
* depth.h
*
* Created on: Jan 15, 2014
* Author: Robert Jacobs
*/

#ifndef ZUROS_DEPTH_H_
#define ZUROS_DEPTH_H_

#include "ros/ros.h"

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

//For laser scanner functionality
#include <sensor_msgs/LaserScan.h>

class XtionToLaser
{
	private:
		ros::NodeHandle node_handle_;
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	protected:
		//Laser scanner topic
    ros::Subscriber points_subscriber_;
		ros::Publisher laser_publisher_;
    float min_range_;
    float max_range_;
    float h_degrees_;

	public:
		XtionToLaser(ros::NodeHandle nh, double min_range, double max_range, double h_degrees);
		void init();
		void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
};


#endif /* ZUROS_DEPTH_H_ */

