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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//For laser scanner functionality
#include <sensor_msgs/LaserScan.h>

class XtionToLaser
{
	private:
		ros::NodeHandle node_handle_;

		// messages
		image_transport::ImageTransport* it_;
		image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
		message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
		message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	protected:
		//Laser scanner topic
		ros::NodeHandle node_;
		ros::Publisher laser_publisher_;

	public:
		XtionToLaser(ros::NodeHandle nh) : node_handle_(nh);
		~XtionToLaser();

		void convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);
		void inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
};


#endif /* ZUROS_DEPTH_H_ */
