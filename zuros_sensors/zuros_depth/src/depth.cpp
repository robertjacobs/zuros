// ROS includes
#include <ros/ros.h>
#include <zuros_depth_sensor/depth.h>

XtionToLaser::XtionToLaser(ros::NodeHandle nh)
{
	node_handle_ = nh;
}

// Destructor
XtionToLaser::~XtionToLaser()
{
	if (it_ != 0)
        delete it_;
	if (sync_input_ != 0)
    	delete sync_input_;
}

void XtionToLaser::init()
{
	it_ = 0;
	sync_input_ = 0;

	//Subscribe to image color and depth points
	it_ = new image_transport::ImageTransport(node_handle_);
	colorimage_sub_.subscribe(*it_, "/camera/rgb/image_raw", 1);
	pointcloud_sub_.subscribe(node_handle_, "/camera/depth_registered/points", 1);

	// Synchronize
	sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
	sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
	sync_input_->registerCallback(boost::bind(&XtionToLaser::inputCallback, this, _1, _2));

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
	*
	* The first parameter defines the data type of the messages that become published.
	* Here it is a string. Find all data types and instructions how to create
	* your own messages at ros.org.
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	laser_publisher_ = node_.advertise<sensor_msgs::LaserScan>("scan", 10);
}

void XtionToLaser::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
	    image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("KinectLaserScanner: cv_bridge exception: %s", e.what());
	}
	
	image = image_ptr->image;
}

// Handles callback
void XtionToLaser::inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
     // convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*pointcloud_msg, *cloud);

	//compute depth_image: greyvalue represents depth z
	cv::Mat depth_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32FC1);

	for (unsigned int v=0; v<cloud->height; v++)
	{
		for (unsigned int u=0; u<cloud->width; u++, u++)
		{
			//matrix indices: row y, column x!
			pcl::PointXYZRGB& point = cloud->at(u,v);
			if(point.z == point.z)	//test nan
				depth_image.at< float >(v,u) = point.z;

		}
	}
	
	// publish to laser scanner topic
	// laser scanner format: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
	// Asus xtion specs: http://www.asus.com/Multimedia/Xtion_PRO_LIVE/#specifications
	sensor_msgs::LaserScan msg;

	// Fill message with data
	msg.header = pointcloud_msg->header;
	msg.header.frame_id = "camera_link";

	// Asus xtion angles: 58° H, 45° V, 70° D (Horizontal, Vertical, Diagonal)
	msg.angle_min = -29./180.*CV_PI;			// -29 .. 0 .. 29
	msg.angle_max = 29./180.*CV_PI;				// -29 .. 0 .. 29
	msg.angle_increment = 0.090625/180.*CV_PI;  // 2*alpha/640

	msg.time_increment = 0;		// sensor is not moving so value is 0
	msg.scan_time = 1/30;			// ?????

	// Asus xtion Distance of Use Between 0.8m and 3.5m
	msg.range_min = 0.5;		// 0.5 meters
	msg.range_max = 8.5;  		// 3.5 meters

	msg.ranges.resize(cloud->width, 0);
	msg.intensities;		// If your device does not provide intensities, please leave the array empty.

	// range we want to look at is 238 - 242 (5 pixels with 240 as center)
	float total;
	for (unsigned int u=0; u<cloud->width; ++u)
	{
		total = 0;
		for (unsigned int v=238; v<243; ++v)
		{
			pcl::PointXYZRGB& point = cloud->at(u,v);
			//matrix indices: row y, column x!

			if(point.z == point.z && point.x == point.x)	//test not a number
				total += sqrt(point.z*point.z + point.x * point.x);
		}
		msg.ranges[u]=total/5.f;
	}

	//publish on topic
	laser_publisher_.publish(msg);

	//visualization on screen
	cv::Mat depth_im_scaled;
	cv::normalize(depth_image, depth_im_scaled,0,1,cv::NORM_MINMAX);
	//cv::imshow("depth_image", depth_im_scaled);
	//cv::waitKey(10);
	//cv::imshow("color_image", color_image);
     //cv::waitKey(10);
}


