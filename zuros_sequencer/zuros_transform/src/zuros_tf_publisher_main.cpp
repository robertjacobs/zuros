#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "zuros_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

	while(n.ok())
	{
		broadcaster.sendTransform(
      	tf::StampedTransform(
		
		// The base_laser is mounted 27.45 cm in X direction and -7.75cm in Z direction from base_link
		// Since it is mounted in the middle of the base_link there is no offset in the Y direction and therefore 0
        	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2745, 0.0, -0.0775)),
        	ros::Time::now(),"base_link", "base_laser"));
    		r.sleep();
  	}
}
