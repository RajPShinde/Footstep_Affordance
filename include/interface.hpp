#ifndef INCLUDE_INTERFACE_HPP_
#define INCLUDE_INTERFACE_HPP_

#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <footstepAffordance.hpp>


namespace footstepAffordance
{

class Interface{
	
	public:

	Interface(ros::NodeHandle& nh);

	~Interface();

	void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

	void getRobotState(Eigen::Vector3d &robotStateXYZ, Eigen::Vector3d &robotStateRPY, const octomap_msgs::Octomap::ConstPtr& msg);

	private:


};
}

#endif  //  INCLUDE_INTERFACE_HPP_