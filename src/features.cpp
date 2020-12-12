#include <features.hpp>
#include <ros/ros.h>

namespace terrainFeature
{

Feature::Feature() : flatThreshold_(8 * (M_PI / 180)), steepThreshold_(60 * (M_PI / 180)) {
}

Feature::~Feature() {
}

void Feature::computeSlopeCost(double &cost, costmap::TerrainData &terrainParameters) {

	double slope = fabs(acos((double) terrainParameters.normal(2)));
	ROS_INFO_STREAM(slope);
	if(slope < flatThreshold_) {
		cost = 0;
	}
	else if(slope < steepThreshold_) {
		cost = -log(1-(slope - flatThreshold_)/(steepThreshold_ - flatThreshold_));
		// ROS_INFO_STREAM(cost);
		if(maxCost_ < cost)
			cost = maxCost_;
	}
	else {
		cost = maxCost_;
	}
	ROS_INFO_STREAM(cost);
}

// void Feature::computeHeightDeviationCost(double& cost, costmap::TerrainData &terrainParameters) {

// }

}
