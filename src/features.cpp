#include <features.hpp>

namespace terrainFeature
{

Feature::Feature() : flatThreshold_(M_PI / 180), steepThreshold_(60 * (M_PI / 180)) {
}

Feature::~Feature() {
}

void Feature::computeSlopeCost(double &cost, costmap::TerrainData &terrainParameters) {

	double slope = fabs(acos((double) terrainParameters.normal(2)));
	if(slope < flatThreshold_) {
		cost = 0;
	}
	else if(slope < steepThreshold_) {
		cost = -log(1-(slope - flatThreshold_)/(flatThreshold_- steepThreshold_));
		if(maxCost_ < cost)
			cost = maxCost_;
	}
	else {
		cost = maxCost_;
	}
}

}
