#ifndef INCLUDE_FEATURE_HPP_
#define INCLUDE_FEATURE_HPP_

#include <data.hpp>
#include <Eigen/Dense>

namespace terrainFeature
{

class Feature {

	public:
		Feature();

		~Feature();

    	void computeSlopeCost(double &cost, costmap::TerrainData &terrainParameters);

	private:
		double flatThreshold_, steepThreshold_;
		double maxCost_ = 1; 
};

}
#endif  //  INCLUDE_FEATURE_HPP_