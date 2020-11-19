#include <footstepAffordance.hpp>

namespace costmap
{

FootstepAffordance::FootstepAffordance() {

}

FootstepAffordance::~FootstepAffordance() {

}

void FootstepAffordance::run(octomap::OcTree* octomap, 
							const Eigen::Vector3d& robotStateXYZ, 
							const Eigen::Vector3d& robotStateRPY, 
							std::map<std::pair<double, double>, double> &hMap,
							std::map<std::pair<double, double>, double> &cMap) {
	ROS_ERROR_STREAM("Run");
    
    Eigen::Vector2d boundaryMin, boundaryMax;

    boundaryMax(0) = maxX + robotStateXYZ(0);
    boundaryMin(0) = minX + robotStateXYZ(0);
    boundaryMax(1) = maxY + robotStateXYZ(1);
    boundaryMin(1) = minY + robotStateXYZ(1);
    double zMax = maxZ + robotStateXYZ(2);
    double zMin = minZ + robotStateXYZ(2);
    double resolution = 0.02;
    for(double x = boundaryMin(0); x<= boundaryMax(0); x += resolution) {
        for(double y = boundaryMin(1); y<= boundaryMax(1); y += resolution) {
            double rotatedX = (x - robotStateXYZ(0)) * cos(robotStateRPY(2)) - (y - robotStateXYZ(1)) * sin(robotStateRPY(2)) + robotStateXYZ(0);
            double rotatedY = (x - robotStateXYZ(0)) * sin(robotStateRPY(2)) - (y - robotStateXYZ(1)) * cos(robotStateRPY(2)) + robotStateXYZ(1);
            //ROS_ERROR_STREAM("rotatedY"<<rotatedY);
            double zMax = maxZ + robotStateXYZ(2);
            

            octomap::OcTreeKey topCellKey;
            // Check if map data present at above X & Y
			if (!octomap->coordToKeyChecked(rotatedX, rotatedY, zMax, depth_, topCellKey)) {       //gives true if point in octree and also 
				printf("No map data at selected location\n");
				return;
			}
            
            int goDownCellBy = 0;
            octomap::OcTreeNode* heightmapNode = octomap->search(topCellKey, depth_);
            // iterate till the surface is not below the zMin limit
            while(zMax >= zMin) {
            	octomap::OcTreeKey currentCellKey;
            	currentCellKey[0] = topCellKey[0];
            	currentCellKey[1] = topCellKey[1];
            	currentCellKey[2] = topCellKey[2] - goDownCellBy;

				heightmapNode = octomap->search(currentCellKey, depth_);
				octomap::point3d currentCellXYZ = octomap->keyToCoord(currentCellKey, depth_);

				zMax = currentCellXYZ(2);

				if(heightmapNode) {
					// If occupied it is the surface
	            	if(octomap->isNodeOccupied(heightmapNode)){
	            		//found surface cell
	            		Eigen::Vector3d surfaceCellXYZ;
	            		surfaceCellXYZ(0) = currentCellXYZ(0);  // x
	            		surfaceCellXYZ(1) = currentCellXYZ(1);  // y
	            		surfaceCellXYZ(2) = currentCellXYZ(2);  // z

	            		// std::cout << surfaceCellXYZ(0)<<" "<<surfaceCellXYZ(1) <<" " <<surfaceCellXYZ(2)<<"***\n";

	            		addToHeightmap(surfaceCellXYZ, hMap);
	            		calculateCost(octomap ,currentCellKey, cMap);
	            		break;
	            	}
            	}
            	goDownCellBy++;
            }
        }
    }
    //hMap = terrainParameters.heightMap;
    //cMap = terrainParameters.costMap;
    ROS_ERROR_STREAM("End");
}

void FootstepAffordance::calculateCost(octomap::OcTree* octomap, const octomap::OcTreeKey& surfaceCellKey, std::map<std::pair<double, double>, double> &cMap) {
    ROS_ERROR_STREAM("calculate Cost");
	firstRun_ = true;
	// put totalCost in costmap
	addToCostmap(surfaceCellXYZ, totalCost, cMap);

}

void FootstepAffordance::addToCostmap(Eigen::Vector3d surfaceCellXYZ, double &totalCost, std::map<std::pair<double, double>, double> &cMap){
	std::map<std::pair<double, double>, double>::iterator it = cMap.find(std::make_pair(surfaceCellXYZ(0),surfaceCellXYZ(1))); 
    if (it != cMap.end()) {
    	if(it->second!=totalCost) {
			it->second = totalCost;
		}
	}
	else
		cMap.insert({std::make_pair(surfaceCellXYZ(0),surfaceCellXYZ(1)), totalCost});	
}

void FootstepAffordance::addToHeightmap(Eigen::Vector3d surfaceCellXYZ, std::map<std::pair<double, double>, double> &hMap){
	std::map<std::pair<double, double>, double>::iterator it = hMap.find(std::make_pair(surfaceCellXYZ(0),surfaceCellXYZ(1))); 
    if (it != hMap.end()) {
    	if(it->second!=surfaceCellXYZ(2)) {
			it->second = surfaceCellXYZ(2);
		}
	}
	else
		hMap.insert({std::make_pair(surfaceCellXYZ(0),surfaceCellXYZ(1)), surfaceCellXYZ(2)});
}
}