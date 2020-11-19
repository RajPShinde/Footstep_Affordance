#ifndef INCLUDE_FOOTSTEPAFFORDANCE_HPP_
#define INCLUDE_FOOTSTEPAFFORDANCE_HPP_


#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <features.hpp>
#include <map>
#include <ros/ros.h>

namespace costmap
{

class FootstepAffordance 
{
    public:
        
        FootstepAffordance();
        
        ~FootstepAffordance();

        void regionOfInterest();

        void run(octomap::OcTree* octomap, 
                const Eigen::Vector3d& robotStateXYZ, 
                const Eigen::Vector3d& robotStateRPY, 
                std::map<std::pair<double, double>, double> &hMap,
                std::map<std::pair<double, double>, double> &cMap);

        void calculateCost(octomap::OcTree* octomap, const octomap::OcTreeKey& surfaceCellKey, std::map<std::pair<double, double>, double> &cMap);

        void addToHeightmap(Eigen::Vector3d surfaceCellXYZ, std::map<std::pair<double, double>, double> &hMap);

        void addToCostmap(Eigen::Vector3d surfaceCellXYZ, double &totalCost, std::map<std::pair<double, double>, double> &cMap);


    private:    

        double maxX = 2; 
        double minX = 1;
        double maxY = 0.15;
        double minY = -0.15; 
        double maxZ = 0.8;
        double minZ = -0.8;
        TerrainData terrainParameters;
        int depth_ = 16;
        bool firstRun_ = false;
        terrainFeature::Feature features;

};

}

#endif  //  INCLUDE_FOOTSTEPAFFORDANCE_HPP_