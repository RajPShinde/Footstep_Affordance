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

        unsigned int computeMeanAndCovariance(Eigen::Vector3d& meanPosition, Eigen::Matrix3d& covarianceMatrix, const std::vector<Eigen::Vector3d>& cloud);

        void solvePlaneParameters(Eigen::Vector3d &normal, double &curvature, const Eigen::Matrix3d &covarianceMatrix);

        void computeRoots(Eigen::Vector3d& roots, const Eigen::Matrix3d& m);

        void computeRoots2(Eigen::Vector3d& roots, const Eigen::Matrix3d::Scalar& b, const Eigen::Matrix3d::Scalar& c);

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

        const double distanceFromRobot = 0.3;
        double maxX = 1.0; 
        double minX = 0.24;
        double maxY = 0.18;
        double minY = -0.18; 
        double maxZ = 0.4;
        double minZ = -0.4;
        TerrainData terrainParameters;
        int depth_ = 16;
        bool firstRun_ = false;
        int neighbourMaxX, neighbourMaxY, neighbourMaxZ = 1;
        int neighbourMinX, neighbourMinY, neighbourMinZ = -1;
        terrainFeature::Feature features;
};

}

#endif  //  INCLUDE_FOOTSTEPAFFORDANCE_HPP_