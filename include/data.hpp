#include <Eigen/Dense>
#include <map>

namespace costmap
{

struct TerrainData
{
    Eigen::Vector3d meanPosition;
    Eigen::Vector3d centroidPosition;
    Eigen::Vector3d normal;
    double curvature;
    Eigen::Matrix3d covarianceMatrix;
    std::map<std::pair<double, double>, double> heightMap;
    std::map<double, std::map<double, double>> costMap;
    double minHeight;
    double resolution;
};
}