#include <footstepAffordance.hpp>

namespace costmap
{

FootstepAffordance::FootstepAffordance() {

}

FootstepAffordance::~FootstepAffordance() {

}

unsigned int FootstepAffordance::computeMeanAndCovariance(Eigen::Vector3d& meanPosition, Eigen::Matrix3d& covarianceMatrix, const std::vector<Eigen::Vector3d>& cloud) {

	Eigen::VectorXd accu = Eigen::VectorXd::Zero(9, 1);

	for (size_t i = 0; i < cloud.size(); ++i) {
		accu[0] += cloud[i](0) * cloud[i](0);  // summation x.x
		accu[1] += cloud[i](0) * cloud[i](1);  // summation x.y
		accu[2] += cloud[i](0) * cloud[i](2);  // summation x.z
		accu[3] += cloud[i](1) * cloud[i](1);  // summation y.y
		accu[4] += cloud[i](1) * cloud[i](2);  // summation y.z
		accu[5] += cloud[i](2) * cloud[i](2);  // summation z.z
		accu[6] += cloud[i](0);                // summation x
		accu[7] += cloud[i](1);                // summation y
		accu[8] += cloud[i](2);                // summation x
	}

	accu /= (cloud.size());    

	// Meam and Covariance                
	if (cloud.size() != 0) {
		meanPosition[0] = accu[6];
		meanPosition[1] = accu[7];
		meanPosition[2] = accu[8];

		covarianceMatrix.coeffRef(0) = accu[0] - accu[6] * accu[6];
		covarianceMatrix.coeffRef(1) = accu[1] - accu[6] * accu[7];
		covarianceMatrix.coeffRef(2) = accu[2] - accu[6] * accu[8];
		covarianceMatrix.coeffRef(4) = accu[3] - accu[7] * accu[7];
		covarianceMatrix.coeffRef(5) = accu[4] - accu[7] * accu[8];
		covarianceMatrix.coeffRef(8) = accu[5] - accu[8] * accu[8];
		covarianceMatrix.coeffRef(3) = covarianceMatrix.coeff(1);
		covarianceMatrix.coeffRef(6) = covarianceMatrix.coeff(2);
		covarianceMatrix.coeffRef(7) = covarianceMatrix.coeff(5);
	}

	return (static_cast<unsigned int> (cloud.size()));
}

void FootstepAffordance::solvePlaneParameters(Eigen::Vector3d &normal, double &curvature, const Eigen::Matrix3d &covarianceMatrix){

	// Extract the smallest eigenvalue and its eigenvector
	EIGEN_ALIGN16 Eigen::Vector3d::Scalar eigenvalue;
	EIGEN_ALIGN16 Eigen::Vector3d eigenvector;

	//typedef typename Eigen::Matrix3f::Scalar Scalar;
	// Scale the matrix so its entries are in [-1,1]. The scaling is applied
	// only when at least one matrix entry has magnitude larger than 1.
	Eigen::Matrix3d::Scalar scale = covarianceMatrix.cwiseAbs().maxCoeff();
	if (scale <= std::numeric_limits<Eigen::Matrix3d::Scalar>::min())
		scale = Eigen::Matrix3d::Scalar(1.0);

	Eigen::Matrix3d scaledMat = covarianceMatrix / scale;

	Eigen::Vector3d eigenvalues;
	computeRoots(eigenvalues, scaledMat);

	eigenvalue = eigenvalues(0) * scale;

	scaledMat.diagonal().array() -= eigenvalues(0);

	Eigen::Vector3d vec1 = scaledMat.row(0).cross(scaledMat.row(1));
	Eigen::Vector3d vec2 = scaledMat.row(0).cross(scaledMat.row(2));
	Eigen::Vector3d vec3 = scaledMat.row(1).cross(scaledMat.row(2));

	Eigen::Matrix3d::Scalar len1 = vec1.squaredNorm();
	Eigen::Matrix3d::Scalar len2 = vec2.squaredNorm();
	Eigen::Matrix3d::Scalar len3 = vec3.squaredNorm();

	if (len1 >= len2 && len1 >= len3)
		eigenvector = vec1 / std::sqrt(len1);
	else if (len2 >= len1 && len2 >= len3)
		eigenvector = vec2 / std::sqrt(len2);
	else
		eigenvector = vec3 / std::sqrt(len3);

	// Check K_hat. The normal vectors should point to the +ve Z direction
	if (eigenvector(2) < 0.)
		eigenvector *= -1;

	normal= eigenvector;
}

void FootstepAffordance::computeRoots(Eigen::Vector3d& roots, const Eigen::Matrix3d& m)
{
	// The characteristic equation is x^3 - c2*x^2 + c1*x - c0 = 0. The
	// eigenvalues are the roots to this equation, all guaranteed to be
	// real-valued, because the matrix is symmetric.
	Eigen::Matrix3d::Scalar c0 = m(0,0) * m(1,1) * m(2,2) +
			Eigen::Matrix3d::Scalar(2) * m(0,1) * m(0,2) * m(1,2)
	- m(0,0) * m(1,2) * m(1,2) - m(1,1) * m(0,2) * m(0,2)
	- m(2,2) * m(0,1) * m(0,1);

	Eigen::Matrix3d::Scalar c1 = m(0,0) * m(1,1) - m(0,1) * m(0,1) + m(0,0) * m(2,2) -
			m(0,2) * m(0,2) + m(1,1) * m(2,2) - m(1,2) * m(1,2);

	Eigen::Matrix3d::Scalar c2 = m(0,0) + m(1,1) + m(2,2);


	if (fabs (c0) < Eigen::NumTraits<Eigen::Matrix3d::Scalar>::epsilon ())// one root is 0 -> quadratic equation
		computeRoots2(roots, c2, c1);
	else
	{
		const Eigen::Matrix3d::Scalar s_inv3 = Eigen::Matrix3d::Scalar(1.0 / 3.0);
		const Eigen::Matrix3d::Scalar s_sqrt3 = std::sqrt(Eigen::Matrix3d::Scalar (3.0));
		// Construct the parameters used in classifying the roots of the equation
		// and in solving the equation for the roots in closed form.
		Eigen::Matrix3d::Scalar c2_over_3 = c2*s_inv3;
		Eigen::Matrix3d::Scalar a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
		if (a_over_3 > Eigen::Matrix3d::Scalar(0))
			a_over_3 = Eigen::Matrix3d::Scalar (0);

		Eigen::Matrix3d::Scalar half_b = Eigen::Matrix3d::Scalar(0.5) *
				(c0 + c2_over_3 * (Eigen::Matrix3d::Scalar(2) * c2_over_3 * c2_over_3 - c1));

		Eigen::Matrix3d::Scalar q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
		if (q > Eigen::Matrix3d::Scalar(0))
			q = Eigen::Matrix3d::Scalar(0);

		// Compute the eigenvalues by solving for the roots of the polynomial.
		Eigen::Matrix3d::Scalar rho = sqrt(-a_over_3);
		Eigen::Matrix3d::Scalar theta = atan2(std::sqrt(-q), half_b) * s_inv3;
		Eigen::Matrix3d::Scalar cos_theta = cos(theta);
		Eigen::Matrix3d::Scalar sin_theta = sin(theta);
		roots(0) = c2_over_3 + Eigen::Matrix3d::Scalar(2) * rho * cos_theta;
		roots(1) = c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta);
		roots(2) = c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta);

		// Sort in increasing order.
		double roots0 = roots(0);
		double roots1 = roots(1);
		double roots2 = roots(2);
		if (roots(0) >= roots(1))
			std::swap(roots0, roots1);
		if (roots(1) >= roots(2)) {
			std::swap(roots1, roots2);
			if (roots(0) >= roots(1))
				std::swap(roots0, roots1);
		}
		roots(0) = roots0;
		roots(1) = roots1;
		roots(2) = roots2;

		if (roots(0) <= 0) // eigenvalue for symmetric positive semi-definite matrix can not be negative! Set it to 0
			computeRoots2(roots, c2, c1);
	}
}

void FootstepAffordance::computeRoots2(Eigen::Vector3d& roots,
				   const Eigen::Matrix3d::Scalar& b,
				   const Eigen::Matrix3d::Scalar& c)
{
	roots(0) = Eigen::Matrix3d::Scalar(0);
	Eigen::Matrix3d::Scalar d = Eigen::Matrix3d::Scalar(b * b - 4.0 * c);
	if (d < 0.0) // no real roots!!!! THIS SHOULD NOT HAPPEN!
		d = 0.0;

	Eigen::Matrix3d::Scalar sd = sqrt(d);

	roots(2) = 0.5f * (b + sd);
	roots(1) = 0.5f * (b - sd);
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
	bool atLeastOneNeighbour = false;

	std::vector<Eigen::Vector3d> allNeighboursPosition;
	octomap::OcTreeNode* surfaceCellNode = octomap->search(surfaceCellKey, depth_);

	Eigen::Vector3d surfaceCellXYZ;
	octomap::point3d surfaceCellPoint = octomap->keyToCoord(surfaceCellKey, depth_);
	surfaceCellXYZ(0) = surfaceCellPoint(0);  // x
	surfaceCellXYZ(1) = surfaceCellPoint(1);  // y
	surfaceCellXYZ(2) = surfaceCellPoint(2);  // z
	allNeighboursPosition.push_back(surfaceCellXYZ);

	octomap::OcTreeKey neighbourKey;
	octomap::OcTreeNode* neighbourNode = surfaceCellNode;
	bool yq = false;
	ROS_ERROR_STREAM("Neighbours");
	for(int z = -1; z<=1; z++) {
		for(int y = -1; y<=1; y++) {
			for(int x = -1; x<=1; x++) {
				neighbourKey[0] = surfaceCellKey[0] + x;
				neighbourKey[1] = surfaceCellKey[1] + y;
				neighbourKey[2] = surfaceCellKey[2] + z;
				neighbourNode = octomap->search(neighbourKey, depth_);
				yq =true;
				//ROS_ERROR_STREAM(z);
				// Check if this neighbour exists
				if(neighbourNode) {
					// ROS_ERROR_STREAM("Found1");
					if(octomap->isNodeOccupied(neighbourNode)) {
						// ROS_ERROR_STREAM("Found2");
						Eigen::Vector3d neighbourCellXYZ;
						octomap::point3d neighbourCellPoint;
						neighbourCellPoint = octomap->keyToCoord(neighbourKey, depth_);
						neighbourCellXYZ(0) = neighbourCellPoint(0); 
						neighbourCellXYZ(1) = neighbourCellPoint(1);
						neighbourCellXYZ(2) = neighbourCellPoint(2);
						allNeighboursPosition.push_back(neighbourCellXYZ);
						atLeastOneNeighbour = true;
					}
				}
			}
		}
	}
	if(yq)
		ROS_WARN_STREAM("HAA andar gaya tha");
	if(atLeastOneNeighbour) {
		//compute cost
		ROS_ERROR_STREAM("At Least One Neighbour");
		EIGEN_ALIGN16 Eigen::Matrix3d covarianceMatrix;
		if(allNeighboursPosition.size()<3 || computeMeanAndCovariance(terrainParameters.meanPosition, covarianceMatrix, allNeighboursPosition)==0){
			ROS_ERROR_STREAM("<3");
			return;
		}
		ROS_WARN_STREAM(allNeighboursPosition.size());
		terrainParameters.centroidPosition(0) = allNeighboursPosition[0](0);
		terrainParameters.centroidPosition(1) = allNeighboursPosition[0](1);
		terrainParameters.centroidPosition(2) = allNeighboursPosition[0](2);
		
		solvePlaneParameters(terrainParameters.normal, terrainParameters.curvature, covarianceMatrix);
	}

	ROS_ERROR_STREAM("Cost");
	ROS_ERROR_STREAM(terrainParameters.normal);
	double cost, weight, totalCost = 0;
	// for(auto i: features) {
		features.computeSlopeCost(cost, terrainParameters);
		totalCost = 1*cost;
	// }

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