#include <interface.hpp>

namespace footstepAffordance
{

Interface::Interface(ros::NodeHandle& nh) : interface_(nh), robotFrame_("base_link"), worldFrame_("odom"), octomapSub_(NULL), tfOctomapSub_(NULL) {
    // Getting the base and world frame
    interface_.param("robotFrame_", robotFrame_, robotFrame_);
    interface_.param("worldFrame_", worldFrame_, worldFrame_);

    octomapSub_ = new message_filters::Subscriber<octomap_msgs::Octomap>(node_, "octomap_binary", 5);
    tfOctomapSub_ = new tf::MessageFilter<octomap_msgs::Octomap>(*octomapSub_, tf_listener_, worldFrame_, 5);
    tfOctomapSub_->registerCallback(boost::bind(&Interface::octomapCallback, this, _1));

    visualizeHeightMapPub = vis_.advertise<visualization_msgs::MarkerArray>( "/heightMap", 10 );
    visualizeCostMapPub = vis_.advertise<visualization_msgs::MarkerArray>( "/costMap", 10 );

}

Interface::~Interface(){

}

void Interface::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg){
    ROS_ERROR_STREAM("Octomap Callback");
    // Creating an octree
    octomap::OcTree* octomap = NULL;

    // convert ros msg to octomaptree for use with octomap library
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

    if (tree) {
        octomap = dynamic_cast<octomap::OcTree*>(tree);
    }

    if (!octomap) {
        ROS_WARN("Failed to create octree structure");
        return;
    }

    Eigen::Vector3d robotStateXYZ = Eigen::Vector3d::Zero();
    Eigen::Vector3d robotStateRPY = Eigen::Vector3d::Zero();
    getRobotState(robotStateXYZ, robotStateRPY, msg);

    cost.run(octomap ,robotStateXYZ, robotStateRPY, heightMap, costMap);
    // displayHeightmap(heightMap);
    visualizeHeightMap(heightMap);
    visualizeCostMap(heightMap, costMap);
}

void Interface::getRobotState(Eigen::Vector3d &robotStateXYZ, Eigen::Vector3d &robotStateRPY, const octomap_msgs::Octomap::ConstPtr& msg) 
{
    tf::StampedTransform tf_transform;
    try {
        tf_listener_.lookupTransform(worldFrame_, robotFrame_, msg->header.stamp, tf_transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what());
        return;
    }

    robotStateXYZ(0) = tf_transform.getOrigin()[0];   // x
    robotStateXYZ(1) = tf_transform.getOrigin()[1];   // y
    robotStateXYZ(2) = tf_transform.getOrigin()[2];   // z

    tf::Quaternion q = tf_transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    robotStateRPY(0) =  roll;                  // roll
    robotStateRPY(1) =  pitch;                 // pitch
    robotStateRPY(2) =  yaw;                   // yaw
        
    ROS_WARN_STREAM("XYZ: "<<robotStateXYZ(0)<<" | "<<robotStateXYZ(1)<<" | "<<robotStateXYZ(2));
    ROS_WARN_STREAM("YPR:"<<yaw<<" | "<<pitch<<" | "<<roll);
}

void Interface::displayHeightmap(std::map<std::pair<double, double>, double> &hMap) {
    ROS_ERROR_STREAM("Printing Heightmap");
    for (const auto& x : hMap) {
        std::cout << x.first.first<<" "<<x.first.second <<" " <<x.second<<"***\n";
    }
}

void Interface::visualizeHeightMap(std::map<std::pair<double, double>, double> &hMap){
    ROS_ERROR_STREAM("Visualize");
    visualization_msgs::MarkerArray marker;
    
    int i=0;
    marker.markers.resize(hMap.size());

    for (const auto& x : hMap) {
        marker.markers[i].header.frame_id = worldFrame_;
        marker.markers[i].header.stamp = ros::Time();
        marker.markers[i].ns = "heightMap";
        marker.markers[i].id = i;
        marker.markers[i].type = visualization_msgs::Marker::CUBE;
        marker.markers[i].action = visualization_msgs::Marker::ADD;
        marker.markers[i].pose.position.x = x.first.first;
        marker.markers[i].pose.position.y = x.first.second;
        marker.markers[i].pose.position.z = x.second;
        marker.markers[i].pose.orientation.x = 0;
        marker.markers[i].pose.orientation.y = 0;
        marker.markers[i].pose.orientation.z = 0;
        marker.markers[i].pose.orientation.w = 1.0;
        marker.markers[i].scale.x = 0.02;
        marker.markers[i].scale.y = 0.02;
        marker.markers[i].scale.z = 0.02;
        marker.markers[i].color.a = 1.0; 
        marker.markers[i].color.r = 1.0;
        marker.markers[i].color.g = 0.0;
        marker.markers[i].color.b = 0.0;
        i++;
    }
    visualizeHeightMapPub.publish(marker);
}

void Interface::visualizeCostMap(std::map<std::pair<double, double>, double> &hMap, std::map<std::pair<double, double>, double> &cMap){
    ROS_ERROR_STREAM("Visualize");
    visualization_msgs::MarkerArray marker;
    
    int i=0;
    marker.markers.resize(hMap.size());

    for (const auto& x : hMap) {
        auto it = cMap.find(std::make_pair(x.first.first, x.first.second)); 
        marker.markers[i].header.frame_id = worldFrame_;
        marker.markers[i].header.stamp = ros::Time();
        marker.markers[i].ns = "costMap";
        marker.markers[i].id = i;
        marker.markers[i].type = visualization_msgs::Marker::CUBE;
        marker.markers[i].action = visualization_msgs::Marker::ADD;
        marker.markers[i].pose.position.x = x.first.first;
        marker.markers[i].pose.position.y = x.first.second;
        marker.markers[i].pose.position.z = x.second;
        marker.markers[i].pose.orientation.x = 0;
        marker.markers[i].pose.orientation.y = 0;
        marker.markers[i].pose.orientation.z = 0;
        marker.markers[i].pose.orientation.w = 1.0;
        marker.markers[i].scale.x = 0.02;
        marker.markers[i].scale.y = 0.02;
        marker.markers[i].scale.z = 0.02;
        marker.markers[i].color.a = 1.0; 
        marker.markers[i].color.r = 0.0;
        marker.markers[i].color.g = 0.0;
        marker.markers[i].color.b = 1.0/it->second;
        i++;
    }
    visualizeCostMapPub.publish(marker);
}
}

int main(int argc, char **argv){
    ros::init(argc, argv, "footstepAffordance");
    ros::NodeHandle nh;
    footstepAffordance::Interface start(nh);
    ros::spin();
    return 0;
}
