//
// Created by shibo zhao on 2020-09-27.
//
#include "rclcpp/rclcpp.hpp"
#include "arise_slam_mid360/FeatureExtraction/featureExtraction.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    options.arguments({"feature_extraction_node"});
    
    std::shared_ptr<arise_slam::featureExtraction> featureExtraction =
        std::make_shared<arise_slam::featureExtraction>(options);

    featureExtraction->imuBuf.allocate(2000);
    featureExtraction->visualOdomBuf.allocate(2000);
    featureExtraction->lidarBuf.allocate(50);
    featureExtraction->initInterface();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(featureExtraction);
    executor.spin();
    // rclcpp::spin(featureExtraction->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
