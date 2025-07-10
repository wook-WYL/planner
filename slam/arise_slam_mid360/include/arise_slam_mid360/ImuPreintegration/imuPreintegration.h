//
// Created by shibo zhao on 2020-09-27.
//
#pragma once
#ifndef IMUPREINTEGRATION_H
#define IMUPREINTEGRATION_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>


#include "utility.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include "arise_slam_mid360/utils/Twist.h"
#include "arise_slam_mid360/container/MapRingBuffer.h"
#include "arise_slam_mid360/config/parameter.h"
#include "arise_slam_mid360/tic_toc.h"
#include <glog/logging.h>
#include "arise_slam_mid360/sensor_data/imu/imu_data.h"


namespace arise_slam {

    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
    using FrameId = std::uint64_t;

    struct imuPreintegration_config{
        float imuAccNoise;
        float imuAccBiasN;
        float imuGyrNoise;
        float imuGyrBiasN;
        float imuGravity;
        float lidar_correction_noise;
        float smooth_factor;
        bool  use_imu_roll_pitch;
        SensorType sensor;

        double imu_acc_x_limit;
        double imu_acc_y_limit;
        double imu_acc_z_limit;
    };

    class imuPreintegration : public rclcpp::Node {
    public:

        imuPreintegration(const rclcpp::NodeOptions & options);

        static constexpr double delta_t = 0;
        static constexpr double imu_laser_timedelay= 0.8;

    public:
        void initInterface();

        bool
        readParameters();

        void
        laserodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

        void
        visualodometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

        void
        imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw);

        void
        initial_system(double currentCorrectionTime, gtsam::Pose3 lidarPose);

        void
        process_imu_odometry(double currentCorrectionTime, gtsam::Pose3 relativePose);

        template<typename Meas>
        bool
        synchronize_measurements(MapRingBuffer<Meas> &measureBuf,
                                 MapRingBuffer<const nav_msgs::msg::Odometry::SharedPtr> &lidarOdomBuf, bool remove_item);

        bool
        build_graph(gtsam::Pose3 lidarPose, double curLaserodomtimestamp);

        void
        repropagate_imuodometry(double currentCorrectionTime);

        bool
        failureDetection(const gtsam::Vector3 &velCur,
                         const gtsam::imuBias::ConstantBias &biasCur);

        void
        synchronize_measurements(std::deque<const nav_msgs::msg::Odometry::SharedPtr> &laserodomQue,
                                 std::deque<const nav_msgs::msg::Odometry::SharedPtr> &visualodomQue,
                                 std::deque<sensor_msgs::msg::Imu> &imuQue);

        void
        obtainCurrodometry(nav_msgs::msg::Odometry::SharedPtr &odomMsg, double &currentCorrectionTime,
                           gtsam::Pose3 &lidarPose,
                           int &currentResetId);

        void
        integrate_imumeasurement(double currentCorrectionTime);

        void
        reset_graph();

        void
        resetOptimization();

        void
        resetParams();

        void 
        addNoMotionFactor(const FrameId& from_id, const FrameId& to_id);

        void 
        addZeroVelocityPrior(const FrameId& frame_id);

        sensor_msgs::msg::Imu
        imuConverter(const sensor_msgs::msg::Imu &imu_in);

        template<typename T>
        double secs(T msg) {
            return msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9;
        }

        Eigen::Affine3f
        odom2affine(nav_msgs::msg::Odometry odom);


    private:
        // ROS Interface
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVisualOdometry;
        
        // Publishers
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry2;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubHealthStatus;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometrySmooth;

        rclcpp::CallbackGroup::SharedPtr cb_group_;
    public:
        gtsam::noiseModel::Diagonal::shared_ptr noMotionNoise;
        gtsam::noiseModel::Diagonal::shared_ptr noVelocityNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVisualPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::Vector noiseModelBetweenBias;
        // gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
        // gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorOpt_;
        std::shared_ptr<gtsam::PreintegratedImuMeasurements> imuIntegratorImu_;
        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
        gtsam::NavState prevStateOdom;
        gtsam::imuBias::ConstantBias prevBiasOdom;
        gtsam::ISAM2 optimizer;
        gtsam::NonlinearFactorGraph graphFactors;
        gtsam::Values graphValues;
        gtsam::Pose3 lidarodom_w_pre;
        gtsam::Pose3 lidarodom_w_cur;


    public:
        //Modify the extrinsic matrxi between laser and imu, laser and camera
        gtsam::Pose3 imu2cam;
        gtsam::Pose3 cam2Lidar;
        gtsam::Pose3 imu2Lidar;
        gtsam::Pose3 lidar2Imu;

    public:
        tf2::Transform map_to_odom;                     // map -> odom
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfMap2Odom;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfOdom2BaseLink;      // odom -> base_link

    public:
        MapRingBuffer<Imu::Ptr> imuBuf;
        MapRingBuffer<Imu::Ptr> imuBufOpt;
        std::deque<sensor_msgs::msg::Imu> imuQueOpt;
        std::deque<sensor_msgs::msg::Imu> imuQueImu;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> lidarOdomBuf;
        MapRingBuffer<nav_msgs::msg::Odometry::SharedPtr> visualOdomBuf;
        std::mutex mBuf;
        Imu::Ptr imu_Init = std::make_shared<Imu>();

    public:
        bool systemInitialized = false;
        bool doneFirstOpt = false;
        bool use_laserodom = false;
        bool use_visualodom = false;
        bool use_onlyimu = false;
        bool switch_odometry = false;
        bool health_status = true;
        bool imu_init_success = false;

       
        Eigen::Quaterniond firstImu;
        Eigen::Vector3d gyr_pre;

        double first_imu_time_stamp;
        double time_period;
        double last_processed_lidar_time = -1;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;
        int key = 1;
        int imuPreintegrationResetId = 0;
        int frame_count = 0;

        enum IMU_STATE : uint8_t {
        FAIL=0,    //lose imu information 
        SUCCESS=1, //Obtain the good imu data 
        UNKNOW=2
        };  

        IMU_STATE RESULT;
        nav_msgs::msg::Odometry::SharedPtr cur_frame = nullptr;
        nav_msgs::msg::Odometry::SharedPtr last_frame = nullptr;
        imuPreintegration_config config_;
    };

    // class TransformFusion : public imuPreintegration {

    // public:
    //     TransformFusion(); //const rclcpp::NodeOptions & tff_options, const rclcpp::NodeOptions & options);

    //     // ROS Interface
    //     // Subscribers
    //     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    //     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry2;

    //     void imuOdometryCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    //     void lidarOdometryCB(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    //     nav_msgs::msg::Odometry smooth_odometry;
    //     bool initialized_smooth_odometry = false;
        

    // };



}

#endif // IMUPREINTEGRATION_H
