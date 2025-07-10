#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#include "motion_msgs/msg/motion_ctrl.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "ception_msgs/msg/imu_euler.hpp"

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool manualMode = false;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
float joyManualFwd = 0;
float joyManualYaw = 0;
int safetyStop = 0;
int slowDown = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
int vehicleReady = -1;
float bodyHeight = 1.0;
float bodyPitch = 0;

float bodyLow = 0.28;
float bodyHigh = 0.52;
float bodySpeedGain = 0.5;
float bodyMaxSpeed = 0.2;

float pitchRateGain = 2.0;
float pitchMaxRate = 0.5;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds(); 
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  joyManualFwd = joy->axes[4];
  joyManualYaw = joy->axes[3];

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

void slowDownHandler(const std_msgs::msg::Int8::ConstSharedPtr slow)
{
  slowDown = slow->data;
}

void robotStatusHandler(const motion_msgs::msg::RobotStatus::ConstSharedPtr status)
{
  if (status->robot_mode_msg == 3) vehicleReady = 1;
  else vehicleReady = 0;

  //printf ("Ctrl mode %d, Robot mode %d\n", status->ctrl_mode_msg, status->robot_mode_msg);
}

void motorStatusHandler(const motion_msgs::msg::LegMotors::ConstSharedPtr status)
{
  bodyHeight = (status->left_leg_length + status->right_leg_length - bodyLow) / (bodyHigh - bodyLow);
}

void imuHandler(const ception_msgs::msg::IMUEuler::ConstSharedPtr status)
{
  bodyPitch = status->pitch;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto subSlowDown = nh->create_subscription<std_msgs::msg::Int8>("/slow_down", 5, slowDownHandler);

  auto subRobotStatus = nh->create_subscription<motion_msgs::msg::RobotStatus>("diablo/sensor/Body_state", 5, robotStatusHandler);

  auto subMotorStatus = nh->create_subscription<motion_msgs::msg::LegMotors>("diablo/sensor/Motors", 5, motorStatusHandler);

  auto subIMU = nh->create_subscription<ception_msgs::msg::IMUEuler>("/diablo/sensor/ImuEuler", 5, imuHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  auto pubMotionCtrl = nh->create_publisher<motion_msgs::msg::MotionCtrl>("diablo/MotionCmd", 5);
  motion_msgs::msg::MotionCtrl ctrl_msg;

  auto pubClearing = nh->create_publisher<std_msgs::msg::Float32>("/map_clearing", 5);
  std_msgs::msg::Float32 clear_msg;
  clear_msg.data = 8.0;

  auto pubClearingExt = nh->create_publisher<std_msgs::msg::Float32>("/cloud_clearing", 5);
  std_msgs::msg::Float32 clear_msg_ext;
  clear_msg_ext.data = 30.0;

  ctrl_msg.value.forward = 0;
  ctrl_msg.value.left = 0;
  ctrl_msg.value.up = 0;
  ctrl_msg.value.roll = 0;
  ctrl_msg.value.pitch = 0;
  ctrl_msg.value.leg_split = 0;
  ctrl_msg.mode.pitch_ctrl_mode = true;
  ctrl_msg.mode.roll_ctrl_mode = false;
  ctrl_msg.mode.height_ctrl_mode = true;
  ctrl_msg.mode.stand_mode = true;
  ctrl_msg.mode.jump_mode = false;
  ctrl_msg.mode.split_mode = false;

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  int ctrlInitFrameCount = 150;
  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      if (twoWayDrive) {
        double time = nh->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0 || slowDown == 1) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0 || slowDown == 2) joySpeed3 *= slowRate2;

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        
        if (manualMode) {
          cmd_vel.twist.linear.x = maxSpeed * joyManualFwd;
          cmd_vel.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;
        }
        
        pubSpeed->publish(cmd_vel);

        if (vehicleReady != 0) {
          ctrl_msg.value.forward = cmd_vel.twist.linear.x;
          ctrl_msg.value.left = cmd_vel.twist.angular.z;

          float desiredHeight = 1.0;
          if (slowDown == 1) desiredHeight = 0;
          else if (slowDown == 2) desiredHeight = 0.5;

          ctrl_msg.value.up = bodySpeedGain * (desiredHeight - bodyHeight);
          if (ctrl_msg.value.up < -bodyMaxSpeed) ctrl_msg.value.up = -bodyMaxSpeed;
          else if (ctrl_msg.value.up > bodyMaxSpeed) ctrl_msg.value.up = bodyMaxSpeed;

          ctrl_msg.value.pitch = -pitchRateGain * bodyPitch;
          if (ctrl_msg.value.pitch < -pitchMaxRate) ctrl_msg.value.pitch = -pitchMaxRate;
          else if (ctrl_msg.value.pitch > pitchMaxRate) ctrl_msg.value.pitch = pitchMaxRate;
        } else {
          ctrl_msg.value.forward = 0;
          ctrl_msg.value.left = 0;
          ctrl_msg.value.up = 0;
          ctrl_msg.value.pitch = 0;

          pubClearing->publish(clear_msg);
          pubClearingExt->publish(clear_msg_ext);
        }

        ctrl_msg.mode_mark = false;
        if (ctrlInitFrameCount > 0) {
          ctrl_msg.mode_mark = true;
          ctrlInitFrameCount--;
        }

        pubMotionCtrl->publish(ctrl_msg);

        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
