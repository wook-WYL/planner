// 包含标准数学库头文件，提供数学函数支持
#include <math.h>
// 包含时间处理相关的头文件，用于处理时间相关操作
#include <time.h>
// 包含标准输入输出库头文件，提供基本的输入输出功能
#include <stdio.h>
// 包含标准库头文件，提供通用工具函数，如内存分配、随机数生成等
#include <stdlib.h>
// 包含 C++ 标准库中的时间处理头文件，提供更高级的时间处理功能
#include <chrono>
// 包含标准输入输出流头文件，用于 C++ 风格的输入输出操作
#include <iostream>
// 包含 ROS 2 的核心库头文件，用于创建 ROS 2 节点和相关功能
#include "rclcpp/rclcpp.hpp"
// 包含 ROS 2 时间相关的头文件，用于处理时间相关操作
#include "rclcpp/time.hpp"
// 包含 ROS 2 时钟相关的头文件，用于获取当前时间
#include "rclcpp/clock.hpp"
// 包含内置接口消息类型的时间消息头文件，用于表示时间戳
#include "builtin_interfaces/msg/time.hpp"

// 包含传感器消息类型的 IMU 消息头文件，用于处理 IMU 数据
#include <sensor_msgs/msg/imu.hpp>
// 包含传感器消息类型的图像消息头文件，用于处理图像数据
#include <sensor_msgs/msg/image.hpp>
// 包含传感器消息类型的压缩图像消息头文件，用于处理压缩后的图像数据
#include <sensor_msgs/msg/compressed_image.hpp>

// 包含 OpenCV 库的综合头文件，包含了 OpenCV 大部分功能
#include <opencv2/opencv.hpp>
// 包含 OpenCV 核心功能的头文件，提供基本的数据结构和算法
#include <opencv2/core/core.hpp>
// 包含 OpenCV 高级图形用户界面功能的头文件，用于显示图像等操作
#include <opencv2/highgui/highgui.hpp>
// 包含 OpenCV 图像处理功能的头文件，提供各种图像处理算法
#include <opencv2/imgproc/imgproc.hpp>
// 包含 OpenCV 特征检测和描述功能的头文件，用于特征提取等操作
#include <opencv2/features2d/features2d.hpp>
// 包含 OpenCV 视频跟踪功能的头文件，用于视频跟踪相关操作
#include <opencv2/video/tracking.hpp>
// 包含 OpenCV 相机标定和三维重建功能的头文件，用于相机标定等操作
#include <opencv2/calib3d/calib3d.hpp>

// 包含 OpenCV 与 ROS 2 图像转换的桥接头文件，用于在两者之间转换图像数据
#include <cv_bridge/cv_bridge.h>

// 使用标准命名空间，避免每次使用标准库中的类和函数时都要写 std::
using namespace std;
// 使用 OpenCV 命名空间，避免每次使用 OpenCV 中的类和函数时都要写 cv::
using namespace cv;

// 定义全局变量，用于存储原始图像和缩小后的图像
Mat image, imageSmall;

// 定义 IMU 数据订阅的话题名称
string imuTopicName = "/imu/data";
// 定义图像上下边缘裁剪的像素值
int topBottonMargin = 160;
// 定义图像延迟时间
double imageLatency = 0;
// 定义压缩图像的质量参数
int compQuality = 50;
// 定义是否始终发布压缩图像的标志
bool alwaysPubCompImage = false;
// 定义是否显示图像的标志
bool showImage = false;

// 定义系统时间与 IMU 时间的差值
double systemToImuTime = 0;

// 定义 OpenCV 与 ROS 2 图像转换的桥接对象
cv_bridge::CvImage bridge;

// 定义 ROS 2 节点的共享指针
rclcpp::Node::SharedPtr nh;

/**
 * @brief IMU 数据处理回调函数
 *
 * 该函数在接收到 IMU 数据时被调用，计算系统时间与 IMU 时间的差值
 *
 * @param imuIn 接收到的 IMU 数据的常量共享指针
 */
void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr imuIn)
{
  // 计算系统时间与 IMU 时间的差值
  systemToImuTime = rclcpp::Time(imuIn->header.stamp).seconds() - nh->now().seconds();
}

/**
 * @brief 主函数，程序入口
 *
 * 初始化 ROS 2 节点，订阅 IMU 数据，发布图像和压缩图像，处理视频流
 *
 * @param argc 命令行参数的数量
 * @param argv 命令行参数的数组
 * @return int 程序退出状态码
 */
int main(int argc, char **argv)
{
  // 初始化 ROS 2 节点
  rclcpp::init(argc, argv);
  // 创建名为 "receiveTheta" 的 ROS 2 节点
  nh = rclcpp::Node::make_shared("receiveTheta");

  // 声明 ROS 2 节点的参数，设置默认值
  nh->declare_parameter<string>("imuTopicName", imuTopicName);
  nh->declare_parameter<int>("topBottonMargin", topBottonMargin);
  nh->declare_parameter<double>("imageLatency", imageLatency);
  nh->declare_parameter<int>("compQuality", compQuality);
  nh->declare_parameter<bool>("alwaysPubCompImage", alwaysPubCompImage);
  nh->declare_parameter<bool>("showImage", showImage);

  // 获取 ROS 2 节点的参数值
  nh->get_parameter("imuTopicName", imuTopicName);
  nh->get_parameter("topBottonMargin", topBottonMargin);
  nh->get_parameter("imageLatency", imageLatency);
  nh->get_parameter("compQuality", compQuality);
  nh->get_parameter("alwaysPubCompImage", alwaysPubCompImage);
  nh->get_parameter("showImage", showImage);

  // 创建 IMU 数据的订阅者，指定话题名称、队列长度和回调函数
  auto imuSub = nh->create_subscription<sensor_msgs::msg::Imu>(imuTopicName, 50, imuHandler);

  // 创建图像数据的发布者，指定话题名称和队列长度
  auto imagePub = nh->create_publisher<sensor_msgs::msg::Image>("/camera/image", 2);

  // 创建压缩图像数据的发布者，指定话题名称和队列长度
  auto compressedPub = nh->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image/compressed", 2);

  // 打开视频流，使用 GStreamer 管道
  VideoCapture video("thetauvcsrc ! decodebin ! videoconvert ! appsink");
  // 检查视频流是否成功打开
  if (!video.isOpened())
  {
    // 若未成功打开，输出错误信息并退出程序
    printf("\nCannot open device, exit.\n\n");
    return 0;
  }

  // 定义压缩参数向量，用于设置 JPEG 压缩质量
  std::vector<int> compression_params;
  // 添加 JPEG 压缩质量参数标识
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  // 添加 JPEG 压缩质量参数值
  compression_params.push_back(compQuality);

  // 当 ROS 2 节点正常运行时，循环处理视频流
  while (rclcpp::ok())
  {
    // 从视频流中读取一帧图像
    video >> image;

    // 计算图像的时间戳
    double imageTime = nh->now().seconds() + systemToImuTime - imageLatency;

    // 确保上下边缘裁剪的像素值在有效范围内
    if (topBottonMargin < 0)
      topBottonMargin = 0;
    else if (topBottonMargin > image.rows / 2 - 1)
      topBottonMargin = image.rows / 2 - 1;

    // 若需要裁剪图像上下边缘
    if (topBottonMargin > 0)
    {
      // 定义感兴趣区域（ROI）
      Rect roi = Rect(0, topBottonMargin, image.cols, image.rows - 2 * topBottonMargin);
      // 裁剪图像
      image = image(roi);
    }

    // 创建消息头，设置坐标系和时间戳
    std_msgs::msg::Header header;
    header.frame_id = "camera";
    header.stamp = rclcpp::Time(static_cast<uint64_t>(imageTime * 1e9));
    // 将 OpenCV 图像转换为 ROS 2 图像消息
    sensor_msgs::msg::Image::SharedPtr imagePtr = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    // 发布图像消息
    imagePub->publish(*imagePtr);

    // 若有订阅者或始终发布压缩图像
    if (compressedPub->get_subscription_count() > 0 || alwaysPubCompImage)
    {
      // 定义存储压缩后图像数据的向量
      std::vector<uint8_t> jpeg_image;
      // 对图像进行 JPEG 压缩
      cv::imencode(".jpg", image, jpeg_image, compression_params);
      // 创建压缩图像消息
      sensor_msgs::msg::CompressedImage compressed_msg;
      compressed_msg.header = header;
      compressed_msg.format = "jpeg";
      compressed_msg.data = jpeg_image;
      // 发布压缩图像消息
      compressedPub->publish(compressed_msg);
    }

    // 若需要显示图像
    if (showImage)
    {
      // 将图像缩小为原来的一半
      resize(image, imageSmall, Size(image.cols / 2, image.rows / 2));
      // 显示缩小后的图像
      imshow("Image (50% res)", imageSmall);
    }

    // 处理 ROS 2 节点的回调函数
    rclcpp::spin_some(nh);
    // 等待 10 毫秒，处理窗口事件
    waitKey(10);
  }

  // 程序正常退出，返回 0
  return 0;
}
