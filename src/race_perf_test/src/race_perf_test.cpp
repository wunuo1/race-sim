#include <cstdio>
#include "race_perf_test/race_perf_test.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

Race_Performace_Test::Race_Performace_Test(std::string node_name, 
    const rclcpp::NodeOptions & node_options)
    : Node(node_name, node_options) {
    this->declare_parameter("image_path", "/home/carry/code/race-sim/image.jpeg");
    rclcpp::QoS custom_qos(rclcpp::KeepLast(10));
    custom_qos.reliable().transient_local();
    image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/zed_camera_left_sensor/image_raw/compressed", custom_qos);
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    custom_qos.durability_volatile();
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      custom_qos,
      std::bind(&Race_Performace_Test::twist_subscribe_callback,
      this,
      std::placeholders::_1),
      sub1_opt
    );
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;
    string_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/hobot_dnn_detection",
      custom_qos,
      std::bind(&Race_Performace_Test::string_subscribe_callback,
      this,
      std::placeholders::_1),
      sub2_opt
    );
    timer_ = this->create_wall_timer(100ms, std::bind(&Race_Performace_Test::timer_callback, this));
    auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(this);
    image_path_ = this->get_parameter("image_path").as_string();
    image_ = cv::imread(image_path_);
    start_time_points_.resize(2000);
    resnet_end_time_points_.resize(2000);
    yolo_end_time_points_.resize(2000);
    publish_count_ = 0;
    resnet_subscribe_count_ = 0;
    yolo_subscribe_count_ = 0;

}

Race_Performace_Test::~Race_Performace_Test()
{

}

void Race_Performace_Test::timer_callback()
{
  if (publish_count_ < 1000)
  {
      RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"), "pub count: %d \n", publish_count_);
      std_msgs::msg::Header header;
      header.frame_id = std::to_string(publish_count_);
      const std::string format = "bgr8";
      sensor_msgs::msg::CompressedImage::SharedPtr msg =
          cv_bridge::CvImage(header, format, image_).toCompressedImageMsg();
      std::chrono::time_point<std::chrono::steady_clock> publish_time = std::chrono::steady_clock::now();
      start_time_points_[publish_count_] = publish_time;
      publish_count_++;
      image_publisher_->publish(*msg);
  } 
}

void Race_Performace_Test::twist_subscribe_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  
  RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"), "sub count: %d \n", resnet_subscribe_count_);
  if (resnet_subscribe_count_ <= 3000)
  {
      std::chrono::time_point<std::chrono::steady_clock> subscribe_time = std::chrono::steady_clock::now();
      resnet_end_time_points_[resnet_subscribe_count_] = subscribe_time;
  }
  resnet_subscribe_count_++;

  if (resnet_subscribe_count_ > 995)
  {
      double duration_sum = 0.0;
      for (int i = 0; i < resnet_subscribe_count_; i++)
      {
          std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(resnet_end_time_points_[i] - start_time_points_[i]);
          duration_sum += duration.count();
          // RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"), "duration_sum: %f \n", duration_sum);
      }
      double duration_avg = duration_sum /resnet_subscribe_count_;

      double duration_var = 0;
      double duration_diff = 0;
      for (int i = 0; i < resnet_subscribe_count_; i++)
      {
          std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(resnet_end_time_points_[i] - start_time_points_[i]);
          duration_diff = duration.count() - duration_avg;
          duration_var = duration_diff * duration_diff + duration_var;
      }
      duration_var = duration_var / resnet_subscribe_count_;

      RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"),
               "resnet average time cost: %f  variance: %f \n", duration_avg, duration_var);
  }
}

void Race_Performace_Test::string_subscribe_callback(std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"), " string sub count: %d \n", yolo_subscribe_count_);
  if (yolo_subscribe_count_ <= 3000)
  {
      std::chrono::time_point<std::chrono::steady_clock> subscribe_time = std::chrono::steady_clock::now();
      yolo_end_time_points_[yolo_subscribe_count_] = subscribe_time;
  }
  yolo_subscribe_count_++;

  if (yolo_subscribe_count_ > 990)
  {
      double duration_sum = 0.0;
      for (int i = 0; i < yolo_subscribe_count_; i++)
      {
          std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(yolo_end_time_points_[i] - start_time_points_[i]);
          duration_sum += duration.count();
          // RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"), "duration_sum: %f \n", duration_sum);
      }
      double duration_avg = duration_sum /yolo_subscribe_count_;

      double duration_var = 0;
      double duration_diff = 0;
      for (int i = 0; i < yolo_subscribe_count_; i++)
      {
          std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(yolo_end_time_points_[i] - start_time_points_[i]);
          duration_diff = duration.count() - duration_avg;
          duration_var = duration_diff * duration_diff + duration_var;
      }
      duration_var = duration_var / yolo_subscribe_count_;

      RCLCPP_INFO(rclcpp::get_logger("Race_Performace_Test"),
               "yolo average time cost: %f  variance: %f \n", duration_avg, duration_var);
  }
}


int main(int argc, char ** argv)
{
  printf("hello world race_perf_test package\n");
  rclcpp::init(argc, argv);


  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<Race_Performace_Test>();
  executor.add_node(node);
  executor.spin();
  // rclcpp::spin(std::make_shared<Race_Performace_Test>());
  rclcpp::shutdown();
  
  return 0;
}
