// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _LINE_FOLLOWER_PERCEPTION_H_
#define _LINE_FOLLOWER_PERCEPTION_H_

#include <vector>
#include <chrono>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"



class Race_Performace_Test : public rclcpp::Node
{
    public:
        explicit Race_Performace_Test(std::string node_name = "Race_Performace_Test", const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
        ~Race_Performace_Test();
    private:
        void twist_subscribe_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void string_subscribe_callback(std_msgs::msg::String::SharedPtr msg);
        void timer_callback();
        std::vector<std::chrono::time_point<std::chrono::steady_clock>> start_time_points_;
        std::vector<std::chrono::time_point<std::chrono::steady_clock>> resnet_end_time_points_;
        std::vector<std::chrono::time_point<std::chrono::steady_clock>> yolo_end_time_points_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscriber_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
        std::string image_path_;
        cv::Mat image_;
        int publish_count_;
        int resnet_subscribe_count_;
        int yolo_subscribe_count_;
};

#endif  // _LINE_FOLLOWER_PERCEPTION_H_