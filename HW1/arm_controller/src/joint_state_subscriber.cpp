// // Copyright 2016 Open Source Robotics Foundation, Inc.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include <functional>
// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// using std::placeholders::_1;

// class MinimalSubscriber : public rclcpp::Node
// {
// public:
//   MinimalSubscriber()
//   : Node("minimal_subscriber")
//   {
//     subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
//   }

// private:
//   void topic_callback(const std_msgs::msg::String & msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
//   }
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalSubscriber>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateSubscriber : public rclcpp::Node
{
public:
  JointStateSubscriber() : Node("joint_state_subscriber")
  {
    // Inizialise the subscriber to the topic "joint_states"
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointStateSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    // Print the name of the joint and the current position  of each one 
    for (size_t i = 0; i < msg->name.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", 
                  msg->name[i].c_str(), msg->position[i]);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}


