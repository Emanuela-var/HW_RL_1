// Copyright 2016 Open Source Robotics Foundation, Inc.
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

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
// #include <vector> // Aggiungi questa libreria per std::vector

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"

// using namespace std::chrono_literals;

// class Arm_controller_node : public rclcpp::Node
// {
// public:
//   Arm_controller_node()
//   : Node("arm_controller_node")
//   {
//     this->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>{0.0, 0.0, 0.0});

//     publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    
//     timer_ = this->create_wall_timer(
//       500ms, std::bind(&Arm_controller_node::publish_command, this));

//     // Inizialise the subscriber to the topic "joint_states"
//     subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "joint_states", 10,
//       std::bind(&Arm_controller_node::publish_command, this, std::placeholders::_1));
//   }

// private:
//   void publish_command()
//   {
//     auto message = std_msgs::msg::Float64MultiArray();

//     // Ottieni i valori dei parametri
//     this->get_parameter("joint_positions", joint_positions_);

//     // Imposta i valori di comando dal parametro
//     message.data = joint_positions_;

//     RCLCPP_INFO(this->get_logger(), "Publishing command: [%f, %f, %f, %f]",
//                 message.data[0], message.data[1], message.data[2], message.data[3]);

//     publisher_->publish(message);

//     // Print the name of the joint and the current position  of each one 
//     for (size_t i = 0; i < msg->name.size(); ++i) {
//       RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", 
//                   msg->name[i].c_str(), msg->position[i]);
//     }
//   }

//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
//   std::vector<double> joint_positions_;  // Vettore per memorizzare i parametri
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Arm_controller_node>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class Arm_controller_node : public rclcpp::Node
{
public:
  Arm_controller_node()
  : Node("arm_controller_node")
  {
    this->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>{0.0, 0.0, 0.0, 0.0});

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Arm_controller_node::publish_command, this));

    // Inizializza il sottoscrittore per il topic "joint_states"
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&Arm_controller_node::topic_callback, this, std::placeholders::_1));
  }

private:
  // Definisci la funzione di callback per i messaggi in arrivo sul topic "joint_states"
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Joint: %s, Position: %f", 
                  msg->name[i].c_str(), msg->position[i]);
    }
  }

  void publish_command()
  {
    auto message = std_msgs::msg::Float64MultiArray();

    // Ottieni i valori dei parametri
    this->get_parameter("joint_positions", joint_positions_);

    // Imposta i valori di comando dal parametro
    message.data = joint_positions_;

    RCLCPP_INFO(this->get_logger(), "Publishing command: [%f, %f, %f, %f]",
                message.data[0], message.data[1], message.data[2], message.data[3]);

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::vector<double> joint_positions_;  // Vettore per memorizzare i parametri
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Arm_controller_node>());
  rclcpp::shutdown();
  return 0;
}
