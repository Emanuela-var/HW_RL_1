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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector> // Aggiungi questa libreria per std::vector

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class PositionCommandPublisher : public rclcpp::Node
{
public:
  PositionCommandPublisher()
  : Node("position_command_publisher")
  {
    this->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>{0.0, 0.0, 0.0});

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PositionCommandPublisher::publish_command, this));
  }

private:
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
  std::vector<double> joint_positions_;  // Vettore per memorizzare i parametri
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
