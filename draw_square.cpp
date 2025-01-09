// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/msg/pose.hpp>

#include "turtlesim/qos.hpp"

#define PI 3.141592f

class DrawSquare final : public rclcpp::Node
{
public:
  explicit DrawSquare(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("draw_square", options)
  {
    const rclcpp::QoS qos = turtlesim::topic_qos();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

    pose_sub_ =
      this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawSquare::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;

    initializeLetterPoints();
  }

private:
  enum State
  {
    FORWARD,
    STOP_FORWARD
  };

  struct Point {
    float x;
    float y;
    Point(float _x, float _y) : x(_x), y(_y) {}
  };

  void initializeLetterPoints()
  {
    float start_x = 2.0f;
    float start_y = 5.5f;
    float letter_height = 3.0f;
    float letter_width = 1.5f;
    float spacing = 2.0f;

    // Points for letter 'A'
    points_.push_back(Point(start_x, start_y));  // Bottom left
    points_.push_back(Point(start_x + letter_width/2, start_y + letter_height));  // Top middle
    points_.push_back(Point(start_x + letter_width, start_y));  // Bottom right
    points_.push_back(Point(start_x + letter_width/2, start_y + letter_height/2));  // Middle crossbar

    // Points for letter 'S'
    start_x += letter_width + spacing;
    points_.push_back(Point(start_x + letter_width, start_y + letter_height));  // Top right
    points_.push_back(Point(start_x, start_y + letter_height));  // Top left
    points_.push_back(Point(start_x, start_y + letter_height/2));  // Middle left
    points_.push_back(Point(start_x + letter_width, start_y + letter_height/2));  // Middle right
    points_.push_back(Point(start_x + letter_width, start_y));  // Bottom right
    points_.push_back(Point(start_x, start_y));  // Bottom left

    // Points for first 'U'
    start_x += letter_width + spacing;
    points_.push_back(Point(start_x, start_y + letter_height));  // Top left
    points_.push_back(Point(start_x, start_y));  // Bottom left
    points_.push_back(Point(start_x + letter_width, start_y));  // Bottom right
    points_.push_back(Point(start_x + letter_width, start_y + letter_height));  // Top right

    // Points for second 'S'
    start_x += letter_width + spacing;
    points_.push_back(Point(start_x + letter_width, start_y + letter_height));  // Top right
    points_.push_back(Point(start_x, start_y + letter_height));  // Top left
    points_.push_back(Point(start_x, start_y + letter_height/2));  // Middle left
    points_.push_back(Point(start_x + letter_width, start_y + letter_height/2));  // Middle right
    points_.push_back(Point(start_x + letter_width, start_y));  // Bottom right
    points_.push_back(Point(start_x, start_y));  // Bottom left
  }

  void poseCallback(const turtlesim::msg::Pose & pose)
  {
    current_pose_ = pose;
    first_pose_set_ = true;
  }

  bool hasReachedGoal()
  {
    return fabsf(current_pose_.x - goal_pose_.x) < 0.1 &&
           fabsf(current_pose_.y - goal_pose_.y) < 0.1;
  }

  bool hasStopped()
  {
    return current_pose_.angular_velocity < 0.0001 && current_pose_.linear_velocity < 0.0001;
  }

  void printGoal()
  {
    RCLCPP_INFO(
      this->get_logger(), "New goal [%f %f, %f]", goal_pose_.x, goal_pose_.y, goal_pose_.theta);
  }

  void commandTurtle(float linear, float angular)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_pub_->publish(twist);
  }

  void timerCallback()
  {
    if (!reset_result_.valid() || !first_pose_set_) {
      return;
    }

    if (!first_goal_set_) {
      first_goal_set_ = true;
      state_ = FORWARD;
      if (!points_.empty()) {
        goal_pose_.x = points_[0].x;
        goal_pose_.y = points_[0].y;
        printGoal();
      }
    }

    if (current_point_ >= points_.size()) {
      commandTurtle(0, 0);
      return;
    }

    float dx = points_[current_point_].x - current_pose_.x;
    float dy = points_[current_point_].y - current_pose_.y;
    float target_angle = atan2(dy, dx);
    float angle_diff = target_angle - current_pose_.theta;

    // Normalize angle difference to [-pi, pi]
    while (angle_diff > PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;

    if (state_ == FORWARD) {
      if (fabsf(angle_diff) > 0.1) {
        // Turn towards the target point
        commandTurtle(0, angle_diff > 0 ? 0.6f : -0.6f);
      } else if (hasReachedGoal()) {
        state_ = STOP_FORWARD;
        commandTurtle(0, 0);
        current_point_++;
        
        if (current_point_ < points_.size()) {
          goal_pose_.x = points_[current_point_].x;
          goal_pose_.y = points_[current_point_].y;
          printGoal();
        } else {
          RCLCPP_INFO(this->get_logger(), "Drawing completed!");
        }
      } else {
        // Move towards the target point
        commandTurtle(0.8, 0);
      }
    } else if (state_ == STOP_FORWARD) {
      if (hasStopped()) {
        state_ = FORWARD;
      } else {
        commandTurtle(0, 0);
      }
    }
  }

  turtlesim::msg::Pose current_pose_;
  turtlesim::msg::Pose goal_pose_;
  bool first_goal_set_ = false;
  bool first_pose_set_ = false;
  State state_ = FORWARD;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reset_result_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Point> points_;
  size_t current_point_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto nh = std::make_shared<DrawSquare>();

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}
