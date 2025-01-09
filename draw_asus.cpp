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

class DrawAsus final : public rclcpp::Node
{
public:
  explicit DrawAsus(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("draw_asus", options)
  {
    const rclcpp::QoS qos = turtlesim::topic_qos();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

    pose_sub_ =
      this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawAsus::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    // Initialize waypoints for ASUS
    initializeWaypoints();

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;
  }

private:
  struct Waypoint {
    float x;
    float y;
  };

  void initializeWaypoints()
  {
    // Each letter will be 2x2 units, with 0.5 units spacing between letters
    // Starting at origin (0,0)
    
    // Letter A
    waypoints_.push_back({0.0f, 0.0f});      // Start at bottom left
    waypoints_.push_back({1.0f, 2.0f});      // Up to peak
    waypoints_.push_back({2.0f, 0.0f});      // Down to bottom right
    waypoints_.push_back({0.5f, 1.0f});      // Back to middle left
    waypoints_.push_back({1.5f, 1.0f});      // Across to middle right
    
    // Letter S
    waypoints_.push_back({2.5f, 0.0f});      // Move to start of S
    waypoints_.push_back({4.5f, 0.0f});      // Bottom of S
    waypoints_.push_back({4.5f, 1.0f});      // Up
    waypoints_.push_back({2.5f, 1.0f});      // Across
    waypoints_.push_back({2.5f, 2.0f});      // Up
    waypoints_.push_back({4.5f, 2.0f});      // Top of S

    // Letter U
    waypoints_.push_back({5.0f, 2.0f});      // Move to start of U
    waypoints_.push_back({5.0f, 0.0f});      // Down
    waypoints_.push_back({7.0f, 0.0f});      // Across
    waypoints_.push_back({7.0f, 2.0f});      // Up

    // Letter S
    waypoints_.push_back({7.5f, 2.0f});      // Move to start of final S
    waypoints_.push_back({9.5f, 2.0f});      // Top of S
    waypoints_.push_back({9.5f, 1.0f});      // Down
    waypoints_.push_back({7.5f, 1.0f});      // Across
    waypoints_.push_back({7.5f, 0.0f});      // Down
    waypoints_.push_back({9.5f, 0.0f});      // Bottom of S
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
      this->get_logger(), "New goal [%f %f]", goal_pose_.x, goal_pose_.y);
  }

  void commandTurtle(float linear, float angular)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_pub_->publish(twist);
  }

  void moveToNextWaypoint()
  {
    if (current_waypoint_ >= waypoints_.size()) {
      commandTurtle(0, 0);
      RCLCPP_INFO(this->get_logger(), "Drawing completed!");
      return;
    }

    goal_pose_.x = waypoints_[current_waypoint_].x;
    goal_pose_.y = waypoints_[current_waypoint_].y;
    
    // Calculate angle to target
    float dx = goal_pose_.x - current_pose_.x;
    float dy = goal_pose_.y - current_pose_.y;
    float target_theta = atan2(dy, dx);
    
    float angle_diff = target_theta - current_pose_.theta;
    // Normalize angle
    while (angle_diff > PI) angle_diff -= 2 * PI;
    while (angle_diff < -PI) angle_diff += 2 * PI;

    if (fabs(angle_diff) > 0.1) {
      // Turn towards target
      commandTurtle(0, angle_diff > 0 ? 0.4f : -0.4f);
    } else {
      // Move towards target
      commandTurtle(0.4f, 0);
    }

    if (hasReachedGoal()) {
      current_waypoint_++;
      printGoal();
    }
  }

  void timerCallback()
  {
    if (!reset_result_.valid() || !first_pose_set_) {
      return;
    }

    moveToNextWaypoint();
  }

  turtlesim::msg::Pose current_pose_;
  turtlesim::msg::Pose goal_pose_;
  bool first_pose_set_ = false;
  size_t current_waypoint_ = 0;
  std::vector<Waypoint> waypoints_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reset_result_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<DrawAsus>();
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}