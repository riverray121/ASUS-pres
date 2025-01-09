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

class DrawASUS final : public rclcpp::Node
{
public:
  explicit DrawASUS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("draw_asus", options)
  {
    const rclcpp::QoS qos = turtlesim::topic_qos();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

    pose_sub_ =
      this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawASUS::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;

    // Initialize the points for drawing ASUS
    initializeLetterPoints();
  }

private:
  struct Point {
    float x;
    float y;
    float theta;
    bool pen_down;  // Whether to draw while moving to this point
    
    Point(float _x, float _y, float _theta, bool _pen_down = true)
    : x(_x), y(_y), theta(_theta), pen_down(_pen_down) {}
  };

  void initializeLetterPoints()
  {
    float start_x = 2.0f;
    float start_y = 5.0f;
    float letter_height = 2.0f;
    float letter_width = 1.0f;
    float spacing = 1.5f;

    // Points for letter 'A'
    letter_points_.push_back(Point(start_x, start_y, PI/2));  // Bottom left
    letter_points_.push_back(Point(start_x + letter_width/2, start_y + letter_height, PI/2));  // Top middle
    letter_points_.push_back(Point(start_x + letter_width, start_y, PI/2));  // Bottom right
    letter_points_.push_back(Point(start_x + letter_width, start_y + letter_height/2, 0, false));  // Move to middle right
    letter_points_.push_back(Point(start_x, start_y + letter_height/2, 0));  // Middle crossbar

    // Points for letter 'S'
    start_x += letter_width + spacing;
    letter_points_.push_back(Point(start_x + letter_width, start_y + letter_height, 0, false));  // Move to top right
    letter_points_.push_back(Point(start_x, start_y + letter_height, 0));  // Top horizontal
    letter_points_.push_back(Point(start_x, start_y + letter_height/2, 0));  // Middle left
    letter_points_.push_back(Point(start_x + letter_width, start_y + letter_height/2, 0));  // Middle right
    letter_points_.push_back(Point(start_x + letter_width, start_y, 0));  // Bottom right
    letter_points_.push_back(Point(start_x, start_y, 0));  // Bottom horizontal

    // Points for first 'U'
    start_x += letter_width + spacing;
    letter_points_.push_back(Point(start_x, start_y + letter_height, 0, false));  // Move to top left
    letter_points_.push_back(Point(start_x, start_y, 0));  // Down left
    letter_points_.push_back(Point(start_x + letter_width, start_y, 0));  // Bottom
    letter_points_.push_back(Point(start_x + letter_width, start_y + letter_height, 0));  // Up right

    // Points for second 'S'
    start_x += letter_width + spacing;
    letter_points_.push_back(Point(start_x + letter_width, start_y + letter_height, 0, false));  // Move to top right
    letter_points_.push_back(Point(start_x, start_y + letter_height, 0));  // Top horizontal
    letter_points_.push_back(Point(start_x, start_y + letter_height/2, 0));  // Middle left
    letter_points_.push_back(Point(start_x + letter_width, start_y + letter_height/2, 0));  // Middle right
    letter_points_.push_back(Point(start_x + letter_width, start_y, 0));  // Bottom right
    letter_points_.push_back(Point(start_x, start_y, 0));  // Bottom horizontal
  }

  enum State
  {
    MOVING,
    STOPPING,
    FINISHED
  };

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
    return current_pose_.angular_velocity < 0.01 && current_pose_.linear_velocity < 0.01;
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

  void moveToNextPoint()
  {
    if (current_point_ >= letter_points_.size()) {
      state_ = FINISHED;
      commandTurtle(0, 0);
      return;
    }

    const Point& target = letter_points_[current_point_];
    float dx = target.x - current_pose_.x;
    float dy = target.y - current_pose_.y;
    float target_angle = atan2(dy, dx);
    float angle_diff = target_angle - current_pose_.theta;

    // Normalize angle difference to [-pi, pi]
    while (angle_diff > PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;

    if (fabsf(angle_diff) > 0.05) {
      // Turn towards the target - increased rotation speed from 1.2 to 2.0
      commandTurtle(0, angle_diff > 0 ? 2.0f : -2.0f);
    } else {
      // Move towards the target - increased speeds
      float distance = sqrt(dx*dx + dy*dy);
      if (target.pen_down) {
        commandTurtle(std::min(2.0f, distance), 0);
      } else {
        commandTurtle(std::min(4.0f, distance), 0);
      }
    }

    if (hasReachedGoal()) {
      state_ = STOPPING;
      commandTurtle(0, 0);
    }
  }

  void timerCallback()
  {
    if (!reset_result_.valid() || !first_pose_set_) {
      return;
    }

    if (!first_goal_set_) {
      first_goal_set_ = true;
      state_ = MOVING;
      if (!letter_points_.empty()) {
        goal_pose_.x = letter_points_[0].x;
        goal_pose_.y = letter_points_[0].y;
        printGoal();
      }
    }

    if (state_ == MOVING) {
      moveToNextPoint();
    } else if (state_ == STOPPING && hasStopped()) {
      current_point_++;
      if (current_point_ < letter_points_.size()) {
        state_ = MOVING;
        goal_pose_.x = letter_points_[current_point_].x;
        goal_pose_.y = letter_points_[current_point_].y;
        printGoal();
      } else {
        state_ = FINISHED;
      }
    }
  }

  std::vector<Point> letter_points_;
  size_t current_point_ = 0;
  turtlesim::msg::Pose current_pose_;
  turtlesim::msg::Pose goal_pose_;
  bool first_goal_set_ = false;
  bool first_pose_set_ = false;
  State state_ = MOVING;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reset_result_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<DrawASUS>();
  rclcpp::spin(nh);
  rclcpp::shutdown();
  return 0;
}