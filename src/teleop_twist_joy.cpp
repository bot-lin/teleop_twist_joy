/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>

// Include additional headers for process management
#include <chrono>
#include <thread>
#include <cstdio>    // For popen, pclose
#include <csignal>   // For kill
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>    // For killpg()
#include <errno.h>     // For errno
#include <unistd.h>

#include "teleop_twist_joy/teleop_twist_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

using namespace std::chrono_literals;

namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  bool require_enable_button;
  int64_t enable_button;
  int64_t enable_turbo_button;
  int64_t linear_increase_button;
  int64_t linear_decrease_button;
  int64_t angular_increase_button;
  int64_t angular_decrease_button;
  double max_linear_speed;
  double min_linear_speed;
  double max_angular_speed;
  double min_angular_speed;
  TeleopTwistJoy* node_;

  std::string cmd_vel_topic;

  std::map<std::string, int64_t> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int64_t> axis_angular_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  bool sent_disable_msg;

  // New member variables for monitoring and restarting joy_node
  rclcpp::TimerBase::SharedPtr monitor_timer;
  rclcpp::Time last_joy_msg_time;
  double joy_timeout;  // In seconds
  std::string joy_node_cmd;
  pid_t joy_node_pid;

  // New methods
  void monitorJoyNode();
  void restartJoyNode();
  void startJoyNode();
  void stopJoyNode();
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions& options) : Node("teleop_twist_joy_node", options)
{
  pimpl_ = new Impl;
  pimpl_->node_ = this;  // Set the node pointer
  pimpl_->cmd_vel_topic = this->declare_parameter("cmd_vel_topic", "cmd_vel_nav");
  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(pimpl_->cmd_vel_topic, 10);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);

  pimpl_->enable_button = this->declare_parameter("enable_button", 5);

  pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);
  pimpl_->linear_increase_button = this->declare_parameter("linear_increase_button", -1);
  pimpl_->linear_decrease_button = this->declare_parameter("linear_decrease_button", -1);
  pimpl_->angular_increase_button = this->declare_parameter("angular_increase_button", -1);
  pimpl_->angular_decrease_button = this->declare_parameter("angular_decrease_button", -1);
  pimpl_->max_linear_speed = this->declare_parameter("max_linear_speed", 0.5);
  pimpl_->min_linear_speed = this->declare_parameter("min_linear_speed", 0.05);
  pimpl_->max_angular_speed = this->declare_parameter("max_angular_speed", 0.5);
  pimpl_->min_angular_speed = this->declare_parameter("min_angular_speed", 0.05);

  // Initialize new member variables for monitoring and restarting joy_node
  pimpl_->joy_timeout = this->declare_parameter("joy_timeout", 5.0);  // Timeout in seconds
  pimpl_->joy_node_cmd = this->declare_parameter("joy_node_cmd", "ros2 run joy joy_node --ros-args -p dev:=/dev/logitech_f710 -p deadzone:=0.3 -p autorepeat_rate:=20.0");
  pimpl_->joy_node_pid = -1;

  // Start the joy_node process
  pimpl_->startJoyNode();

  // Create a timer to monitor the /joy topic
  pimpl_->monitor_timer = this->create_wall_timer(
    1000ms,  // Check every 1 second
    std::bind(&Impl::monitorJoyNode, pimpl_)
  );

  // Initialize last_joy_msg_time to current time
  pimpl_->last_joy_msg_time = this->now();

  std::map<std::string, int64_t> default_linear_map{
    {"x", 5L},
    {"x2", 1L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("axis_linear", default_linear_map);
  this->get_parameters("axis_linear", pimpl_->axis_linear_map);

  std::map<std::string, int64_t> default_angular_map{
    {"yaw", 2L},
    {"yaw2", 4L},
    {"pitch", -1L},
    {"roll", -1L},
  };
  this->declare_parameters("axis_angular", default_angular_map);
  this->get_parameters("axis_angular", pimpl_->axis_angular_map);

  std::map<std::string, double> default_scale_linear_normal_map{
    {"x", 0.5},
    {"x2", 0.5},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  std::map<std::string, double> default_scale_linear_turbo_map{
    {"x", 1.0},
    {"x2", 1.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  std::map<std::string, double> default_scale_angular_normal_map{
    {"yaw", 0.5},
    {"yaw2", 0.5},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular", default_scale_angular_normal_map);
  this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

  std::map<std::string, double> default_scale_angular_turbo_map{
    {"yaw", 1.0},
    {"yaw2", 1.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
  this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "TeleopTwistJoy",
      "Teleop enable button %" PRId64 ".", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
    "Turbo on button %" PRId64 ".", pimpl_->enable_turbo_button);

  ROS_INFO_COND_NAMED(pimpl_->linear_increase_button >= 0, "TeleopTwistJoy",
    "Linear increase on button %" PRId64 ".", pimpl_->linear_increase_button);
  ROS_INFO_COND_NAMED(pimpl_->linear_decrease_button >= 0, "TeleopTwistJoy",
    "Linear decrease on button %" PRId64 ".", pimpl_->linear_decrease_button);
  ROS_INFO_COND_NAMED(pimpl_->angular_increase_button >= 0, "TeleopTwistJoy",
    "Angular increase on button %" PRId64 ".", pimpl_->angular_increase_button);
  ROS_INFO_COND_NAMED(pimpl_->angular_decrease_button >= 0, "TeleopTwistJoy",
    "Angular decrease on button %" PRId64 ".", pimpl_->angular_decrease_button);
  ROS_INFO_NAMED("TeleopTwistJoy", "Max linear: %.2f, Min linear: %.2f, Max angular: %.2f, Min angular: %.2f.",
    pimpl_->max_linear_speed, pimpl_->min_linear_speed, pimpl_->max_angular_speed, pimpl_->min_angular_speed);

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
       it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Linear axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
      "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
       it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(it->second != -1L, "TeleopTwistJoy", "Angular axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopTwistJoy",
      "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;

  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z",
                                              "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                              "enable_button", "enable_turbo_button", "linear_increase_button",
                                              "linear_decrease_button", "angular_increase_button", "angular_decrease_button"};
    static std::set<std::string> doubleparams = {"scale_linear.x", "scale_linear.y", "scale_linear.z",
                                                 "scale_linear_turbo.x", "scale_linear_turbo.y", "scale_linear_turbo.z",
                                                 "scale_angular.yaw", "scale_angular.pitch", "scale_angular.roll",
                                                 "scale_angular_turbo.yaw", "scale_angular_turbo.pitch", "scale_angular_turbo.roll",
                                                 "max_linear_speed", "min_linear_speed", "max_angular_speed", "min_angular_speed"};
    static std::set<std::string> boolparams = {"require_enable_button"};
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    // Loop to check if changed parameters are of expected data type
    for(const auto & parameter : parameters)
    {
      if (intparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (doubleparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (boolparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
        {
          result.reason = "Only boolean values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
    }

    // Loop to assign changed parameters to the member variables
    for (const auto & parameter : parameters)
    {
      if (parameter.get_name() == "require_enable_button")
      {
        this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
      }
      if (parameter.get_name() == "enable_button")
      {
        this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "enable_turbo_button")
      {
        this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "linear_increase_button")
      {
        this->pimpl_->linear_increase_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "linear_decrease_button")
      {
        this->pimpl_->linear_decrease_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "angular_increase_button")
      {
        this->pimpl_->angular_increase_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "angular_decrease_button")
      {
        this->pimpl_->angular_decrease_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.x")
      {
        this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.y")
      {
        this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.z")
      {
        this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.yaw")
      {
        this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.pitch")
      {
        this->pimpl_->axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.roll")
      {
        this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "max_linear_speed")
      {
        this->pimpl_->max_linear_speed = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "min_linear_speed")
      {
        this->pimpl_->min_linear_speed = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "max_angular_speed")
      {
        this->pimpl_->max_angular_speed = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "min_angular_speed")
      {
        this->pimpl_->min_angular_speed = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_turbo.x")
      {
        this->pimpl_->scale_linear_map["turbo"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_turbo.y")
      {
        this->pimpl_->scale_linear_map["turbo"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear_turbo.z")
      {
        this->pimpl_->scale_linear_map["turbo"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear.x")
      {
        this->pimpl_->scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear.y")
      {
        this->pimpl_->scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear.z")
      {
        this->pimpl_->scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_turbo.yaw")
      {
        this->pimpl_->scale_angular_map["turbo"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_turbo.pitch")
      {
        this->pimpl_->scale_angular_map["turbo"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_turbo.roll")
      {
        this->pimpl_->scale_angular_map["turbo"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular.yaw")
      {
        this->pimpl_->scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular.pitch")
      {
        this->pimpl_->scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular.roll")
      {
        this->pimpl_->scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
    }
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

TeleopTwistJoy::~TeleopTwistJoy()
{
  // Stop the joy_node process when this node is destroyed
  pimpl_->stopJoyNode();
  delete pimpl_;
}

double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      axis_map.at(fieldname) == -1L ||
      scale_map.find(fieldname) == scale_map.end() ||
      static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  if (joy_msg->buttons[linear_increase_button])
  {
    scale_linear_map[which_map].at("x") = scale_linear_map[which_map].at("x") + 0.05;
    if (scale_linear_map[which_map].at("x") > max_linear_speed)
    {
      scale_linear_map[which_map].at("x") = max_linear_speed;
    }
    scale_linear_map[which_map].at("x2") = scale_linear_map[which_map].at("x2") + 0.05;
    if (scale_linear_map[which_map].at("x2") > max_linear_speed)
    {
      scale_linear_map[which_map].at("x2") = max_linear_speed;
    }
    scale_linear_map[which_map].at("y") = scale_linear_map[which_map].at("y") + 0.05;
    if (scale_linear_map[which_map].at("y") > max_linear_speed)
    {
      scale_linear_map[which_map].at("y") = max_linear_speed;
    }
    scale_linear_map[which_map].at("z") = scale_linear_map[which_map].at("z") + 0.05;
    if (scale_linear_map[which_map].at("z") > max_linear_speed)
    {
      scale_linear_map[which_map].at("z") = max_linear_speed;
    }
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear increase button pressed");
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear x: %f, y: %f, z: %f", scale_linear_map[which_map].at("x"), scale_linear_map[which_map].at("y"), scale_linear_map[which_map].at("z"));
  }
  if (joy_msg->buttons[linear_decrease_button])
  {
    scale_linear_map[which_map].at("x") = scale_linear_map[which_map].at("x") - 0.05;
    if (scale_linear_map[which_map].at("x") < min_linear_speed)
    {
      scale_linear_map[which_map].at("x") = min_linear_speed;
    }
    scale_linear_map[which_map].at("x2") = scale_linear_map[which_map].at("x2") - 0.05;
    if (scale_linear_map[which_map].at("x2") < min_linear_speed)
    {
      scale_linear_map[which_map].at("x2") = min_linear_speed;
    }
    scale_linear_map[which_map].at("y") = scale_linear_map[which_map].at("y") - 0.05;
    if (scale_linear_map[which_map].at("y") < min_linear_speed)
    {
      scale_linear_map[which_map].at("y") = min_linear_speed;
    }
    scale_linear_map[which_map].at("z") = scale_linear_map[which_map].at("z") - 0.05;
    if (scale_linear_map[which_map].at("z") < min_linear_speed)
    {
      scale_linear_map[which_map].at("z") = min_linear_speed;
    }
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear decrease button pressed");
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear x: %f, y: %f, z: %f", scale_linear_map[which_map].at("x"), scale_linear_map[which_map].at("y"), scale_linear_map[which_map].at("z"));
  }
  if (joy_msg->buttons[angular_increase_button])
  {
    scale_angular_map[which_map].at("yaw") = scale_angular_map[which_map].at("yaw") + 0.05;
    if (scale_angular_map[which_map].at("yaw") > max_angular_speed)
    {
      scale_angular_map[which_map].at("yaw") = max_angular_speed;
    }
    scale_angular_map[which_map].at("yaw2") = scale_angular_map[which_map].at("yaw2") + 0.05;
    if (scale_angular_map[which_map].at("yaw2") > max_angular_speed)
    {
      scale_angular_map[which_map].at("yaw2") = max_angular_speed;
    }
    scale_angular_map[which_map].at("pitch") = scale_angular_map[which_map].at("pitch") + 0.05;
    if (scale_angular_map[which_map].at("pitch") > max_angular_speed)
    {
      scale_angular_map[which_map].at("pitch") = max_angular_speed;
    }
    scale_angular_map[which_map].at("roll") = scale_angular_map[which_map].at("roll") + 0.05;
    if (scale_angular_map[which_map].at("roll") > max_angular_speed)
    {
      scale_angular_map[which_map].at("roll") = max_angular_speed;
    }
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular increase button pressed");
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular yaw: %f, pitch: %f, roll: %f", scale_angular_map[which_map].at("yaw"), scale_angular_map[which_map].at("pitch"), scale_angular_map[which_map].at("roll"));
  }
  if (joy_msg->buttons[angular_decrease_button])
  {
    scale_angular_map[which_map].at("yaw") = scale_angular_map[which_map].at("yaw") - 0.05;
    if (scale_angular_map[which_map].at("yaw") < min_angular_speed)
    {
      scale_angular_map[which_map].at("yaw") = min_angular_speed;
    }
    scale_angular_map[which_map].at("yaw2") = scale_angular_map[which_map].at("yaw2") - 0.05;
    if (scale_angular_map[which_map].at("yaw2") < min_angular_speed)
    {
      scale_angular_map[which_map].at("yaw2") = min_angular_speed;
    }
    scale_angular_map[which_map].at("pitch") = scale_angular_map[which_map].at("pitch") - 0.05;
    if (scale_angular_map[which_map].at("pitch") < min_angular_speed)
    {
      scale_angular_map[which_map].at("pitch") = min_angular_speed;
    }
    scale_angular_map[which_map].at("roll") = scale_angular_map[which_map].at("roll") - 0.05;
    if (scale_angular_map[which_map].at("roll") < min_angular_speed)
    {
      scale_angular_map[which_map].at("roll") = min_angular_speed;
    }
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular decrease button pressed");
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular yaw: %f, pitch: %f, roll: %f", scale_angular_map[which_map].at("yaw"), scale_angular_map[which_map].at("pitch"), scale_angular_map[which_map].at("roll"));
  }

  double v = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  if (v != 0.0) cmd_vel_msg->linear.x = v;
  else cmd_vel_msg->linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x2");
  cmd_vel_msg->linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  cmd_vel_msg->linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  double yaw = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  if (yaw != 0) cmd_vel_msg->angular.z = yaw;
  else cmd_vel_msg->angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw2");
  cmd_vel_msg->angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
  cmd_vel_msg->angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

  cmd_vel_pub->publish(std::move(cmd_vel_msg));
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // Update the time of the last received joy message
  last_joy_msg_time = node_->get_clock()->now();

  if (enable_turbo_button >= 0 &&
      static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (!require_enable_button ||
       (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
           joy_msg->buttons[enable_button]))
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel_pub->publish(std::move(cmd_vel_msg));
      sent_disable_msg = true;
    }
  }
}

// Implement the monitorJoyNode function
void TeleopTwistJoy::Impl::monitorJoyNode()
{
  auto now = node_->get_clock()->now();
  double time_since_last_msg = (now - last_joy_msg_time).seconds();

  if (time_since_last_msg > joy_timeout)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TeleopTwistJoy"),
                 "No /joy messages received in the last %.1f seconds. Restarting joy_node...", time_since_last_msg);

    // Restart the joy_node process
    restartJoyNode();

    // Reset the last_joy_msg_time to now
    last_joy_msg_time = now;
  }
}

void TeleopTwistJoy::Impl::startJoyNode()
{
  if (joy_node_pid != -1)
  {
    RCLCPP_WARN(rclcpp::get_logger("TeleopTwistJoy"), "joy_node is already running with PID %d", joy_node_pid);
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "Starting joy_node with command: %s", joy_node_cmd.c_str());

  // Parse the command into an argument vector
  std::vector<char*> args;
  std::istringstream iss(joy_node_cmd);
  std::string token;
  while (iss >> token)
  {
    char* arg = new char[token.size() + 1];
    std::strcpy(arg, token.c_str());
    args.push_back(arg);
  }
  args.push_back(nullptr);  // execvp requires a nullptr-terminated array

  // Fork a new process
  joy_node_pid = fork();

  if (joy_node_pid == 0)
  {
    // Child process: Set up a new process group
    setpgid(0, 0);  // Set the process group ID to the child's PID

    // Execute the joy_node command
    execvp(args[0], args.data());

    // If execvp returns, there was an error
    perror("execvp");
    exit(1);
  }
  else if (joy_node_pid > 0)
  {
    // Parent process: Set the process group ID for the child process
    setpgid(joy_node_pid, joy_node_pid);

    RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "joy_node started with PID %d", joy_node_pid);
  }
  else
  {
    // Fork failed
    RCLCPP_ERROR(rclcpp::get_logger("TeleopTwistJoy"), "Failed to start joy_node");
    joy_node_pid = -1;
  }

  // Clean up allocated arguments in the parent process
  for (size_t i = 0; i < args.size(); ++i)
  {
    delete[] args[i];
  }
}

void TeleopTwistJoy::Impl::stopJoyNode()
{
  if (joy_node_pid == -1)
  {
    RCLCPP_WARN(rclcpp::get_logger("TeleopTwistJoy"), "joy_node is not running");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "Stopping joy_node with PID %d", joy_node_pid);

  // Send SIGTERM to the process group
  if (killpg(joy_node_pid, SIGTERM) == 0)
  {
    // Wait for all processes in the group to terminate
    int status;
    pid_t pid;
    do
    {
      pid = waitpid(-joy_node_pid, &status, WUNTRACED | WNOHANG);
    } while (pid != -1 || (pid == -1 && errno == EINTR));

    RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "joy_node stopped");
  }
  else
  {
    perror("killpg");
    RCLCPP_ERROR(rclcpp::get_logger("TeleopTwistJoy"), "Failed to stop joy_node");
  }

  joy_node_pid = -1;
}

// Implement the restartJoyNode function
void TeleopTwistJoy::Impl::restartJoyNode()
{
  // Stop the joy_node if it's running
  stopJoyNode();

  // Small delay before restarting
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Start the joy_node again
  startJoyNode();
}

}  // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)