//
// Created by qyd on 25-2-13.
//
#include "ros/ros.h"
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>  // 确保包含了该头文件
#include <random>
#include <cmath>

namespace my_controller_ns
{
class MyPositionController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& n)
  {
    std::string my_joint;
    if (!n.getParam("joint", my_joint))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!n.getParam("use_feedforward", use_feedforward))
    {
      ROS_ERROR("Could not find  use_feedforward");
      return false;
    }
    if (!n.getParam("feedforward_gain_", feedforward_gain_))
    {
      ROS_ERROR("Could not find  feedforward_gain_");
      return false;
    }
    joint_ = hw->getHandle(my_joint);  // throws on failure

    kp_ = 1.0;
    ki_ = 0.1;
    kd_ = 0.05;
    if (!n.getParam("pid/p", kp_))
    {
      ROS_ERROR("Could not find  p");
      return false;
    }
    if (!n.getParam("pid/i", ki_))
    {
      ROS_ERROR("Could not find  i");
      return false;
    }
    if (!n.getParam("pid/d", kd_))
    {
      ROS_ERROR("Could not find  d");
      return false;
    }
    start_time_ = ros::Time::now();

    // Start command subscriber
    sub_command_ = n.subscribe<std_msgs::Int64>("command", 1, &MyPositionController::setCommandCB, this);
    generator_ = std::default_random_engine(std::random_device{}());
    dist_a_ = std::uniform_real_distribution<double>(0.780, 1.045);  // a 范围
    dist_b_ = std::uniform_real_distribution<double>(1.884, 2.000);  // b 范围
    a_ = dist_a_(generator_);
    b_ = dist_b_(generator_);
    c_ = 2.090 - a_;

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // double elapsed_time = (time - start_time_).toSec();
    // ros::Duration duration_time(0.5);
    // time_zero_ = time_zero_ + duration_time;

    // 获取目标速度（通过前馈控制）
    double target_speed;
    double pre_target_speed;
    time_zero_ = ros::Time::now();
    switch (speed_mode)
    {
      case 0:

        target_speed = computeTargetSpeed(time_zero_.toSec());
        pre_target_speed = computeTargetSpeed(prev_time.toSec());

        prev_time = time_zero_;
        break;
      case 1:
        target_speed = M_PI / 3;
        break;
    }

    // 计算PID控制器误差
    double error = target_speed - joint_.getVelocity();
    integral_ += error * period.toSec();
    double derivative = (error - prev_error_) / period.toSec();
    prev_error_ = error;

    // 计算PID控制输出
    double pid_output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    double feedforward_output;
    switch (use_feedforward)
    {
      case 0:
        command = pid_output;
        break;
      case 1:
        // 计算前馈控制量
        feedforward_output = computeFeedforward(target_speed, pre_target_speed);

        // 结合前馈控制和PID输出，得到最终的命令
        command = pid_output + feedforward_output;
        break;
    }
    // 设定电机控制量
    joint_.setCommand(command);
  }
  double computeFeedforward(double target_speed, double pre_target_speed)
  {
    return (target_speed - pre_target_speed) * feedforward_gain_;
  }
  double computeTargetSpeed(double time)
  {
    // 通过正弦波轨迹控制速度
    return a_ * sin(b_ * time) + c_;
  }

  void setCommandCB(const std_msgs::Int64::ConstPtr& msg)
  {
    speed_mode = msg->data;
  }

  void starting(const ros::Time& time)
  {
    time_zero_ = ros::Time::now();
    prev_time = ros::Time::now();
  }
  void stopping(const ros::Time& time)
  {
  }

private:
  hardware_interface::JointHandle joint_;
  double gain_;
  double command;
  double kp_, ki_, kd_;
  int speed_mode = 0;

  double integral_;
  double prev_error_ = 0.0;
  double feedforward_gain_ = 0.1;

  std::default_random_engine generator_;
  std::uniform_real_distribution<double> dist_a_, dist_b_, dist_c_;
  double a_, b_, c_;

  ros::Time start_time_;
  int use_feedforward = 0;

  ros::Subscriber sub_command_;
  ros::Time time_zero_;
  double pre_target_speed;
  double target_speed;
  ros::Time prev_time;
};

PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyPositionController, controller_interface::ControllerBase);
}  // namespace my_controller_ns
