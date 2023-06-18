/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef GAZEBO_ROS_OUSTER_LASER_H_
#define GAZEBO_ROS_OUSTER_LASER_H_

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ray_plugin/gazebo_ray_plugin.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <gazebo/plugins/RayPlugin.hh>


namespace gazebo {

class GazeboRosOusterLaser : public RayPlugin {
public:
  GazeboRosOusterLaser();

  ~GazeboRosOusterLaser();

  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  void ConnectCb();

  sensors::RaySensorPtr parent_ray_sensor_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

  std::string topic_name_;
  std::string frame_name_;

  double min_range_;
  double max_range_;

  double gaussian_noise_;

  static double gaussianKernel(double mu, double sigma) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(mu, sigma);
    return d(gen);
  }

  std::mutex lock_;

  std::string robot_namespace_;

  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;
  rclcpp::CallbackGroup::SharedPtr gazebo_callback_group_;

  rclcpp::CallbackGroup::SharedPtr laser_queue_thread_callback_group_;

  void laserQueueThread();

  rclcpp::TimerBase::SharedPtr laser_queue_thread_timer_;

  transport::NodePtr gazebo_node_;

  transport::SubscriberPtr sub_;

  void OnScan(ConstLaserScanStampedPtr& _msg);
};

}  // namespace gazebo

#endif /* GAZEBO_ROS_OUSTER_LASER_H_ */
