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

#include <cassert>
#include <memory>
#include <chrono>
#include <algorithm>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#if GAZEBO_GPU_RAY
  #include <gazebo/sensors/GpuRaySensor.hh>
#else
  #include <gazebo/sensors/RaySensor.hh>
#endif

void SensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Modify the PointCloud2 data as needed

  // Access PointCloud2 fields
  for (auto& field : msg->fields)
  {
    // Access field values
    std::cout << "Field name: " << field.name << ", offset: " << field.offset
              << ", datatype: " << field.datatype
              << ", count: " << field.count << std::endl;
  }

  // Access PointCloud2 data
  const size_t pointCount = msg->width * msg->height;
  const size_t pointStep = msg->point_step;
  const size_t dataSize = pointCount * pointStep;
  const uint8_t* dataPtr = msg->data.data();

  for (size_t i = 0; i < dataSize; i += pointStep)
  {
    // Access individual points
    const float x = *reinterpret_cast<const float*>(dataPtr + i);
    const float y = *reinterpret_cast<const float*>(dataPtr + i + sizeof(float));
    const float z = *reinterpret_cast<const float*>(dataPtr + i + 2 * sizeof(float));

    std::cout << "Point: (" << x << ", " << y << ", " << z << ")" << std::endl;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gazebo_plugin_node");

  // Subscribe to the sensor topic
  auto subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points", 10, SensorCallback);

  // Start the ROS event loop
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}