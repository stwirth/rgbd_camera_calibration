/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

//! \author Stephan Wirth

#ifndef RGBD_CAMERA_CALIBRATION_RGBD_PROCESSOR_H_
#define RGBD_CAMERA_CALIBRATION_RGBD_PROCESSOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.h>

namespace rgbd_camera_calibration
{


class RgbdProcessor
{

public:

  /**
   * Constructor, subscribes to input topics using image transport and registers
   * the callback.
   */
  RgbdProcessor(ros::NodeHandle& nh)
  {
    // Resolve topic names
    std::string image_topic = nh.resolveName("image");
    std::string depth_topic = nh.resolveName("depth_image");

    std::string image_info_topic = ros::names::append(
      ros::names::parentNamespace(image_topic), "camera_info");
    std::string depth_info_topic = ros::names::append(
      ros::names::parentNamespace(depth_topic), "camera_info");

    // Subscribe to four input topics.
    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s", 
        image_topic.c_str(), depth_topic.c_str(),
        image_info_topic.c_str(), depth_info_topic.c_str());

    image_transport::ImageTransport it(nh);
    image_transport::TransportHints rgb_transport_hints(
        "raw", ros::TransportHints(), ros::NodeHandle("~"), "rgb_image_transport");
    image_transport::TransportHints depth_transport_hints(
        "raw", ros::TransportHints(), ros::NodeHandle("~"), "depth_image_transport");
    image_sub_.subscribe(it, image_topic, 1, rgb_transport_hints);
    depth_sub_.subscribe(it, depth_topic, 1, depth_transport_hints);
    image_info_sub_.subscribe(nh, image_info_topic, 1);
    depth_info_sub_.subscribe(nh, depth_info_topic, 1);

    // Read local parameters
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size_, 5);
    bool approx;
    local_nh.param("approximate_sync", approx, true);
    if (approx)
    {
      approximate_sync_.reset(
          new ApproximateSync(ApproximatePolicy(queue_size_),
              image_sub_, depth_sub_, image_info_sub_, depth_info_sub_) );
      approximate_sync_->registerCallback(
          boost::bind(&RgbdProcessor::imageCallback, this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset(
          new ExactSync(ExactPolicy(queue_size_),
              image_sub_, depth_sub_, image_info_sub_, depth_info_sub_) );
      exact_sync_->registerCallback(
          boost::bind(&RgbdProcessor::imageCallback, this, _1, _2, _3, _4));
    }
  }

protected:

  /**
   * Implement this method in sub-classes 
   */
  virtual void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                             const sensor_msgs::ImageConstPtr& depth_msg,
                             const sensor_msgs::CameraInfoConstPtr& image_info_msg,
                             const sensor_msgs::CameraInfoConstPtr& depth_info_msg) = 0;

private:

  // subscriber
  image_transport::SubscriberFilter image_sub_, depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> image_info_sub_, depth_info_sub_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  int queue_size_;

};

} // end of namespace

#endif

