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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "rgbd_camera_calibration/checkerboard_detector.h"

rgbd_camera_calibration::CheckerboardDetector::CheckerboardDetector(
    ros::NodeHandle& nh)
{
  nh.param("num_rows", num_rows_, 6);
  nh.param("num_cols", num_cols_, 7);
  nh.param("square_size", square_size_, 0.108);
  nh.param("subpixel_window_size", subpixel_window_size_, 11);

  // compute ideal 3D points
  double width = (num_cols_ - 1) * square_size_;
  double height = (num_rows_ - 1) * square_size_;
  for (int r = 0; r < num_rows_; ++r)
  {
    for (int c = 0; c < num_cols_; ++c)
    {
      corners3d_.push_back(
          cv::Point3f(c*square_size_ - width/2,
                      r*square_size_ - height/2, 0.0));
    }
  }
}

bool rgbd_camera_calibration::CheckerboardDetector::detect(
    const sensor_msgs::ImageConstPtr& image_msg,
    std::vector<cv::Point2f>& corners) const
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvShare(image_msg, "mono8")->image;
  }
  catch (cv_bridge::Exception error)
  {
    ROS_ERROR("error: %s", error.what());
    return false;
  }
    
  std::vector<cv::Point2f> corners2d;
  cv::Size board_size(num_cols_, num_rows_);
  bool found = cv::findChessboardCorners(
      image, board_size, corners2d, 
      CV_CALIB_CB_ADAPTIVE_THRESH + 
      CV_CALIB_CB_NORMALIZE_IMAGE + 
      CV_CALIB_CB_FAST_CHECK);

  if (!found)
  {
    ROS_DEBUG("cv::findChessboardCorners() failed.");
    return false;
  }

  // Subpixel refinement
  cv::Size subpixel_window(subpixel_window_size_, subpixel_window_size_);
  cv::Size subpixel_zero_zone(subpixel_zero_zone_, subpixel_zero_zone_);
  cv::cornerSubPix(image, corners2d, subpixel_window, subpixel_zero_zone,
                       cv::TermCriteria(CV_TERMCRIT_ITER,20,1e-2));

  corners = corners2d;
  return true;
}

