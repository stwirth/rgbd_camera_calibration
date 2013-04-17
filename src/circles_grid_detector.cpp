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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "rgbd_camera_calibration/circles_grid_detector.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");


  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}

rgbd_camera_calibration::CirclesGridDetector::CirclesGridDetector(
    ros::NodeHandle& nh) : flags_(0), reconfigure_server_(nh)
{
  nh.param("num_rows", num_rows_, 5);
  nh.param("num_cols", num_cols_, 8);
  nh.param("square_size", square_size_, 0.0915);

  /* not supported yet
  bool use_asymmetric_grid;
  nh.param("use_asymmetric_grid", use_asymmetric_grid_, false);
  if (use_asymmetric_grid)
  {
    flags_ += cv::CALIB_CB_ASYMMETRIC_GRID;
  }
  else
  */
  {
    flags_ += cv::CALIB_CB_SYMMETRIC_GRID;
  }

  bool use_clustering;
  nh.param("use_clustering", use_clustering, false);
  if (use_clustering)
  {
    flags_ += cv::CALIB_CB_CLUSTERING;
  }

  reconfigure_server_.setCallback(boost::bind(&CirclesGridDetector::configureBlobDetector, this, _1, _2));

  // compute ideal 3D points
  // TODO asymmetric grid
  double width = (num_cols_ - 1) * square_size_;
  double height = (num_rows_ - 1) * square_size_;
  for (int r = 0; r < num_rows_; ++r)
  {
    for (int c = 0; c < num_cols_; ++c)
    {
      centers3d_.push_back(
          cv::Point3f(c*square_size_ - width/2,
                      r*square_size_ - height/2, 0.0));
    }
  }
}

void rgbd_camera_calibration::CirclesGridDetector::configureBlobDetector(
    rgbd_camera_calibration::BlobDetectorConfig& config, uint32_t /*level*/) 
{

  cv::SimpleBlobDetector::Params params;
  params.thresholdStep = config.threshold_step;
  if (config.max_threshold < config.min_threshold) config.max_threshold = config.min_threshold;
  params.minThreshold = config.min_threshold;
  params.maxThreshold = config.max_threshold;

  params.minRepeatability = config.min_repeatability;
  params.minDistBetweenBlobs = config.min_dist_between_blobs;

  params.filterByColor = config.filter_by_color;
  if (config.blob_color > 127) config.blob_color = 255; else config.blob_color = 0;
  params.blobColor = config.blob_color;

  params.filterByArea = config.filter_by_area;
  if (config.max_area < config.min_area) config.max_area = config.min_area;
  params.minArea = config.min_area;
  params.maxArea = config.max_area;

  params.filterByCircularity = config.filter_by_circularity;
  if (config.max_circularity < config.min_circularity) config.max_circularity = config.min_circularity;
  params.minCircularity = config.min_circularity;
  params.maxCircularity = config.max_circularity;

  params.filterByInertia = config.filter_by_inertia;
  if (config.max_inertia_ratio < config.min_inertia_ratio) config.max_inertia_ratio = config.min_inertia_ratio;
  params.minInertiaRatio = config.min_inertia_ratio;
  params.maxInertiaRatio = config.max_inertia_ratio;

  params.filterByConvexity = config.filter_by_convexity;
  if (config.max_convexity < config.min_convexity) config.max_convexity = config.min_convexity;
  params.minConvexity = config.min_convexity;
  params.maxConvexity = config.max_convexity;

  blob_detector_ = cv::Ptr<cv::FeatureDetector>(new cv::SimpleBlobDetector(params));
}

bool rgbd_camera_calibration::CirclesGridDetector::detect(
    const sensor_msgs::ImageConstPtr& image_msg,
    std::vector<cv::Point2f>& centers) const
{
  cv::Mat image;
  // We want to scale floating point images so that they display nicely
  if(image_msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image_bridge = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
    cv::Mat_<float> float_image = float_image_bridge;
    float max_val = 0;
    for(int i = 0; i < float_image.rows; ++i)
    {
      for(int j = 0; j < float_image.cols; ++j)
      {
        max_val = std::max(max_val, float_image(i, j));
      }
    }

    if(max_val > 0)
    {
      float_image /= max_val;
    }
    float_image.convertTo(image, CV_8UC1, 255.0);
  }
  else
  {
    try {
      image = cv_bridge::toCvShare(image_msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& error) {
      ROS_ERROR("error: %s", error.what());
      return false;
    }
  }

  std::vector<cv::Point2f> centers2d;
  cv::Size board_size(num_cols_, num_rows_);
  bool found = cv::findCirclesGrid(
      image, board_size, centers2d, flags_, blob_detector_);

  if (!found)
  {
    ROS_DEBUG("cv::findCirclesGrid() failed.");
    return false;
  }

  centers = centers2d;
  return true;
}

