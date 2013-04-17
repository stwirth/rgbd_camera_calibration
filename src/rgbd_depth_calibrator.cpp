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

#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/console.h>
#include <ros/ros.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rgbd_camera_calibration/checkerboard_detector.h"

static const std::string WINDOW_NAME = "Depth Calibration";

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> CameraSyncPolicy;

class RgbdDepthCalibrator
{
public:
  RgbdDepthCalibrator(ros::NodeHandle & n) : 
    nh_(n),
    nh_priv_("~"),
    image_sub_ (nh_, "image", 3),
    depth_sub_(nh_, "depth", 3),
    caminfo_sub_(nh_, "camera_info", 3),
    sync_(CameraSyncPolicy(10), image_sub_, depth_sub_, caminfo_sub_),
    finished_(false)
  {
    ROS_INFO_STREAM("Subscribing to:\n"
        "   " << nh_.resolveName("image") << "\n"
        "   " << nh_.resolveName("depth") << "\n"
        "   " << nh_.resolveName("camera_info"));
    rgb_points_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("rgb_checkerboard_points", 1);
    depth_points_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("depth_checkerboard_points", 1);
    sync_.registerCallback(boost::bind(&RgbdDepthCalibrator::cameraCallback, this, _1, _2, _3));

    detector_ = new rgbd_camera_calibration::CheckerboardDetector(nh_priv_);

    std::string logfile_name;
    nh_priv_.param("logfile", logfile_name, std::string(""));
    if (logfile_name != "")
    {
      has_log_ = true;
      log_.open(logfile_name.c_str());
      log_ << "# rgb_z depth_z" << std::endl;
    }

    nh_priv_.param("min_z", min_z_, 0.0);
    nh_priv_.param("max_z", max_z_, 3.5);
    int num_samples;
    nh_priv_.param("num_samples", num_samples, 100);
    sample_bins_.resize(num_samples, 0);
    cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
  }

  ~RgbdDepthCalibrator()
  {
    if (has_log_)
      log_.close();
    cv::destroyWindow(WINDOW_NAME);
    delete detector_;

    std::ofstream out("camera_info/rgbd_depth_params.yaml");
    out << "z_offset_mm: " << offset_ * 1000 << std::endl;
    out << "z_scaling: " << scaling_ << std::endl;
    out.close();
  }

  void cameraCallback ( const sensor_msgs::ImageConstPtr& image_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& caminfo_msg)
  {
    // update camera model
    camera_model_.fromCameraInfo(caminfo_msg);

    // detect checkerboard corners
    std::vector<cv::Point2f> corners;
    bool detected = detector_->detect(image_msg, corners);
    if (!detected)
    {
      ROS_WARN_THROTTLE(2.0, "No calibration pattern detected.");
      return;
    }

    // find 3D checkerboard pose
    const cv::Mat P(3,4, CV_64FC1, const_cast<double*>(caminfo_msg->P.data()));
    // We have to take K' here extracted from P to take the R|t into account
    // that was performed during rectification.
    // This way we obtain the pattern pose with respect to the same frame that
    // is used in stereo depth calculation.
    const cv::Mat K_prime = P.colRange(cv::Range(0,3));
    cv::Mat image_points_mat(corners);
    std::vector<cv::Point3f> object_points(detector_->getIdealWorldPoints());
    cv::Mat object_points_mat(object_points);
    cv::Mat t_vec(3,1,CV_64FC1);
    cv::Mat r_vec(3,1,CV_64FC1);
    cv::solvePnP(object_points_mat, image_points_mat, K_prime, cv::Mat(), r_vec, t_vec);
    // convert to tf
    tf::Vector3 axis(r_vec.at<double>(0, 0), r_vec.at<double>(1, 0), r_vec.at<double>(2, 0));
    double angle = cv::norm(r_vec);
    tf::Quaternion quaternion(axis, angle);
    tf::Vector3 translation(
        t_vec.at<double>(0, 0), t_vec.at<double>(1, 0), t_vec.at<double>(2, 0));
    tf::Transform transform(quaternion, translation);
    // compute checkerboard corners point cloud
    pcl::PointCloud<pcl::PointXYZ> rgb_corner_cloud;
    for (size_t i = 0; i < object_points.size(); ++i)
    {
      pcl::PointXYZ point;
      point.x = object_points[i].x;
      point.y = object_points[i].y;
      point.z = object_points[i].z;
      rgb_corner_cloud.push_back(point);
    }
    pcl_ros::transformPointCloud(rgb_corner_cloud, rgb_corner_cloud, transform);

    // compute checkerboard corners from depth image
    const float* depth_ptr = reinterpret_cast<const float*>(&depth_msg->data[0]);
    std::size_t width = depth_msg->width;
    std::size_t height = depth_msg->height;
    pcl::PointCloud<pcl::PointXYZ> depth_corner_cloud;
    for(size_t i = 0; i < corners.size(); i++)
    {
      const cv::Point2f& pixel = corners.at(i);
      cv::Point3d ray = camera_model_.projectPixelTo3dRay(corners.at(i));
      // todo linear interpolation
      float depth = *(depth_ptr+width*(unsigned int)pixel.y+(unsigned int)pixel.x);
      if ( isnan(depth) )
      {
        ROS_ERROR("nan value in depth, skipping.");
        return;
      }
      double scale = depth / ray.z;
      pcl::PointXYZ depth_point;
      depth_point.x = scale * ray.x;
      depth_point.y = scale * ray.y;
      depth_point.z = scale * ray.z;
      depth_corner_cloud.push_back(depth_point);
    }
    /*
      const pcl::PointXYZ rgb_point = rgb_corner_cloud.points[i];
      cv::Point3d checkerboard_corner(rgb_point.x, rgb_point.y, rgb_point.z);
      cv::Point2d reprojected_point = 
        camera_model_.project3dToPixel(checkerboard_corner);
      double xdiff = reprojected_point.x - pixel.x;
      double ydiff = reprojected_point.y - pixel.y;
      double reprojection_error = sqrt(xdiff*xdiff + ydiff*ydiff);
      */
      /*
      if (has_log_)
      {
        log_ << rgb_point.x << " " << rgb_point.y << " " << rgb_point.z << " "
          << depth_point.x << " " << depth_point.y << " " << depth_point.z << " "
          << pixel.x << " " << pixel.y << " " << reprojection_error << std::endl;
      }
      */
    ROS_ASSERT(rgb_corner_cloud.size() == depth_corner_cloud.size());
    
    rgb_corner_cloud.header = image_msg->header;
    depth_corner_cloud.header = depth_msg->header;
    rgb_points_pub_.publish(rgb_corner_cloud);
    depth_points_pub_.publish(depth_corner_cloud);

    double sum_rgb_z = 0.0;
    double sum_depth_z = 0.0;
    for (size_t i = 0; i < rgb_corner_cloud.points.size(); ++i)
    {
      sum_rgb_z += rgb_corner_cloud.at(i).z;
      sum_depth_z += depth_corner_cloud.at(i).z;
    }
    double rgb_z = sum_rgb_z / rgb_corner_cloud.size();
    double depth_z = sum_depth_z / rgb_corner_cloud.size();
    if (rgb_z <= max_z_ && rgb_z >= min_z_)
    {
      int bin_index = static_cast<int>(
            (sample_bins_.size() - 1) * (rgb_z - min_z_) / (max_z_ - min_z_));
      if (sample_bins_[bin_index] == 0)
      {
        samples_.push_back(std::make_pair(depth_z, rgb_z));
        sample_bins_[bin_index] = 255;
        if (has_log_)
        {
          log_ << rgb_z << " " << depth_z << std::endl;
        }
      }
    }
    if (samples_.size() > 1)
    {
      // line fitting
      double x_sq_sum = 0.0;
      double y_sq_sum = 0.0;
      double xy_sum = 0.0;
      double x_sum = 0.0;
      double y_sum = 0.0;
      for (size_t i = 0; i < samples_.size(); ++i)
      {
        x_sq_sum += samples_[i].first * samples_[i].first;
        y_sq_sum += samples_[i].second * samples_[i].second;
        xy_sum += samples_[i].first * samples_[i].second;
        x_sum += samples_[i].first;
        y_sum += samples_[i].second;
      }
      double d = samples_.size()*x_sq_sum - x_sum*x_sum;
      scaling_ = (samples_.size()*xy_sum - x_sum*y_sum) / d;
      offset_ = (y_sum - scaling_*x_sum) / samples_.size();
      ROS_INFO_STREAM("depth_real = " << scaling_ << " * depth + " << offset_);
    }
    // create the canvas image
    bool copy_data = false;
    cv::Mat_<unsigned char> sample_bin_image(sample_bins_, copy_data);
    sample_bin_image.reshape(0, 1);
    cv::imshow(WINDOW_NAME, sample_bin_image);
    cv::waitKey(5);
  }

  inline bool isFinished() const
  {
    return finished_;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  rgbd_camera_calibration::PatternDetector* detector_;

  message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  message_filters::Synchronizer<CameraSyncPolicy> sync_;

  ros::Publisher rgb_points_pub_;
  ros::Publisher depth_points_pub_;

  bool has_log_;
  std::ofstream log_;
  double min_z_;
  double max_z_;

  image_geometry::PinholeCameraModel camera_model_;

  std::vector<std::pair<double, double> > samples_;
  std::vector<unsigned char> sample_bins_;

  std::vector<cv::Point3f> corners3d_;
  
  bool finished_;

  double scaling_;
  double offset_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_depth_calibrator");
  ros::NodeHandle n;
  RgbdDepthCalibrator calibrator(n);
  while (!calibrator.isFinished() && ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}

