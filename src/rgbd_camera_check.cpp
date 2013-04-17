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

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>

#include "rgbd_camera_calibration/rgbd_processor.h"
#include "rgbd_camera_calibration/circles_grid_detector.h"

static const std::string WINDOW_NAME = "Depth Calibration";

namespace rgbd_camera_calibration
{

class RgbdCameraCheck : public RgbdProcessor
{
public:
  RgbdCameraCheck(ros::NodeHandle & n) : RgbdProcessor(n),
    nh_(n),
    nh_priv_("~")
  {
    ros::NodeHandle rgb_detector_nh(nh_priv_, "rgb_detector");
    rgb_detector_ = 
      new rgbd_camera_calibration::CirclesGridDetector(rgb_detector_nh);
    ros::NodeHandle depth_detector_nh(nh_priv_, "depth_detector");
    depth_detector_ = 
      new rgbd_camera_calibration::CirclesGridDetector(depth_detector_nh);
    cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
  }

  ~RgbdCameraCheck()
  {
    cv::destroyWindow(WINDOW_NAME);
    delete rgb_detector_;
    delete depth_detector_;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::ImageConstPtr& depth_msg,
                     const sensor_msgs::CameraInfoConstPtr& rgb_info_msg,
                     const sensor_msgs::CameraInfoConstPtr& depth_info_msg)
  {
    // update camera model
    rgb_camera_model_.fromCameraInfo(rgb_info_msg);
    depth_camera_model_.fromCameraInfo(depth_info_msg);

    // detect circles
    std::vector<cv::Point2f> centers;
    bool detected = depth_detector_->detect(depth_msg, centers);
    if (!detected)
    {
      ROS_WARN_THROTTLE(2.0, "No pattern detected.");
    }

    cv::Mat depth_image = cv_bridge::toCvShare(depth_msg)->image;
    cv::drawChessboardCorners(
        depth_image, 
        cv::Size(depth_detector_->getNumCols(), depth_detector_->getNumRows()), 
        centers, detected);
    cv::imshow(WINDOW_NAME, depth_image);
    cv::waitKey(5);

    /*

    // find 3D pattern pose
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
    // compute pattern point cloud
    pcl::PointCloud<pcl::PointXYZ> rgb_point_cloud;
    for (size_t i = 0; i < object_points.size(); ++i)
    {
      pcl::PointXYZ point;
      point.x = object_points[i].x;
      point.y = object_points[i].y;
      point.z = object_points[i].z;
      rgb_point_cloud.push_back(point);
    }
    pcl_ros::transformPointCloud(rgb_point_cloud, rgb_point_cloud, transform);

    */

    /*
    // detect corners from depth image
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
    */
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
    /*
    ROS_ASSERT(rgb_corner_cloud.size() == depth_corner_cloud.size());
    
    rgb_corner_cloud.header = image_msg->header;
    depth_corner_cloud.header = depth_msg->header;
    rgb_points_pub_.publish(rgb_corner_cloud);
    depth_points_pub_.publish(depth_corner_cloud);
    */
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  rgbd_camera_calibration::CirclesGridDetector* rgb_detector_;
  rgbd_camera_calibration::CirclesGridDetector* depth_detector_;

  ros::Publisher rgb_points_pub_;
  ros::Publisher depth_points_pub_;

  image_geometry::PinholeCameraModel rgb_camera_model_;
  image_geometry::PinholeCameraModel depth_camera_model_;
};

} // end of namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_camera_check");
  ros::NodeHandle n;
  rgbd_camera_calibration::RgbdCameraCheck check(n);
  ros::spin();
  return 0;
}

