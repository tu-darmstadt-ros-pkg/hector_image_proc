//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef CV_IMAGE_WARP_H___
#define CV_IMAGE_WARP_H___

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

namespace cv_image_warp{

enum corners {BOTTOM_LEFT, UPPER_LEFT, UPPER_RIGHT, BOTTOM_RIGHT};


/**
 * @brief getSampleRectangleObjectPoints is a convenience function
 * for specifying a rectangle on the yz plane.
 * @param rectangle_size Directly used for the rectangle corners
 * @return vector of points
 */
std::vector<Eigen::Vector3d> getSampleRectangleObjectPoints(double size);

/**
 * @brief getPerspectiveTransformedImage generates a perspective
 * transformed image target_img for the four corner points provided
 * @param points_image_coords_ The four corner points for the warped
 * cutout
 * @param source_img The source image
 * @param target_img The target image
 * @param target_size_pixels The size in pixels for the target image
 * @return vector of points
 */
void getPerspectiveTransformedImage(const std::vector<cv::Point2d> points_image_coords_,
                                    const cv::Mat& source_img,
                                    cv::Mat& target_img,
                                    const cv::Size& target_size_pixels,
                                    const int interpolation_mode=cv::INTER_CUBIC,
                                    const int border_mode=cv::BORDER_CONSTANT,
                                    const cv::Scalar& border_value=cv::Scalar());

class WarpProvider{
public:

  WarpProvider(boost::shared_ptr<tf::Transformer> transformer_in,
               const std::string target_frame_in = std::string("map"));

  /**
   * @brief getWarpedImage generates a perspective warped image for a
   * ROI in the provided image and for the provided object data
   * @param image The image to use
   * @param cam_info The camera info for the input image
   * @param pose_object_frame The pose of the object of interest
   * @param points_object_frame Four points given in the object's
   * frame of reference that are the corner points of the image
   * that is provided
   * @param target_size_pixels The size in pixels for the target image
   * @param interpolation_mode The interpolation mode for border,
   * @param border_mode The border mode value type,
   * @param border_value The value used for border,
   * @param allow_pose_off_camera Indicates if the pose of object
   * might be outside camera fov
   * @return Indication if operation was successful
   */
  bool getWarpedImage(const sensor_msgs::ImageConstPtr& image,
                      const sensor_msgs::CameraInfoConstPtr& cam_info,
                      const geometry_msgs::Pose& pose_object_frame,
                      const std::vector<Eigen::Vector3d>& points_object_frame,
                      const cv::Size& target_size_pixels,
                      cv_bridge::CvImagePtr out_msg,
                      const int interpolation_mode=cv::INTER_CUBIC,
                      const int border_mode=cv::BORDER_CONSTANT,
                      const cv::Scalar& border_value=cv::Scalar(),
                      const bool allow_pose_off_camera = false
                     );

protected:
  boost::shared_ptr<tf::Transformer> transformer_;
  std::string target_frame_;

};

class RemapWarpProvider{
public:
    
  RemapWarpProvider(boost::shared_ptr<tf::Transformer> transformer_in,
                    const std::string target_frame_in = std::string("map"));
  
  bool generateLookupTable(const sensor_msgs::CameraInfoConstPtr& cam_info,
                           const geometry_msgs::Pose& pose_object_frame,
                           const std::vector<Eigen::Vector3d>& points_object_frame,
                           const cv::Size& target_size_pixels);
  
  bool getWarpedImage(const sensor_msgs::ImageConstPtr& image,
                      const sensor_msgs::CameraInfoConstPtr& cam_info,
                      const geometry_msgs::Pose& pose_object_frame,
                      const std::vector<Eigen::Vector3d>& points_object_frame,
                      const cv::Size& target_size_pixels,
                      cv_bridge::CvImagePtr out_msg,
                      const int interpolation_mode=cv::INTER_CUBIC,
                      const int border_mode=cv::BORDER_CONSTANT,
                      const cv::Scalar& border_value=cv::Scalar(),
                      const bool allow_pose_off_camera = false,
                      cv::Mat *mask_img = 0);

  const cv::Mat& getPerspectiveTransform() const;
  const cv::Mat& getInversePerspectiveTransform() const;

protected:
  boost::shared_ptr<tf::Transformer> transformer_;
  std::string target_frame_;

  cv::Mat transformation_x_;
  cv::Mat transformation_y_;

  cv::Mat perspective_transform_;
  cv::Mat inverse_perspective_transform_;

  std::vector<Eigen::Vector3d> points_object_frame_;
  cv::Size target_size_pixels_;
};


}

#endif
