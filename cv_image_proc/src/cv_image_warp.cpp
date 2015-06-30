//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
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


#include <cv_image_proc/cv_image_warp.h>


#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace cv_image_warp{


std::vector<Eigen::Vector3d> getSampleRectangleObjectPoints(double rectangle_size)
{
  std::vector<Eigen::Vector3d> tmp;
  tmp.resize(4);
  tmp[BOTTOM_LEFT]  = Eigen::Vector3d(0.0, -rectangle_size, -rectangle_size);
  tmp[UPPER_LEFT]   = Eigen::Vector3d(0.0,  rectangle_size, -rectangle_size);
  tmp[UPPER_RIGHT]  = Eigen::Vector3d(0.0,  rectangle_size,  rectangle_size);
  tmp[BOTTOM_RIGHT] = Eigen::Vector3d(0.0, -rectangle_size,  rectangle_size);
  return tmp;
}

void getPerspectiveTransformedImage(const std::vector<cv::Point2d> points_image_coords_,
                                    const cv::Mat& source_img,
                                    cv::Mat& target_img,
                                    const cv::Size& target_size_pixels)
{
  cv::Point2f src[4];
  cv::Point2f dst[4];


  // Order BOTTOM_LEFT, UPPER_LEFT, UPPER_RIGHT, BOTTOM_RIGHT
  for (size_t i = 0; i < 4; ++i){
    src[i] = points_image_coords_[i];
  }

  double aspect_ratio = target_size_pixels.width / target_size_pixels.height;

  float x_max = target_size_pixels.width;
  float y_max = target_size_pixels.height;

  dst[BOTTOM_LEFT]  = cv::Point2f(0.0f,0.0f);
  dst[UPPER_LEFT]   = cv::Point2f(0.0f,y_max);
  dst[UPPER_RIGHT]  = cv::Point2f(x_max, y_max);
  dst[BOTTOM_RIGHT] = cv::Point2f(x_max,0.0f);

  cv::Mat M = cv::getPerspectiveTransform(src, dst);
  cv::warpPerspective(source_img, target_img, M, target_size_pixels, CV_INTER_LANCZOS4);
}


WarpProvider::WarpProvider(boost::shared_ptr<tf::Transformer> transformer_in,
                           const std::string target_frame_in)
  : transformer_(transformer_in)
  , target_frame_(target_frame_in)
{

}

bool WarpProvider::getWarpedImage(const sensor_msgs::ImageConstPtr& image,
                                  const sensor_msgs::CameraInfoConstPtr& cam_info,
                                  const geometry_msgs::Pose& pose_target_frame,
                                  const std::vector<Eigen::Vector3d>& points_object_frame,
                                  const cv::Size& target_size_pixels,
                                  cv_bridge::CvImagePtr out_msg)
{

  if (points_object_frame.size() != 4)
  {
    ROS_ERROR("Need exactly four points in object frame for perspective warp, aborting!");
    return false;
  }

  tf::Pose object_pose_world_tf;
  tf::poseMsgToTF(pose_target_frame, object_pose_world_tf);

  //ROS_INFO_STREAM("\n" << object_pose_world << "\n");

  tf::StampedTransform trans_to_camera_frame;
  try{
    transformer_->lookupTransform(image->header.frame_id, "world",
                                  ros::Time(0), trans_to_camera_frame);
    //listener->lookupTransform("arm_stereo_left_camera_optical_frame", "world",
    //                          ros::Time(0), trans_to_camera_frame);

  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Lookup Transform failed: %s",ex.what());
    return false;
  }

  ROS_INFO("After tf");


  tf::Pose object_pose_camera_tf = trans_to_camera_frame * object_pose_world_tf;

  image_geometry::PinholeCameraModel cam_model;

  //sensor_msgs::CameraInfo cam_info_fixed = *latest_camera_info_;
  //cam_info_fixed.distortion_model = "plumb_bob";
  //cam_model.fromCameraInfo(cam_info_fixed);
  cam_model.fromCameraInfo(*cam_info);

  cv::Point2d object_camera_pixels = cam_model.project3dToPixel(
        cv::Point3d(object_pose_camera_tf.getOrigin().getX(),
                    object_pose_camera_tf.getOrigin().getY(),
                    object_pose_camera_tf.getOrigin().getZ()));


  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    //if (sensor_msgs::image_encodings::isColor(latest_img_->encoding))
    //  cv_ptr = cv_bridge::toCvShare(latest_img_, sensor_msgs::image_encodings::BGR8);
    //else
    cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  //cv::Mat img = cv_ptr->image;

  //image_pub.publish(cv_ptr);

  Eigen::Affine3d transform_camera_t_world;
  tf::transformTFToEigen(trans_to_camera_frame, transform_camera_t_world);

  Eigen::Affine3d transform_object_t_world;
  tf::poseMsgToEigen(pose_target_frame, transform_object_t_world);

  //geometry_msgs::PolygonStamped plane_poly;
  //plane_poly.header = latest_img_->header;
  //plane_poly.header.frame_id = "arm_stereo_left_camera_optical_frame";

  //plane_poly.polygon.points.resize(4);

  std::vector<cv::Point2d> points_image_coords_;
  points_image_coords_.resize(4);

  for (size_t i = 0; i < 4; ++i)
  {

    Eigen::Vector3d point_cam = transform_camera_t_world *
        transform_object_t_world *
        points_object_frame[i];

    points_image_coords_[i] = cam_model.project3dToPixel(
          cv::Point3d(point_cam.x(), point_cam.y(), point_cam.z()));


    //plane_poly.polygon.points[i].x = point_cam.x();
    //plane_poly.polygon.points[i].y = point_cam.y();
    //plane_poly.polygon.points[i].z = point_cam.z();

    //std::cout << "\nx: " << object_rectify_plane_points_[i].point_image.x << " y: " <<
    //            object_rectify_plane_points_[i].point_image.y <<"\n";
  }

  //poly_pub.publish(plane_poly);

  //cv::Mat rectified_image;
  //cv_bridge::CvImage out_msg;
  out_msg->header   = image->header;
  out_msg->encoding = cv_ptr->encoding;

  getPerspectiveTransformedImage(points_image_coords_,
                                 cv_ptr->image,
                                 out_msg->image,
                                 target_size_pixels);
  return true;
  
}



}
