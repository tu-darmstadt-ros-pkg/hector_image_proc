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

bool generatePerspectiveTransform(cv::Mat& perspective_transform,
                        const std::vector<cv::Point2d> points_image_coords,
                        const cv::Size& target_size_pixels)
{
  cv::Point2f src[4];
  cv::Point2f dst[4];


  // Order BOTTOM_LEFT, UPPER_LEFT, UPPER_RIGHT, BOTTOM_RIGHT
  for (size_t i = 0; i < 4; ++i){
    src[i] = points_image_coords[i];
  }

  double aspect_ratio = target_size_pixels.width / target_size_pixels.height;

  float x_max = target_size_pixels.width;
  float y_max = target_size_pixels.height;

  dst[BOTTOM_LEFT]  = cv::Point2f(0.0f,0.0f);
  dst[UPPER_LEFT]   = cv::Point2f(0.0f,y_max);
  dst[UPPER_RIGHT]  = cv::Point2f(x_max, y_max);
  dst[BOTTOM_RIGHT] = cv::Point2f(x_max,0.0f);

  perspective_transform = cv::getPerspectiveTransform(src, dst);

  return true;

}

void getPerspectiveTransformedImage(const std::vector<cv::Point2d> points_image_coords_,
                                    const cv::Mat& source_img,
                                    cv::Mat& target_img,
                                    const cv::Size& target_size_pixels,
                                    const int interpolation_mode,
                                    const int border_mode,
                                    const cv::Scalar& border_value)
{

  cv::Mat perspective_transform;

  generatePerspectiveTransform(perspective_transform,
                          points_image_coords_,
                          target_size_pixels);

  //cv::warpPerspective(source_img, target_img, M, target_size_pixels, CV_INTER_AREA, cv::BORDER_CONSTANT , heatval );
  cv::warpPerspective(source_img, target_img, perspective_transform, target_size_pixels, interpolation_mode, border_mode, border_value);
}

bool getImageCoords(std::vector<cv::Point2d>& points_image_coords,
                    const sensor_msgs::CameraInfoConstPtr& cam_info,
                    const std::string& target_frame,
                    const geometry_msgs::Pose& pose_object_frame,
                    const std::vector<Eigen::Vector3d>& points_object_frame,
                    tf::Transformer& transformer,
                    const bool allow_pose_off_camera)
{
  if (points_object_frame.size() != 4)
  {
    ROS_ERROR("Need exactly four points in object frame for perspective warp, aborting!");
    return false;
  }

  tf::Pose object_pose_world_tf;
  tf::poseMsgToTF(pose_object_frame, object_pose_world_tf);

  //ROS_INFO_STREAM("\n" << object_pose_world << "\n");

  tf::StampedTransform trans_to_camera_frame;
  try{
    transformer.waitForTransform(cam_info->header.frame_id, target_frame, ros::Time(0), ros::Duration(0.5));
    transformer.lookupTransform(cam_info->header.frame_id, target_frame,
                                  ros::Time(0), trans_to_camera_frame);
    //listener->lookupTransform("arm_stereo_left_camera_optical_frame", "world",
    //                          ros::Time(0), trans_to_camera_frame);

  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Lookup Transform failed: %s",ex.what());
    return false;
  }

  tf::Pose object_pose_camera_tf = trans_to_camera_frame * object_pose_world_tf;

  image_geometry::PinholeCameraModel cam_model;

  cam_model.fromCameraInfo(*cam_info);

  cv::Point2d object_camera_pixels = cam_model.project3dToPixel(
        cv::Point3d(object_pose_camera_tf.getOrigin().getX(),
                    object_pose_camera_tf.getOrigin().getY(),
                    object_pose_camera_tf.getOrigin().getZ()));

  if (!allow_pose_off_camera){
    // Note CV vs standard ROS coords
    if((object_camera_pixels.x < 0.0) || (object_camera_pixels.x > cam_info->width) ||
       (object_camera_pixels.y < 0.0) || (object_camera_pixels.y > cam_info->height))
    {
      ROS_WARN("Projection outside image. Coords: %f, %f",object_camera_pixels.x, object_camera_pixels.y);
      return false;
    }
  }

  //ROS_INFO("Projection. Coords: %f, %f",object_camera_pixels.x, object_camera_pixels.y);



  //cv::Mat img = cv_ptr->image;

  //image_pub.publish(cv_ptr);

  Eigen::Affine3d transform_camera_t_world;
  tf::transformTFToEigen(trans_to_camera_frame, transform_camera_t_world);

  Eigen::Affine3d transform_world_t_object;
  tf::poseMsgToEigen(pose_object_frame, transform_world_t_object);

  //geometry_msgs::PolygonStamped plane_poly;
  //plane_poly.header = latest_img_->header;
  //plane_poly.header.frame_id = "arm_stereo_left_camera_optical_frame";

  //plane_poly.polygon.points.resize(4);

  //std::vector<cv::Point2d> points_image_coords_;
  points_image_coords.resize(4);

  for (size_t i = 0; i < 4; ++i)
  {

    Eigen::Vector3d point_cam = transform_camera_t_world *
        transform_world_t_object *
        points_object_frame[i];

    points_image_coords[i] = cam_model.project3dToPixel(
          cv::Point3d(point_cam.x(), point_cam.y(), point_cam.z()));


    //plane_poly.polygon.points[i].x = point_cam.x();
    //plane_poly.polygon.points[i].y = point_cam.y();
    //plane_poly.polygon.points[i].z = point_cam.z();

    //std::cout << "\nx: " << object_rectify_plane_points_[i].point_image.x << " y: " <<
    //            object_rectify_plane_points_[i].point_image.y <<"\n";
  }

  return true;

}


WarpProvider::WarpProvider(boost::shared_ptr<tf::Transformer> transformer_in,
                           const std::string target_frame_in)
  : transformer_(transformer_in)
  , target_frame_(target_frame_in)
{

}

bool WarpProvider::getWarpedImage(const sensor_msgs::ImageConstPtr& image,
                                  const sensor_msgs::CameraInfoConstPtr& cam_info,
                                  const geometry_msgs::Pose& pose_object_frame,
                                  const std::vector<Eigen::Vector3d>& points_object_frame,
                                  const cv::Size& target_size_pixels,
                                  cv_bridge::CvImagePtr out_msg,
                                  const int interpolation_mode,
                                  const int border_mode,
                                  const cv::Scalar& border_value,
                                  const bool allow_pose_off_camera
                                 )
{
  std::vector<cv::Point2d> points_image_coords;
  bool got_image_coords = getImageCoords(points_image_coords,
                                         cam_info,
                                         target_frame_,
                                         pose_object_frame,
                                         points_object_frame,
                                         *transformer_,
                                         allow_pose_off_camera);

  if (!got_image_coords)
      return false;
  
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    //if (sensor_msgs::image_encodings::isColor(latest_img_->encoding))
    //  cv_ptr = cv_bridge::toCvShare(latest_img_, sensor_msgs::image_encodings::BGR8);
    //else
    if(image->encoding.compare("mono16")== 0 || image->encoding.compare("MONO16")== 0 ){
        cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO16);
    }else{
        cv_ptr = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  
  out_msg->header   = image->header;
  out_msg->encoding = cv_ptr->encoding;

  getPerspectiveTransformedImage(points_image_coords,
                                 cv_ptr->image,
                                 out_msg->image,
                                 target_size_pixels, interpolation_mode, border_mode, border_value);
  return true;

}

RemapWarpProvider::RemapWarpProvider(boost::shared_ptr<tf::Transformer> transformer_in,
                           const std::string target_frame_in)
  : transformer_(transformer_in)
  , target_frame_(target_frame_in)
{

}

bool RemapWarpProvider::generateLookupTable(const sensor_msgs::CameraInfoConstPtr& cam_info,
                                            const geometry_msgs::Pose& pose_object_frame,
                                            const std::vector<Eigen::Vector3d>& points_object_frame,
                                            const cv::Size& target_size_pixels)
{

  std::vector<cv::Point2d> points_image_coords;
  bool got_image_coords = getImageCoords(points_image_coords,
                                         cam_info,
                                         target_frame_,
                                         pose_object_frame,
                                         points_object_frame,
                                         *transformer_,
                                         true);

  if (!got_image_coords){
    ROS_ERROR("Failed to retrieve image coords for warping.");
    return false;
  }

  generatePerspectiveTransform(perspective_transform_,
                          points_image_coords,
                          target_size_pixels);

  cv::invert(perspective_transform_, inverse_perspective_transform_);

  // Since the camera won't be moving, let's pregenerate the remap LUT
    
  // Generate the warp matrix
  cv::Mat map_x, map_y, srcTM;
  srcTM = inverse_perspective_transform_.clone(); // If WARP_INVERSE, set srcTM to transformationMatrix

  cv::Size sourceFrameSize = target_size_pixels;
  int sourceFrameCols = target_size_pixels.width;
  int sourceFrameRows = target_size_pixels.height;


  map_x.create(sourceFrameSize, CV_32FC1);
  map_y.create(sourceFrameSize, CV_32FC1);
    
  double M11, M12, M13, M21, M22, M23, M31, M32, M33;
  M11 = srcTM.at<double>(0,0);
  M12 = srcTM.at<double>(0,1);
  M13 = srcTM.at<double>(0,2);
  M21 = srcTM.at<double>(1,0);
  M22 = srcTM.at<double>(1,1);
  M23 = srcTM.at<double>(1,2);
  M31 = srcTM.at<double>(2,0);
  M32 = srcTM.at<double>(2,1);
  M33 = srcTM.at<double>(2,2);
    
  for (int y = 0; y < sourceFrameRows; y++) {
    double fy = (double)y;
    for (int x = 0; x < sourceFrameCols; x++) {
      double fx = (double)x;
      double w = ((M31 * fx) + (M32 * fy) + M33);
      w = w != 0.0f ? 1.f / w : 0.0f;
      float new_x = (float)((M11 * fx) + (M12 * fy) + M13) * w;
      float new_y = (float)((M21 * fx) + (M22 * fy) + M23) * w;
      map_x.at<float>(y,x) = new_x;
      map_y.at<float>(y,x) = new_y;
    }
  }
    
  // Create a fixed-point representation of the mapping
  transformation_x_.create(sourceFrameSize, CV_16SC2);
  transformation_y_.create(sourceFrameSize, CV_16UC1);
  cv::convertMaps(map_x, map_y, transformation_x_, transformation_y_, false);
    
  return true;
}

bool RemapWarpProvider::getWarpedImage(const sensor_msgs::ImageConstPtr& image,
                    const sensor_msgs::CameraInfoConstPtr& cam_info,
                    const geometry_msgs::Pose& pose_object_frame,
                    const std::vector<Eigen::Vector3d>& points_object_frame,
                    const cv::Size& target_size_pixels,
                    cv_bridge::CvImagePtr out_msg,
                    const int interpolation_mode,
                    const int border_mode,
                    const cv::Scalar& border_value,
                    const bool allow_pose_off_camera
                   )
{
  if ((points_object_frame_ != points_object_frame) ||
      (target_size_pixels_ != target_size_pixels)){
    ROS_INFO("[RemapWarpProvider] Generating warp remap lookup table.");
    if (generateLookupTable(cam_info,
                        pose_object_frame,
                        points_object_frame,
                        target_size_pixels)){
      points_object_frame_ = points_object_frame;
      target_size_pixels_  = target_size_pixels;
    }else{
      return false;
    }
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  out_msg->header   = image->header;
  out_msg->encoding = cv_ptr->encoding;

  cv::remap(cv_ptr->image,
            out_msg->image,
            transformation_x_,
            transformation_y_,
            interpolation_mode,
            border_mode,
            border_value);

  return true;

}

const cv::Mat& RemapWarpProvider::getPerspectiveTransform() const
{
  return perspective_transform_;
}

const cv::Mat& RemapWarpProvider::getInversePerspectiveTransform() const
{
  return inverse_perspective_transform_;
}

}
