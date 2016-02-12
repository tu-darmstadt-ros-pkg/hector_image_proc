//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef CV_DEBUG_PROVIDER_H___
#define CV_DEBUG_PROVIDER_H___

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_subscriber.h>


/**
 * This class provides easy to use tooling to quickly
 * publish OpenCV images (i.e. cv::Mat) via ROS and view
 * it remotely.
 */
class CvDebugProvider
{
public:
  /**
   * Contructor, the Nodehandle should be within a unique namespace.
   * The encoding default to BGR8 at the moment and is ignored due
   * to a issue encountered.
   */
  CvDebugProvider(ros::NodeHandle nh, const std::string encoding = sensor_msgs::image_encodings::BGR8);

  /**
   * Adds a debug image if there are subscribers.
   * In case of no subscribers, does not add image.
   * @param img The image to be added
   * @return Indicated success or failure of adding
   */
  bool addDebugImage(const cv::Mat& img);

  /**
    * Publishes debug image if there are subscribers
    * and clears the debug image vector afterwards.
    * This should be called at the end of image processing
    * operations.
    */
  void publishDebugImage();

  /**
   * True if there is a subscriber to the debug image
   * publisher. Can be used to selectively perform computation
   * only when somebody is interested in debug data
   * @return true if there is a subscriber to debug image data
   */
  bool areDebugImagesRequested() const;

  /**
   * @return Reference to the last image added to the internal debug
   * image vector.
   */
  cv::Mat& getLastAddedImage();

protected:

  boost::shared_ptr<image_transport::ImageTransport> it_out_;
  image_transport::Publisher image_pub_;

  sensor_msgs::ImageConstPtr latest_img_;
  sensor_msgs::CameraInfoConstPtr latest_camera_info_;

  std::vector<cv::Mat> debug_img_vector_;
  std::string encoding_;
};



#endif
