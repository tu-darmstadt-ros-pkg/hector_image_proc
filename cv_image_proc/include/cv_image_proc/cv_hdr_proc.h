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

#ifndef CV_HDR_PROC_H___
#define CV_HDR_PROC_H___

#include <opencv/cv.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "opencv2/core/version.hpp"

#if CV_MAJOR_VERSION == 2
  //#include <opencv2/nonfree/nonfree.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/photo.hpp>
  #include <opencv2/core.hpp>
#endif

#include <cv_bridge/cv_bridge.h>

namespace cv_hdr_proc{

class HdrProcessor{

public:

    bool createExposureFusionImage(const std::vector<cv::Mat>& images,
                        const std::vector<float>& exposure_times,
                        cv::Mat& result)
    {
#if CV_MAJOR_VERSION == 2
  return false;
#elif CV_MAJOR_VERSION == 3
  ROS_INFO("Create HDR");



  //ROS_INFO("Pre Calibrate");

  //for (size_t i = 0; i < images.size(); ++i)
 // {
    //cv::Mat bla;
    //bla.depth()
   // std::cout << "d: " << images[i].depth() << " ch: " << images[i].channels() << "\n";
  //}
  //cv::Mat response;
  //cv::Ptr<cv::CalibrateDebevec> calibrate = cv::createCalibrateDebevec();
  //calibrate->process(images, response, exposure_times);

  //ROS_INFO("Post Calibrate");
  //cv::Mat fusion;
  //cv::Ptr<cv::MergeMertens> merge_mertens = cv::createMergeMertens();
  //merge_mertens->process(images, fusion);
  //ROS_INFO("Post merge");

  //Mat hdr;
  //ROS_INFO_STREAM("ims: " << images.size() << " exps :" << exposure_times.size() << "\n");


  //cv::Mat hdr;
  //cv::Ptr<cv::MergeDebevec> merge_debevec = cv::createMergeDebevec();
  //merge_debevec->process(images, hdr, exposure_times);

  //cv::Ptr<cv::MergeDebevec> merge_debevec = cv::createMergeDebevec();
  //merge_debevec->process(images, result, exposure_times, response);
  //cv::Mat fusion;
  cv::Ptr<cv::MergeMertens> merge_mertens = cv::createMergeMertens();
  merge_mertens->process(images, result);
  ROS_INFO("Done");
  return true;
#endif
    }

    bool createExposureFusionImage(const std::vector<sensor_msgs::Image>& images,
                        const std::vector<sensor_msgs::CameraInfo>& camera_infos,
                        const std::vector<float>& exposure_times,
                        cv::Mat& result)
    {

      ROS_INFO("Create HDR");

      std::vector<cv::Mat> images_cv;
      std::vector<cv_bridge::CvImageConstPtr> cv_ptr_images;

      //std::vector<float> exposure_times_cpy = exposure_times;
      cv::Mat exposure_times_cpy(exposure_times);

      //@TODO Make this nicer (less copying)
      for(size_t i = 0; i < images.size(); ++i)
      {
        cv_ptr_images.push_back(cv_bridge::toCvCopy(images[i]));
        images_cv.push_back(cv_ptr_images[i]->image);
        //exposure_times_cpy.push_back(1.0/ exposure_times[i]);

        //int a = i;
        //std::stringstream ss;
        //ss << a;
        //std::string str = ss.str();
        //cv::imwrite("test_zoom" + str +".png",cv_ptr_images[i]->image);
      }

      return createExposureFusionImage(images_cv, exposure_times, result);

    }


    bool createExposureFusionImage(const std::vector<sensor_msgs::ImageConstPtr>& images,
                        const std::vector<sensor_msgs::CameraInfoConstPtr>& camera_infos,
                        const std::vector<float>& exposure_times,
                        cv::Mat& result)
    {

      ROS_INFO("Create HDR");

      std::vector<cv::Mat> images_cv;
      std::vector<cv_bridge::CvImageConstPtr> cv_ptr_images;

      //std::vector<float> exposure_times_cpy = exposure_times;
      //cv::Mat exposure_times_cpy(exposure_times);

      //@TODO Make this nicer (less copying)
      for(size_t i = 0; i < images.size(); ++i)
      {
        cv_ptr_images.push_back(cv_bridge::toCvShare(images[i]));
        images_cv.push_back(cv_ptr_images[i]->image);
      }

      return createExposureFusionImage(images_cv, exposure_times, result);

    }

   //


    //void ();
};


}

#endif
