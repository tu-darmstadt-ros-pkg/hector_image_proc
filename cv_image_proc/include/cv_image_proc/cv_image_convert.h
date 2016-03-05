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

#ifndef CV_LINE_TOOLS_H___
#define CV_LINE_TOOLS_H___

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>

namespace cv_image_convert{

  
/**
 * Converts a FC1 image to UC8 so that min_val maps to 0 and max_val to 255.
 */
bool getUC8ImageFromFC1(const cv::Mat& in, cv::Mat out, double min_val, double max_val)
{
  cv::Mat tmp = in;
  
  for (size_t i = 0; i < tmp.total(); ++i){
    tmp.at<float>(i) -= min_val;
  }
  
  
  const double alpha = 255.0 / (max_val - min_val);
  const double beta = -alpha * min_val;
  cv_ptr->image.convertTo(convertedImage, CV_8UC1, alpha, beta);  
  
  
  cv::Mat upper_thresh;
  cv::threshold(tmp, upper_thresh, 2.5, 0.0, cv::THRESH_TRUNC);

  cv::Mat lower_thresh;
  cv::threshold(upper_thresh, lower_thresh, 0.8, 0.0, cv::THRESH_TOZERO);
  
}


}

#endif
