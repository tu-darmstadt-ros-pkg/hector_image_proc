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
bool getUC8ImageFromFC1(const cv::Mat& in, cv::Mat& out, double min_val, double max_val)
{  
  const double alpha = 255.0 / (max_val - min_val);
  const double beta = -alpha * min_val;
  in.convertTo(out, CV_8UC1, alpha, beta);   
}

bool getInpaintedImage(const cv::Mat&in, cv::Mat& out, double min_val, double max_val)
{
  cv::Mat tmp;
  cv_image_convert::getUC8ImageFromFC1(in, tmp, min_val, max_val);

  cv::Mat mask(tmp.size(), CV_8UC1);

  for (size_t i = 0; i < tmp.total(); ++i){
    if (tmp.at<uchar>(i) == 0){
      mask.at<uchar>(i) = 255;
    }else{
      mask.at<uchar>(i) = 0;
    }
  }

  int erosion_type = cv::MORPH_RECT;
  int erosion_size = 2;
  cv::Mat element = cv::getStructuringElement( erosion_type,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );

  cv::Mat eroded;
  cv::erode(mask, eroded, element);
  //eroded = 255 - eroded;

  cv::Mat local_mask;
  cv::compare(mask, eroded, local_mask, cv::CMP_NE);

  cv::inpaint(tmp, local_mask, out, 0.0, cv::INPAINT_NS);
}

bool getGradientMagnitudeImage(const cv::Mat& in, cv::Mat& out)
{
  cv::Mat blurred;
  cv::GaussianBlur( in, blurred, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

  int kernel_size = 3;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_32F;

  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  cv::Sobel( blurred, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
  /// Gradient Y
  cv::Sobel( blurred, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );

  cv::Mat grad_mag_img;
  cv::magnitude(grad_x, grad_y, grad_mag_img);


  cv::convertScaleAbs( grad_mag_img, out );

}


}

#endif
