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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

namespace cv_line_tools{

class Line
{
public:
  Line(const cv::Vec2f& line_in){
    double rho   = static_cast<double>(line_in[0]);
    double theta = static_cast<double>(line_in[1]);

    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;

    start = Eigen::Vector2d(x0 + 1000.0 * (-b), y0 + 1000.0*(a));
    end   = Eigen::Vector2d(x0 - 1000.0 * (-b), y0 - 1000.0*(a));
  }

  Line(const cv::Vec4i& line_in)
  {
    start = Eigen::Vector2d(static_cast<double> (line_in[0]), static_cast<double> (line_in[1]));
    end   = Eigen::Vector2d(static_cast<double> (line_in[2]), static_cast<double> (line_in[3]));
  }

  double distanceToPoint(const cv::Point2d& point) const
  {

    double dist_line_points_squared = (start - end).squaredNorm();

    double u =
        ( ( ( point.x - start.x() ) * ( end.x() - start.x() ) ) +
          ( ( point.y - start.y() ) * ( end.y() - start.y() ) ) ) /
        ( dist_line_points_squared );

    Eigen::Vector2d closest_point_on_line = start + (end - start) * u;

    double dist = (Eigen::Vector2d(point.x, point.y) - closest_point_on_line).norm();

    return dist;
    /*
    double dist =  std::fabs(
          (end.y() - start.y()) * point.x - (end.x() - start.x()) * point.y
          + end.x() * start.y()  + end.y() * start.x()) /
        std::sqrt(
          //std::pow(end.y() - start.y(), 2) + std::pow(end.x() - start.x(), 2)
          (end.y() - start.y())*(end.y() - start.y()) + (end.x() - start.x())*(end.x() - start.x())
          );
    */

    /*
    std::cout << "\nstart:\n" << start << "\nend:\n" << end << "\n";
    std::cout << "point:\n" << point.x << " " << point.y << " dist: " << dist << "\n";

    return dist;

    return std::fabs(
          (end.y() - start.y()) * point.x - (end.x() - start.x()) * point.y
          + end.x() * start.y()  + end.y() * start.x()) /
        std::sqrt(
          //std::pow(end.y() - start.y(), 2) + std::pow(end.x() - start.x(), 2)
          (end.y() - start.y())*(end.y() - start.y()) + (end.x() - start.x())*(end.x() - start.x())
          );
          */
  }

  cv::Point2d getStart() const
  {
    return cv::Point2d(start.x(), start.y());
  }

  cv::Point2d getEnd() const
  {
    return cv::Point2d(end.x(), end.y());
  }

  double getAngle() const
  {
    Eigen::Vector2d diff (end - start);

    double angle = std::atan2(diff.y(), diff.x()) - M_PI * 0.5;

    //std::cout << "\nangle" << angle;

    //Cannot really make out which direction we are pointing here,
    //so normalize to one half of circle.
    if (angle < -M_PI*0.5){
      angle += M_PI;
    }else if (angle > M_PI * 0.5){
      angle -= M_PI;
    }

    return angle;
  }

protected:
  Eigen::Vector2d start;
  Eigen::Vector2d end;

};

class LineContainer
{
public:
  LineContainer(const std::vector<cv::Vec2f>& lines)
  {
    for (size_t i = 0; i < lines.size(); ++i)
    {
      lines_.push_back(Line(lines[i]));
    }
  }

  LineContainer(const std::vector<cv::Vec4i>& lines)
  {
    for (size_t i = 0; i < lines.size(); ++i)
    {
      lines_.push_back(Line(lines[i]));
    }
  }

  void removeLinesThresholdDistanceFromPoint(const cv::Point2d point, double threshold_distance)
  {
    std::vector<Line> tmp;

    for (size_t i = 0; i < lines_.size(); ++i)
    {
      if (lines_[i].distanceToPoint(point) < threshold_distance){
        tmp.push_back(lines_[i]);
      }
    }

    lines_ = tmp;
  }

  double getAverageAngle() const
  {
    if (lines_.size() != 0)
    {
      return lines_[0].getAngle();

    }else{
      return -999.0;
    }
    /*
    for (size_t i = 0; i < lines_.size(); ++i)
    {
      Eigen::Vector2d diff (end - start);

      double angle = std::atan2(diff.y(), diff.x());

      //Cannot really make out which direction we are pointing,
      //so normalize to one half of circle.
      if (angle < M_PI*0.5){
        angle += M_PI*0.5;
      }else if (angle > M_PI * 0.5){
        angle -= M_PI * 0.5;
      }
    }
    */

  }

  const Line& getLine(size_t index) { return lines_[index]; };

  size_t size() const { return lines_.size(); };

protected:
  std::vector<Line> lines_;

};

}

#endif
