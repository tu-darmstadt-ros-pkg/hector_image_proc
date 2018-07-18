#include <cv_debug_provider/cv_debug_provider.h>
CvDebugProvider::CvDebugProvider(ros::NodeHandle nh, const std::string encoding, bool latch)
  : encoding_(encoding)
{
  encoding_ = sensor_msgs::image_encodings::BGR8;

  it_out_ .reset(new image_transport::ImageTransport(nh));
  image_pub_ = it_out_->advertise("debug_image_array", 4, latch);
}


bool CvDebugProvider::addDebugImage(const cv::Mat& img)
{
  if (image_pub_.getNumSubscribers() > 0)
  {
    if (img.type() == CV_16UC1){
      cv::Mat tmp, converted_image;
      img.convertTo(tmp, CV_8UC1, 0.00390625);
      cv::cvtColor(tmp, converted_image, CV_GRAY2BGR);
      debug_img_vector_.push_back(converted_image);
    }else if (img.type() == CV_8UC1 ){
      cv::Mat converted_image;
      cv::cvtColor(img, converted_image, CV_GRAY2BGR);
      debug_img_vector_.push_back(converted_image);
    }else if (img.type() == CV_8UC3){
      debug_img_vector_.push_back(img);
    }else if (img.type() == CV_32FC1){
      cv::Mat debug_out_depth_UC8;

      //@TODO: Be more clever about finding max and min
      double max_val = 1.0;
      double min_val = 0.0;
      const double alpha = 255.0 / (max_val - min_val);
      const double beta = -alpha * min_val;
      img.convertTo(debug_out_depth_UC8, CV_8UC1, alpha, beta);

      cv::Mat converted_image;
      cv::cvtColor(debug_out_depth_UC8, converted_image, CV_GRAY2BGR);
      debug_img_vector_.push_back(converted_image);
    }else if (img.type() == CV_8UC4){
      //ROS_ERROR("Type CV_8UC4");
      cv::Mat converted_image;
      cv::cvtColor(img, converted_image, CV_BGRA2BGR);
      debug_img_vector_.push_back(converted_image);
    }else{
      ROS_ERROR("Unknown image encoding: %d", img.type());
    }
  }

  return true;
}

void CvDebugProvider::publishDebugImage(bool concat_vertical)
{
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "only_for_debugging_no_valid_frame_id";
    out_msg.encoding = encoding_;

    if (!concat_vertical){
      cv::hconcat(debug_img_vector_, out_msg.image);
    }else{
      cv::vconcat(debug_img_vector_, out_msg.image);
    }

    image_pub_.publish(out_msg.toImageMsg());

    debug_img_vector_.clear();
  }
}

bool CvDebugProvider::areDebugImagesRequested() const
{
  return (image_pub_.getNumSubscribers() > 0);
}

cv::Mat& CvDebugProvider::getLastAddedImage()
{
  //@Todo: Properly use iterator
  size_t size = debug_img_vector_.size();
  return debug_img_vector_[size-1];
}
