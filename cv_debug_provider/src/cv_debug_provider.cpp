#include <cv_debug_provider/cv_debug_provider.h>



CvDebugProvider::CvDebugProvider(ros::NodeHandle nh, const std::string encoding)
  : encoding_(encoding)
{
  encoding_ = sensor_msgs::image_encodings::BGR8;

  it_out_ .reset(new image_transport::ImageTransport(nh));
  image_pub_ = it_out_->advertise("debug_image_array", 4);
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
    }else {
        ROS_ERROR("Unknown image encoding: %d", img.type());
    }
  }

  return true;
}

void CvDebugProvider::publishDebugImage()
{
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "only_for_debugging_no_valid_frame_id";
    out_msg.encoding = encoding_;

    cv::hconcat(debug_img_vector_, out_msg.image);

    image_pub_.publish(out_msg.toImageMsg());

    debug_img_vector_.clear();
  }
}

bool CvDebugProvider::areDebugImagesRequested() const
{
  return (image_pub_.getNumSubscribers() > 0);
}
