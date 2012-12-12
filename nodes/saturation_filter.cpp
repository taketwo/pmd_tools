#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

class SaturationFilterNode
{

public:

  SaturationFilterNode()
  : depth_subscriber_(nh_, "camera/depth/image", 1)
  , amplitude_subscriber_(nh_, "camera/amplitude/image", 1)
  , synchronizer_(SyncPolicy(10), depth_subscriber_, amplitude_subscriber_)
  {
    depth_filtered_publisher_ = nh_.advertise<sensor_msgs::Image>("depth_filtered", 1);
    synchronizer_.registerCallback(boost::bind(&SaturationFilterNode::imagesCallback, this, _1, _2));
    ROS_INFO("Started [saturation_filter] node.");
  }

  void imagesCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& amplitude_msg)
  {
    size_t width = depth_msg->width;
    size_t height = depth_msg->height;
    if (width != amplitude_msg->width || height != amplitude_msg->height)
    {
      ROS_WARN("Width and height mismatch, skipping...");
    }
    const float* dd = reinterpret_cast<const float*>(&depth_msg->data[0]);
    const float* ad = reinterpret_cast<const float*>(&amplitude_msg->data[0]);
    sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
    msg->header.stamp = depth_msg->header.stamp;
    msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    msg->height = height;
    msg->width = width;
    msg->step = msg->width * sizeof(float);
    msg->data.resize(msg->height * msg->step);
    float* ddn = reinterpret_cast<float*>(&msg->data[0]);
    size_t i = 0;
    size_t rc = 0;
    for (size_t x = 0; x < width; x++)
    {
      for (size_t y = 0; y < height; y++, i++)
      {
        if (ad[i] > 18000)
        {
          ddn[i] = std::numeric_limits<float>::quiet_NaN();
          rc++;
        }
        else
        {
          ddn[i] = dd[i];
        }
      }
    }
    ROS_INFO("Removed %zu saturated pixels", rc);
    depth_filtered_publisher_.publish(msg);
  }

private:

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> depth_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> amplitude_subscriber_;
  Synchronizer synchronizer_;
  ros::Publisher depth_filtered_publisher_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "saturation_filter");
  SaturationFilterNode sfn;
  ros::spin();
  return 0;
}
