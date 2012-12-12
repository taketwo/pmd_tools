/******************************************************************************
 * Copyright (c) 2012 Sergey Alexandrov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt32.h>

#include <pmd_tools/SaturationFilterConfig.h>

namespace pmd_tools
{

class SaturationFilterNodelet : public nodelet::Nodelet
{

public:

  SaturationFilterNodelet()
  : synchronizer_(SyncPolicy(10), depth_subscriber_, amplitude_subscriber_)
  {
  }

private:

  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pn = getPrivateNodeHandle();
    amplitude_subscriber_.subscribe(nh, "amplitude/image", 1);
    depth_subscriber_.subscribe(nh, "depth/image", 1);
    synchronizer_.registerCallback(boost::bind(&SaturationFilterNodelet::imagesCallback, this, _1, _2));

    // Advertise topics
    depth_filtered_publisher_ = nh.advertise<sensor_msgs::Image>("depth/filtered", 1);
    saturated_fixels_publisher_ = pn.advertise<std_msgs::UInt32>("saturated_pixels", 1);

    // Setup dynamic reconfigure server
    reconfigure_server_.reset(new ReconfigureServer(pn));
    reconfigure_server_->setCallback(boost::bind(&SaturationFilterNodelet::reconfigureCallback, this, _1, _2));
  }

  void imagesCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& amplitude_msg)
  {
    size_t width = depth_msg->width;
    size_t height = depth_msg->height;
    if (width != amplitude_msg->width || height != amplitude_msg->height)
    {
      NODELET_WARN("Width and height mismatch, skipping...");
    }
    const float* dd = reinterpret_cast<const float*>(&depth_msg->data[0]);
    const float* ad = reinterpret_cast<const float*>(&amplitude_msg->data[0]);
    sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
    msg->header.stamp = depth_msg->header.stamp;
    msg->header.frame_id = depth_msg->header.frame_id;
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
        if (ad[i] > config_.saturation_threshold)
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
    std_msgs::UInt32 pixels_msg;
    pixels_msg.data = rc;
    saturated_fixels_publisher_.publish(pixels_msg);
    depth_filtered_publisher_.publish(msg);
  }

  void reconfigureCallback(pmd_tools::SaturationFilterConfig &config, uint32_t level)
  {
    config_ = config;
  }

private:

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> depth_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> amplitude_subscriber_;
  Synchronizer synchronizer_;
  ros::Publisher saturated_fixels_publisher_;
  ros::Publisher depth_filtered_publisher_;

  typedef dynamic_reconfigure::Server<pmd_tools::SaturationFilterConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  pmd_tools::SaturationFilterConfig config_;

};

}

// Register as a nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (pmd_tools, saturation_filter, pmd_tools::SaturationFilterNodelet, nodelet::Nodelet);

