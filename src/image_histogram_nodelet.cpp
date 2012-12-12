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

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <gsl/gsl_histogram.h>

namespace pmd_tools
{

class ImageHistogramNodelet : public nodelet::Nodelet
{

public:

  ImageHistogramNodelet()
  : histogram_(0)
  { }

  virtual ~ImageHistogramNodelet()
  {
    if (histogram_)
      gsl_histogram_free(histogram_);
    if (gnuplot_)
      fclose(gnuplot_);
  }

private:

  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    //ros::NodeHandle& pn = getPrivateNodeHandle();
    image_subscriber_ = nh.subscribe("image", 1, &ImageHistogramNodelet::imageCallback, this);

    gnuplot_ = popen("gnuplot -rv -persist -noraise >gnuplot.log 2>gnuplot.err", "we");
    if (!gnuplot_)
    {
      ROS_ERROR("Failed to open pipe to Gnuplot. No visualization!");
    }
    else
    {
      fprintf(gnuplot_, "set xlabel \"Distance\"\n");
      fprintf(gnuplot_, "set ylabel \"Samples\"\n");
      fprintf(gnuplot_, "set grid\n");
      fprintf(gnuplot_, "set boxwidth 0.95 relative\n");
      fprintf(gnuplot_, "set style fill transparent solid 0.8 noborder\n");
    }

    size_t bins = 10;
    double min = 0.0;
    double max = 10.24;
    histogram_ = gsl_histogram_alloc(bins);
    gsl_histogram_set_ranges_uniform(histogram_, min, max);

    // Advertise topics
    //depth_filtered_publisher_ = nh.advertise<sensor_msgs::Image>("depth/filtered", 1);
    //saturated_fixels_publisher_ = pn.advertise<std_msgs::UInt32>("saturated_pixels", 1);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    size_t missed = 0;
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      NODELET_WARN("Image should be encoded in 32FC1.");
      return;
    }
    const float* data = reinterpret_cast<const float*>(&msg->data[0]);
    gsl_histogram_reset(histogram_);
    for (size_t i = 0; i < msg->width * msg->height; i++)
    {
      if (gsl_histogram_increment(histogram_, *data++))
        missed++;
    }
    //depth_filtered_publisher_.publish(msg);
    if (gnuplot_)
    {
      NODELET_INFO("NEW DATA");
      fprintf(gnuplot_, "plot '-' u (($1+$2)/2.0):3 w boxes lc rgb\"#4B0082\" notitle\n");
      gsl_histogram_fprintf(gnuplot_, histogram_, "%g", "%g");
      fprintf(gnuplot_, "E\n");
    }
  }

private:

  ros::Subscriber image_subscriber_;
  //ros::Publisher saturated_fixels_publisher_;
  //ros::Publisher depth_filtered_publisher_;

  FILE* gnuplot_;
  gsl_histogram* histogram_;

};

}

// Register as a nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (pmd_tools, image_histogram, pmd_tools::ImageHistogramNodelet, nodelet::Nodelet);

