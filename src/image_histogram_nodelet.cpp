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
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
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
  }

private:

  virtual void onInit()
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& pn = getPrivateNodeHandle();

    // Subscribe to topics
    image_subscriber_ = nh.subscribe("image", 1, &ImageHistogramNodelet::imageCallback, this);

    // Advertise topics
    histogram_image_publisher_ = pn.advertise<sensor_msgs::CompressedImage>("compressed", 1);

    // Retrieve parameters from server
    int num_bins;
    double min;
    double max;
    pn.param<int>("num_bins", num_bins, 30);
    pn.param<double>("min", min, 0.0);
    pn.param<double>("max", max, 3.0);

    // Initialize histogram
    histogram_ = gsl_histogram_alloc(num_bins);
    gsl_histogram_set_ranges_uniform(histogram_, min, max);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      NODELET_WARN("Unsupported image encoding, should be 32FC1.");
      return;
    }

    // Construct histogram
    size_t outliers = 0;
    const float* data = reinterpret_cast<const float*>(&msg->data[0]);
    gsl_histogram_reset(histogram_);
    for (size_t i = 0; i < msg->width * msg->height; i++)
    {
      float d = *data++;
      if (std::isnan(d))
        continue;
      if (gsl_histogram_increment(histogram_, d))
        outliers++;
    }

    // Plot and publish histogram if there are subscribers
    if (histogram_image_publisher_.getNumSubscribers() > 0)
    {
      sensor_msgs::CompressedImagePtr msg = boost::make_shared<sensor_msgs::CompressedImage>();
      if (plotHistogram(histogram_, msg))
        histogram_image_publisher_.publish(msg);
    }
  }

private:

  bool plotHistogram(gsl_histogram* hist, sensor_msgs::CompressedImagePtr msg)
  {
    // Create a Gnuplot script in a temporary file
    char tmp_filename[L_tmpnam];
    tmpnam(tmp_filename);
    FILE* script = fopen(tmp_filename, "w");
    fprintf(script, "set term png truecolor\n"
                    "set xlabel \"Distance\"\n"
                    "set ylabel \"Samples\"\n"
                    "set yrange [0:15000]\n"
                    "set grid\n"
                    "set boxwidth 0.95 relative\n"
                    "set style fill transparent solid 0.8 noborder\n"
                    "plot '-' u (($1+$2)/2.0):3 w boxes lc rgb\"#4B0082\" notitle\n");

    // Push the histogram data in the file
    gsl_histogram_fprintf(script, hist, "%g", "%g");
    fclose(script);

    // Invoke Gnuplot with created script
    std::stringstream cmd;
    cmd << "gnuplot " << tmp_filename;
    FILE* gnuplot = popen(cmd.str().c_str(), "r");
    if (gnuplot)
    {
      // Fill ROS message with Gnuplot output
      static size_t max_size = 15000;
      msg->data.reserve(max_size);
      while (!ros::isShuttingDown() && !feof(gnuplot))
        msg->data.push_back(fgetc(gnuplot));

      // If the default space limit was exceeded, increase it for future
      if (msg->data.size() > max_size)
        max_size = msg->data.size();

      // Set format and cleanup
      msg->format = "png";
      pclose(gnuplot);
    }
    else
    {
      ROS_ERROR_ONCE("Failed to open pipe to Gnuplot.");
    }

    remove(tmp_filename);
    return (bool)gnuplot;
  }

  ros::Subscriber image_subscriber_;
  ros::Publisher histogram_image_publisher_;

  gsl_histogram* histogram_;

};

}

// Register as a nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (pmd_tools, image_histogram, pmd_tools::ImageHistogramNodelet, nodelet::Nodelet);

