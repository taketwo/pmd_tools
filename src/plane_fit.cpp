#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include "aliases.h"

class PlaneFitNode
{

public:

  PlaneFitNode()
  : polygon_visualizer_("fitted_plane", "/camera_optical_frame", Color::TEAL)
  {
    ss_.setOptimizeCoefficients(true);
    ss_.setModelType(pcl::SACMODEL_PLANE);
    ss_.setMethodType(pcl::SAC_RANSAC);
    ss_.setDistanceThreshold(0.01);
    pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    ros::Subscriber points_subscriber = nh.subscribe("/camera/points", 1, boost::bind(&PlaneFitNode::pointsCallback, this, _1));
  }

  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Fit a plane
    PointCloud::Ptr input_cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *input_cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ss_.setInputCloud(input_cloud);
    ss_.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      ROS_WARN("Unable to fit a plane!");
      return;
    }
    pcl::SampleConsensusModelPtr model = ss_.getModel();
    std::vector<double> distances;
    Eigen::VectorXf coeff;
    memcpy(&coeff[0], &coefficients->values[0], coefficients->values.size() * sizeof(coefficients->values[0]));
    model->getDistancesToModel(coeff, distances);
    double sum = 0;
    for (size_t i = 0; i < inliers->indices.size(); i++)
    {
      sum += distances[i];
    }
    ROS_INFO("Average deviation among inliers: %.3f", sum);
    // Visualization
    // The following processing is only for the visualization purposes
    // Project points onto the plane
    PointCloud::Ptr plane_cloud(new PointCloud);
    pi_.setInputCloud(input_cloud);
    pi_.setIndices(inliers);
    pi_.setModelCoefficients(coefficients);
    pi_.filter(*plane_cloud);
    // Compute convex hull around boundary points
    pcl::ConvexHull<PointT> convex_hull;
    PointCloud::Ptr plane_hull(new PointCloud);
    convex_hull.setDimension(2);
    convex_hull.setInputCloud(plane_cloud);
    convex_hull.reconstruct(*plane_hull);
    // Create and publish planar polygon
    polygon_visualizer_.publish(PlanarPolygon(plane_hull->points, coefficients));
  }

private:

  ros::NodeHandle nh;

  pcl::SACSegmentation<PointT> ss_;
  pcl::ProjectInliers<PointT> pi_;
  hbrs::visualization::PlanarPolygonVisualizer polygon_visualizer_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_fit");
  PlaneFitNode pfn;
  ros::spin();
  return 0;
}
