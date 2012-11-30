#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/moment.hpp>

#include "aliases.h"
#include <planar_polygon_visualizer.h>
#include <clustered_point_cloud_visualizer.h>

using namespace hbrs::visualization;

class PlaneFitNode
{

public:

  PlaneFitNode()
  : polygon_visualizer_("fitted_plane", "/camera_optical_frame", Color::TEAL)
  , cluster_visualizer_("clusters", "/camera_optical_frame")
  {
    ss_.setOptimizeCoefficients(true);
    ss_.setModelType(pcl::SACMODEL_PLANE);
    ss_.setMethodType(pcl::SAC_RANSAC);
    ss_.setDistanceThreshold(0.003);
    eppd_.setHeightLimits(-1.00, 1.00);
    pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/points", 1, boost::bind(&PlaneFitNode::pointsCallback, this, _1));
  }

  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Step 1: fit a plane
    PointCloud::Ptr input_cloud(new PointCloud);
    pcl::fromROSMsg(*msg, *input_cloud);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    ss_.setInputCloud(input_cloud);
    ss_.segment(*plane_inliers, *coefficients);
    if (plane_inliers->indices.size() == 0)
      return;
    //PointCloud::Ptr filtered_inliers_cloud(new PointCloud);
    //ror_.setInputCloud(input_cloud);
    //ror_.setIndices(plane_inliers);
    //ror_.filter(*filtered_inliers_cloud);
    Eigen::Vector4f coeff;
    memcpy(&coeff[0], &coefficients->values[0], coefficients->values.size() * sizeof(coefficients->values[0]));

    // Step 2: compute boundary points (convex hull)
    PointCloud::Ptr plane_cloud(new PointCloud);
    pi_.setInputCloud(input_cloud);
    pi_.setIndices(plane_inliers);
    pi_.setModelCoefficients(coefficients);
    pi_.filter(*plane_cloud);
    // Compute convex hull around boundary points
    pcl::ConvexHull<PointT> convex_hull;
    PointCloud::Ptr plane_hull(new PointCloud);
    convex_hull.setDimension(2);
    convex_hull.setInputCloud(plane_cloud);
    convex_hull.reconstruct(*plane_hull);
    PlanarPolygon plane_polygon(plane_hull->points, coeff);

    // Step 3: get points within the boundary
    pcl::PointIndices::Ptr inner_indices(new pcl::PointIndices);
    eppd_.setInputCloud(input_cloud);
    eppd_.setInputPlanarHull(plane_hull);
    eppd_.segment(*inner_indices);

    auto model = ss_.getModel();
    std::vector<double> distances;
    model->getDistancesToModel(coeff, distances);
    typedef boost::accumulators::tag::mean mean;
    typedef boost::accumulators::tag::moment<2> variance;
    boost::accumulators::accumulator_set<double, boost::accumulators::stats<mean, variance>> d;
    pcl::PointIndices inner_outlier_indices;
    pcl::PointIndices inner_inlier_indices;
    for (const auto& i : inner_indices->indices)
    {
      d(distances[i]);
      if (std::abs(distances[i]) > 0.01)
        inner_outlier_indices.indices.push_back(i);
      else
        inner_inlier_indices.indices.push_back(i);
    }
    double area = computePlanarPolygonArea(plane_polygon);
    ROS_INFO("%.5f, %.7f, %.3f", boost::accumulators::mean(d), boost::accumulators::moment<2>(d), area);

    // Visualization
    std::vector<PointCloud::Ptr> clusters;
    PointCloud::Ptr cluster1(new PointCloud);
    PointCloud::Ptr cluster2(new PointCloud);
    pcl::copyPointCloud(*input_cloud, inner_inlier_indices, *cluster1);
    clusters.push_back(cluster1);
    pcl::copyPointCloud(*input_cloud, inner_outlier_indices, *cluster2);
    clusters.push_back(cluster2);
    polygon_visualizer_.publish(plane_polygon);
    cluster_visualizer_.publish<PointT>(clusters);
  }

private:

  double computePlanarPolygonArea(const PlanarPolygon& polygon)
  {
    const auto& normal = polygon.getCoefficients();
    const auto& points = polygon.getContour();

    // Find axis with largest normal component and project onto perpendicular plane
    int k0, k1, k2;
    k0 = (std::fabs(normal[0]) > std::fabs(normal[1])) ? 0 : 1;
    k0 = (std::fabs(normal[k0]) > std::fabs(normal[2])) ? k0 : 2;
    k1 = (k0 + 1) % 3;
    k2 = (k0 + 2) % 3;

    double area = 0;
    for (size_t i = 0; i < points.size(); i++)
    {
      size_t j = (i + 1) % points.size();
      float p1[3] = { points[i].x, points[i].y, points[i].z };
      float p2[3] = { points[j].x, points[j].y, points[j].z };
      area += p1[k1] * p2[k2] - p1[k2] * p2[k1];
    }

    return std::fabs(area) / (2 * std::fabs(normal[k0]));
  }

  ros::NodeHandle nh_;
  ros::Subscriber points_subscriber_;

  pcl::SACSegmentation<PointT> ss_;
  pcl::ExtractPolygonalPrismData<PointT> eppd_;
  pcl::ProjectInliers<PointT> pi_;
  //pcl::RadiusOutlierRemoval<PointT> ror_;

  PlanarPolygonVisualizer polygon_visualizer_;
  ClusteredPointCloudVisualizer cluster_visualizer_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_fit");
  PlaneFitNode pfn;
  ros::spin();
  return 0;
}
