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
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/moment.hpp>

#include "aliases.h"
#include <planar_polygon_visualizer.h>
#include <clustered_point_cloud_visualizer.h>
#include <pmd_tools/PlaneFitResult.h>

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
    ss_.setDistanceThreshold(0.004);
    eppd_.setHeightLimits(-1.00, 1.00);
    ror_.setRadiusSearch(0.01);
    ror_.setMinNeighborsInRadius(16);
    pi_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    ch_.setDimension(2);
    points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/points", 1, boost::bind(&PlaneFitNode::pointsCallback, this, _1));
    fit_publisher_ = nh_.advertise<pmd_tools::PlaneFitResult>("fit", 10);
  }

  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    PointCloud::Ptr input_cloud(new PointCloud);
    PointCloud::Ptr filtered_plane_inliers_cloud(new PointCloud);
    PointCloud::Ptr plane_cloud(new PointCloud);
    PointCloud::Ptr plane_hull(new PointCloud);
    pcl::ModelCoefficients::Ptr plane_model_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_model_inliers_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr within_boundary_indices(new pcl::PointIndices);

    // Step 0: convert ROS message into PCL object
    pcl::fromROSMsg(*msg, *input_cloud);

    // Step 1: fit a plane
    ss_.setInputCloud(input_cloud);
    ss_.segment(*plane_model_inliers_indices, *plane_model_coefficients);
    if (plane_model_inliers_indices->indices.size() == 0)
      return;

    // Step 2: remove noisy points on the boundary with RadiusOutlierRemoval
    ror_.setInputCloud(input_cloud);
    ror_.setIndices(plane_model_inliers_indices);
    ror_.filter(*filtered_plane_inliers_cloud);

    // Step 3: project inliers on the plane and compute convex hull
    pi_.setInputCloud(filtered_plane_inliers_cloud);
    pi_.setModelCoefficients(plane_model_coefficients);
    pi_.filter(*plane_cloud);
    ch_.setInputCloud(plane_cloud);
    ch_.reconstruct(*plane_hull);

    // Step 4: determine which points fall within the boundary (whether they
    // belong to the plane or not does not matter)
    eppd_.setInputCloud(input_cloud);
    eppd_.setInputPlanarHull(plane_hull);
    eppd_.segment(*within_boundary_indices);

    // Step 5: calculate statistics
    // Construct planar polygon
    Eigen::Vector4f cf;
    memcpy(&cf[0], &plane_model_coefficients->values[0], 4 * sizeof(float));
    PlanarPolygon planar_polygon(plane_hull->points, cf);
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::mean, tag::moment<2>>> d;
    pcl::PointIndices inner_outliers_indices;
    pcl::PointIndices inner_inliers_indices;
    for (const auto& i : within_boundary_indices->indices)
    {
      double distance = pointToPlaneDistanceSigned(input_cloud->points[i], cf);
      d(distance);
      if (std::abs(distance) > 0.01)
        inner_outliers_indices.indices.push_back(i);
      else
        inner_inliers_indices.indices.push_back(i);
    }
    double std = sqrt(moment<2>(d));
    double area = computePlanarPolygonArea(planar_polygon);
    ROS_INFO("%.5f, %.7f, %.3f", mean(d), std, area);

    pmd_tools::PlaneFitResult fit;
    fit.distance = cf[3];
    fit.std = std;
    fit.area = area;
    fit_publisher_.publish(fit);

    // Visualization
    std::vector<PointCloud::Ptr> clusters;
    PointCloud::Ptr cluster1(new PointCloud);
    PointCloud::Ptr cluster2(new PointCloud);
    pcl::copyPointCloud(*input_cloud, inner_inliers_indices, *cluster1);
    clusters.push_back(cluster1);
    pcl::copyPointCloud(*input_cloud, inner_outliers_indices, *cluster2);
    clusters.push_back(cluster2);
    polygon_visualizer_.publish(planar_polygon);
    cluster_visualizer_.publish<PointT>(clusters);
  }

private:

  PlanarPolygonPtr computeBoundary(const PointCloud::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_indices, const pcl::ModelCoefficients::Ptr& coefficients)
  {
    // Remove noisy points on the boundary with RadiusOutlierRemoval
    PointCloud::Ptr filtered_inliers_cloud(new PointCloud);
    ror_.setInputCloud(cloud);
    ror_.setIndices(inliers_indices);
    ror_.filter(*filtered_inliers_cloud);
    // Project inliers on the plane
    PointCloud::Ptr plane_cloud(new PointCloud);
    pi_.setInputCloud(filtered_inliers_cloud);
    pi_.setModelCoefficients(coefficients);
    pi_.filter(*plane_cloud);
    // Compute convex hull around inliers
    pcl::ConvexHull<PointT> convex_hull;
    PointCloud::Ptr plane_hull(new PointCloud);
    convex_hull.setDimension(2);
    convex_hull.setInputCloud(plane_cloud);
    convex_hull.reconstruct(*plane_hull);
    // Construct planar polygon
    Eigen::Vector4f cf;
    memcpy(&cf[0], &coefficients->values[0], coefficients->values.size() * sizeof(coefficients->values[0]));
    return boost::make_shared<PlanarPolygon>(plane_hull->points, cf);
  }

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
  ros::Publisher fit_publisher_;

  pcl::SACSegmentation<PointT> ss_;
  pcl::ExtractPolygonalPrismData<PointT> eppd_;
  pcl::ProjectInliers<PointT> pi_;
  pcl::RadiusOutlierRemoval<PointT> ror_;
  pcl::ConvexHull<PointT> ch_;

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
