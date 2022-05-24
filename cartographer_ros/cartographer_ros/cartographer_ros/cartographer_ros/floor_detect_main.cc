#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_msgs/PointIndices.h>
#include <pcl_ros/point_cloud.h>
using PointType  = pcl::PointXYZI;
using PointT= pcl::PointXYZI;
constexpr int sensor_height =0.8;
constexpr double normal_filter_thresh =5;
constexpr double tilt_deg =0.4;
constexpr double height_clip_range = 3;
constexpr int floor_pts_thresh = 200;
constexpr double floor_normal_thresh =10;
pcl::PointCloud<PointT>::Ptr plane_clip(
    const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane,
    bool negative) {
  pcl::PlaneClipper3D<PointT> clipper(plane);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  clipper.clipPointCloud3D(*src_cloud, indices->indices);

  pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*dst_cloud);

  return dst_cloud;
}
pcl::PointCloud<PointT>::Ptr normal_filtering(
    const pcl::PointCloud<PointT>::Ptr& cloud)  {
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(10);
  ne.setViewPoint(0.0f, 0.0f, sensor_height);
  ne.compute(*normals);

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  filtered->reserve(cloud->size());

  for (int i = 0; i < cloud->size(); i++) {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(
        Eigen::Vector3f::UnitZ());
    if (std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
      filtered->push_back(cloud->at(i));
    }
  }

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

ros::Subscriber points_sub;
ros::Publisher floor_points_pub;
ros::Publisher floor_filtered_pub;
void CloudCallBack(sensor_msgs::PointCloud2::Ptr cloud_msg) {
 pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*cloud_msg, *cloud);
  //
  Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
  tilt_matrix.topLeftCorner(3, 3) =
      Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY())
          .toRotationMatrix();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
  filtered = plane_clip(
      filtered,
      Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range),
      false);
  filtered = plane_clip(
      filtered,
      Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range),
      true);

  filtered = normal_filtering(filtered);

  pcl::transformPointCloud(*filtered, *filtered,
                           static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

  if (floor_filtered_pub.getNumSubscribers()) {
    filtered->header = cloud->header;
    floor_filtered_pub.publish(filtered);
  }

  // too few points for RANSAC
  if (filtered->size() < floor_pts_thresh) {
  }

  // RANSAC
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<PointT>(filtered));
  pcl::RandomSampleConsensus<PointT> ransac(model_p);
  ransac.setDistanceThreshold(0.1);
  ransac.computeModel();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ransac.getInliers(inliers->indices);

  // too few inliers
  if (inliers->indices.size() < floor_pts_thresh) {
  }

  // verticality check of the detected floor's normal
  Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);

  double dot = coeffs.head<3>().dot(reference.head<3>());
  if (std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
  }

  // make the normal upward
  if (coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
    coeffs *= -1.0f;
  }

  if (floor_points_pub.getNumSubscribers()) {
    pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.filter(*inlier_cloud);
    inlier_cloud->header = cloud->header;

    floor_points_pub.publish(inlier_cloud);
  }
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "floor_detect");
  ros::NodeHandle nh;
  points_sub = nh.subscribe("/ns1/velodyne_points", 256, &CloudCallBack);
  floor_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/floor_detection/floor_filter", 32);
  floor_points_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "/floor_detection/floor_points", 32);
  ros::spin();
}
