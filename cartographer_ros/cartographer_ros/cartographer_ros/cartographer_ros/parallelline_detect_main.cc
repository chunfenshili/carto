#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>

#include "gtest/gtest.h"
#include "cartographer/mapping/internal/parallelline_detect.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "cartographer/common/time.h"
#include "msg_conversion.h"
#include "time_conversion.h"
using namespace cartographer;
using namespace cartographer_ros;
::ros::Publisher point_cloud_publisher_;
void HandleLaserScanMessage(const sensor_msgs::LaserScan::ConstPtr& msg) {
  cartographer::sensor::PointCloudWithIntensities point_cloud;
  cartographer::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);

  sensor::PointCloud para_point_cloud;
  for (auto point : point_cloud.points) {
    para_point_cloud.push_back({point.position});
  }
  ParallelLineDetect parallellinedetect(0.9);
  auto pareller = parallellinedetect.Detect(para_point_cloud);

  ::cartographer::sensor::TimedPointCloud time_point_cloud;
  if (pareller.size() != 1) {
    return;
  }
  LOG(INFO) << "detect paraller with score: " << pareller[0].score << " "
            << pareller[0].param;
  for (const auto& point_clouds : *pareller[0].point_cloud) {
    CHECK_GE(point_clouds.size(), 1);
    for (const auto& point : point_clouds) {
      time_point_cloud.push_back({point.position, 0.0});
    }
  }
  CHECK_EQ(time_point_cloud.size(), para_point_cloud.size());
  auto msg1 = ToPointCloud2Message(ToUniversal(FromRos(ros::Time::now())),
                                   "laser", time_point_cloud);

  point_cloud_publisher_.publish(msg1);
}

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "parallerlines");
  ros::NodeHandle nh("");
  ::ros::start();
  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &HandleLaserScanMessage);
  //
  point_cloud_publisher_ =
      nh.advertise<sensor_msgs::PointCloud2>("/param_point_cloud", 10);
  ros::spin();
}