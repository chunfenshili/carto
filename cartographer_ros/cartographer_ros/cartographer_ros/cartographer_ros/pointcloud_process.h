#ifndef _POINT_CLOUDE_PROCESS_H
#define _POINT_CLOUDE_PORCESS_H

#include "cartographer/sensor/point_cloud.h"


class PointCloudProcess
{
 public:
 PointCloudProcess(float dis);
  cartographer::sensor::PointCloud OutlierFilter(
      const cartographer::sensor::PointCloud& point_cloud);

  cartographer::sensor::PointCloudWithIntensities OutlierFilter(
      const cartographer::sensor::PointCloudWithIntensities& point_cloud) ;

 private:
  cartographer::sensor::PointCloud  point_clouds_;
  float filter_dist_;
};



#endif