#ifndef _ACML_LOCATION_H
#define  _ACML_LOCATION_H
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
struct Partical
{
  float score;
  float x;
  float y;
  float theta;
 

};
struct AmclLoctionOption {
  struct AcmOption {
    int num_partical;
    float init_pose_vari_x;
    float init_pose_vari_y;
    float init_pose_vari_theta;
  };
  struct MotionPara {
    float alpa1;
    float alpa2;
    float alpa3;
    float alpa4;
    // float variant_trans;
    // float variant_theta;
  };
  AcmOption acml_para;
  MotionPara motion_para;
};

class AmclLocaton {
 public:
  AmclLocaton(const AmclLoctionOption& option,
               const     cartographer::mapping::Grid2D* map_grid);
  AmclLocaton(const AmclLoctionOption& option,
              const cartographer::transform::Rigid2d& init_pose,
              const cartographer::mapping::Grid2D* map_grid);

  std::vector<Partical> operator()(
      const cartographer::sensor::PointCloud& point_cloud,
      const cartographer::transform::Rigid2d& motion);

 private:
 
  bool ParticalInGrid(const Partical &partical);
  std::vector<Partical> SampleFromMoton(
      const cartographer::transform::Rigid2d& motion);

  void ScoreParticals(std::vector<Partical>& particals,
                      const cartographer::sensor::PointCloud& point_cloud);

  std::vector<Partical> ImportanceSampling(std::vector<Partical>& particals);
  const cartographer::mapping::Grid2D* map_grid_;
  AmclLoctionOption option_;
  std::vector<Partical> particals_;
  std::unique_ptr<cartographer::transform::Rigid2d> last_motion_;
};

#endif