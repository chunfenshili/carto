#include "acml_location.h"
#include "Eigen/Geometry"
#include <algorithm>
#include <random>
#include <mutex>
namespace {
using namespace cartographer::mapping;
using namespace cartographer;


std::vector<sensor::PointCloud> GenerateParticsScan(
    const std::vector<Partical>& particals,
    const sensor::PointCloud& point_cloud) {
  std::vector<sensor::PointCloud> partical_scans;
  for (const auto particl : particals) {
    transform::Rigid3f transfor(
        {particl.x, particl.y, 0},
        Eigen::AngleAxisf(particl.theta, Eigen::Vector3f::UnitZ()));
    partical_scans.push_back(
        sensor::TransformPointCloud(point_cloud, transfor));
  }
  return partical_scans;
}

Partical  AveragePartical(const std::vector<Partical> &particals);

template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * floor((angle_radians + T(M_PI)) / two_pi);
}
std::ostream& operator<<(std::ostream& out, const Partical& partic) {
  out << "partic : [" << partic.x << " "
      << " " << partic.y << " " << partic.theta
      << " ] with score : " << partic.score << std::endl;
  return out;
}
}

Partical SamplingInGridMap(const Grid2D& grid) {
  Eigen::Array2i offset;
  CellLimits cell_limits;
  grid.ComputeCroppedLimits(&offset, &cell_limits);
  static std::mt19937 prng(42);
  Eigen::Array2i cell_index;
  do {
    std::uniform_int_distribution<int> random(
        0, grid.limits().cell_limits().num_x_cells *
               grid.limits().cell_limits().num_y_cells);
    int index = random(prng);
    cell_index =
        Eigen::Array2i{index % grid.limits().cell_limits().num_x_cells,
                       index / grid.limits().cell_limits().num_x_cells};
    cell_index += offset;
  } while (!grid.IsKnown(cell_index));
  Eigen::Vector2f partic_positon = grid.limits().GetCellCenter(cell_index);
  std::uniform_real_distribution<> random_theta(-3.14, 3.14);
  return Partical{0.1, partic_positon.x(), partic_positon.y(),
                  static_cast<float>(random_theta(prng))};

}
bool AmclLocaton::ParticalInGrid(const Partical& partical) {
  Eigen::Vector2f point(partical.x, partical.y);
  auto cell_index = map_grid_->limits().GetCellIndex(point);

  return map_grid_->IsKnown(cell_index) ||
         map_grid_->GetCorrespondenceCost(
             map_grid_->limits().GetCellIndex(point)) < 0.5;
}
AmclLocaton::AmclLocaton(
    const AmclLoctionOption& option,
    const cartographer::mapping::Grid2D* map_grid)
    : map_grid_((map_grid)), option_(option) {
  static std::mt19937 prng(42);
  Eigen::Array2i offset;
  CellLimits cell_limits;
  map_grid->ComputeCroppedLimits(&offset, &cell_limits);
  for (int i = 0; i < option_.acml_para.num_partical;) {
    std::uniform_int_distribution<int> random(
        0, map_grid_->limits().cell_limits().num_x_cells *
               map_grid_->limits().cell_limits().num_y_cells);
    int index = random(prng);
    Eigen::Array2i cell_index{
        index % map_grid_->limits().cell_limits().num_x_cells,
        index / map_grid_->limits().cell_limits().num_x_cells};
    cell_index += offset;
    //
    if (!map_grid_->IsKnown(cell_index)) continue;
    i++;
    Eigen::Vector2f partic_positon =
        map_grid_->limits().GetCellCenter(cell_index);
    std::uniform_real_distribution<> random_theta(-3.14, 3.14);
    particals_.push_back(Partical{0.2, partic_positon.x(), partic_positon.y(),
                            static_cast<float>(random_theta(prng))});
  }
}

Partical GeneratePartics(const AmclLoctionOption& option,
                         const cartographer::mapping::Grid2D& grid,
                         std::function<float()> random) {
  return {};
}

AmclLocaton::AmclLocaton(const AmclLoctionOption& option,
                         const cartographer::transform::Rigid2d& init_pose,
                         const cartographer::mapping::Grid2D* map_grid)
    : map_grid_((map_grid)), option_(option) {
  static std::mt19937 prng(49);
  Eigen::Array2i offset;
  CellLimits cell_limits;
  std::cout << "start with init pose------------" << std::endl;
  map_grid->ComputeCroppedLimits(&offset, &cell_limits);
  for (int i = 0; i < option_.acml_para.num_partical;) {
    std::normal_distribution<float> normal_x(
        init_pose.translation().x(), option_.acml_para.init_pose_vari_x);
    std::normal_distribution<float> normal_y(
        init_pose.translation().y(), option_.acml_para.init_pose_vari_y);
    auto cell_index =
        map_grid_->limits().GetCellIndex({normal_x(prng), normal_y(prng)});
    if (!map_grid_->IsKnown(cell_index)) continue;
    Eigen::Vector2f partic_positon =
        map_grid_->limits().GetCellCenter(cell_index);

    std::normal_distribution<float> random_theta(
        init_pose.rotation().angle(), option_.acml_para.init_pose_vari_theta);
    particals_.push_back(Partical{0.2, partic_positon.x(), partic_positon.y(),
                                  static_cast<float>(random_theta(prng))});
    std::cout << particals_[i];
    i++;
  }
}

std::tuple<float, float, float> GetDeltaFromTwoMotion(
    const AmclLoctionOption::MotionPara& option,
    const cartographer::transform::Rigid2d& last_motion,
    const cartographer::transform::Rigid2d& motion) {
  // transform::Rigid2d delta_trans = motion* last_motion.inverse();
  Eigen::Vector2d t = motion.translation() - last_motion.translation();
  float rot1;
  if (fabs(t.x()) < 1e-6   ||  t.norm()< 1e-6) {
    rot1 = 0;
  } else {
    rot1 =(atan2(t.y(), t.x()) - last_motion.rotation().angle());
  }
  float trans = t.norm();
  float rot2 =(motion.rotation().angle() - atan2(t.y(), t.x()));
  //
  // static std::mt19937 prng(42);
  static std::default_random_engine prng;
  float variat = option.alpa1 * std::pow(rot1, 2) +
                 option.alpa2 * std::pow(trans, 2);
  std::normal_distribution<float> normal_distribution(0, variat);
  rot1-= normal_distribution(prng);

  //

  variat = option.alpa3 * std::pow(trans, 2) +
           option.alpa4 * std::pow(rot1, 2) +
           option.alpa4 * std::pow(rot2, 2);
  normal_distribution = std::normal_distribution<float>(0, variat);
  trans-= normal_distribution(prng);
  //
  variat = option.alpa1 * std::pow(rot2, 2) +
           option.alpa2 * std::pow(trans, 2);
  normal_distribution = std::normal_distribution<float>(0, variat);

  rot2-= normal_distribution(prng);


  return {rot1, rot2,trans};
}

std::vector<Partical> AmclLocaton::SampleFromMoton(
    const cartographer::transform::Rigid2d& motion) {
 std::vector<Partical> result;
 for (const auto& partic : particals_) {
   auto [rot1, rot2,trans] =
       GetDeltaFromTwoMotion(option_.motion_para, *last_motion_, motion);
   result.push_back(Partical{
       partic.score,
       partic.x +trans* float(cos(partic.theta +rot1)),
       partic.y + trans* float(sin(partic.theta +rot1)),
       NormalizeAngle( partic.theta + (rot1+rot2))});
 }
 return result;
}


void AmclLocaton::ScoreParticals(
    std::vector<Partical>& particals,
    const cartographer::sensor::PointCloud& point_cloud) {
  auto partical_scans = GenerateParticsScan(particals, point_cloud);
  std::vector<Eigen::Array2i> neighbor_cell{{0, 0},   {1, 0},  {1, 1},
                                            {0, 1},   {-1, 1}, {-1, 0},
                                            {-1, -1}, {0, -1}, {1, -1}};
  auto& grid = *map_grid_;
  for (int i = 0; i < particals.size(); i++) {
    const auto& point_cloud = partical_scans[i];
    float sum = 0.0;
    for (const auto& point : point_cloud) {
      auto cell_index =
          grid.limits().GetCellIndex(point.position.head<2>());
      float score = 0.0;
      // for (auto ncell : neighbor_cell) {
      for (auto i : {-2, -1, 0, 1, 2}) {
        for (auto j : {-2, -1, 0, 1, 2}) {
          if (grid.IsKnown(cell_index + Eigen::Array2i{i, j})) {
            score =
                std::max(score, 1.f - std::abs(grid.GetCorrespondenceCost(
                                          cell_index + Eigen::Array2i{i, j})));
          }
        }
      }
      // double dist = 100;
      // for (int i = 0;i<10;i++) {
      //   for (int j = 0;j<10; j++) {
      //     if (grid.IsKnown(cell_index + Eigen::Array2i{i, j})) {
      //       if (grid.GetCorrespondenceCost(cell_index + Eigen::Array2i{i, j}) <
      //           0.5) {
      //           dist = std::min(dist,sqrt(i*i+j*j));
      //       }
      //     }
      //   }
      // }
      // sum += fabs((1-exp(1-dist/100)));
       sum += score;
    }
    // particals[i].score  *= sum / partical_scans[i].size();
     particals[i].score  = sum / partical_scans[i].size();
    // if(particals[i].score<0.05){
    //   particals[i] =  SamplingInGridMap(*map_grid_);
    // }
  }
  static std::once_flag oc;
  std::call_once(oc, [=] {
    for (auto part : particals) {
      std::cout << part;
    }
  });
}
std::vector<Partical> AmclLocaton::ImportanceSampling(
    std::vector<Partical>& particals) {
 
  float sum_partical_score = 
  std::accumulate(particals.begin(), particals.end(),0.0,
                  [](const float& init, const Partical& partical) -> float {
                    return init + partical.score;
                  });

  std::transform(particals.begin(), particals.end(), particals.begin(),
                 [=](const Partical& partical) -> Partical {
                   return {partical.score / sum_partical_score, partical.x,
                           partical.y, partical.theta};
                 });
  std::vector<Partical> partics_import;
  for (int i = 0; i < particals.size(); i++) {
    for (int j = 0; j < particals[i].score * particals.size(); j++) {
      partics_import.push_back(particals[i]);
    }
  }
  if (partics_import.empty()) {
    return particals;
  }

  std::vector<Partical> result;
 static  std::default_random_engine dre;
  std::uniform_int_distribution<int> di(0, partics_import.size()-1);
  std::random_shuffle(partics_import.begin(), partics_import.end());
  for (int i = 0; i < particals.size(); i++) {
    result.push_back(partics_import[di(dre)]);
  }
  return result;
}

void  NormalizerPartical(std::vector<Partical>& particals)
{


}

std::vector<Partical> AmclLocaton::operator()(
    const cartographer::sensor::PointCloud& point_cloud,
    const cartographer::transform::Rigid2d& motion) {
  if(point_cloud.empty())return{};
  if (last_motion_ == nullptr) {
    last_motion_ = std::make_unique<transform::Rigid2d>(motion);
    return {};
  }
  if ((last_motion_->inverse() * motion).translation().norm() >= 1) {
    *last_motion_ = motion;
    return {};
  }
  std::vector<Partical>  particals = SampleFromMoton(motion);
  ScoreParticals(particals,point_cloud);
   particals_ = ImportanceSampling(particals);
  // // particals_ = particals;
  //  std::random_shuffle(particals.begin(),particals.end());

  // std::vector<Partical> half_particls1(
  //     particals.begin(), particals.begin() + particals.size() / 2);
  // std::vector<Partical> half_particls2(particals.begin() + particals.size() / 2,
  //                                      particals.end());
  // float half_score_sum = 0.0;
  // for (int i = 0; i < half_particls1.size(); i++) {
  //   float score = particals_[i].score;
  //   particals_[i] = half_particls1[i];
  //   particals_[i].score *= score;
  //   half_score_sum += particals_[i].score;
  // }
  // for (int i = 0; i < half_particls1.size(); i++) {
  //   particals_[i].score /= half_score_sum;
  // }
  // half_particls2 = ImportanceSampling(half_particls2);
  // half_score_sum = 0.0;

  // for (int i = 0; i < half_particls2.size(); i++) {
  //   half_score_sum += half_particls2[i].score;
  // }
  // for (int i = half_particls1.size(); i < particals_.size(); i++) {
  //   particals_[i] = half_particls2[i - half_particls1.size()];
  //   particals_[i].score =
  //       half_particls2[i - half_particls1.size()].score / half_score_sum;
  // }
  for (auto& partical : particals_) {
    if (!ParticalInGrid(partical)) {
      partical = SamplingInGridMap(*map_grid_);
    }
  }
  *last_motion_  = motion;
  return particals_;

}
