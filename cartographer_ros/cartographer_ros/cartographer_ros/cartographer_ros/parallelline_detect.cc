#include "parallelline_detect.h"
#include "cartographer/sensor/point_cloud.h"
#include "tuple"
std::ostream& operator<<(std::ostream& out, const ParallelParam& param) {
  out << " param a:" << param.a << " b:" << param.b << " c1:" << param.c1
      << " c2:" << param.c2 << "\n";
  return out;
};
namespace {
using namespace cartographer;

Eigen::Vector2f ComputeCentroid(const sensor::PointCloud& point_cloud) {
  Eigen::Vector2f sum = Eigen::Vector2f::Zero();
  for (const sensor::RangefinderPoint& point : point_cloud) {
    sum += point.position.head<2>();
  }
  return sum / static_cast<float>(point_cloud.size());
}
const Eigen::MatrixXd MinusCentroid(const sensor::PointCloud& point_cloud,
                                    const Eigen::Vector2f& centroid) {
  Eigen::MatrixXd average_point_cloud = Eigen::MatrixXd(point_cloud.size(), 2);
  for (int i = 0; i < point_cloud.size(); i++) {
    average_point_cloud.row(i) =
        Eigen::Vector2d{point_cloud[i].position.x() - centroid.x(),
                        point_cloud[i].position.y() - centroid.y()};
  }
  return average_point_cloud;
}
}  // namespace

// https://blog.csdn.net/liyuanbhu/article/details/97612800
ParallelParam ParallelLIneFillting::Fit() {
  const sensor::PointCloud& x1 = sengment_[0];
  const sensor::PointCloud& x2 = sengment_[1];
  auto x1_mean = ComputeCentroid(x1);
  auto x2_mean = ComputeCentroid(x2);
  Eigen::MatrixXd x1_vector = MinusCentroid(x1, x1_mean);
  Eigen::MatrixXd x2_vector = MinusCentroid(x2, x2_mean);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(x1_vector.rows() + x2_vector.rows());
  Eigen::VectorXd y = Eigen::VectorXd::Zero(x1_vector.rows() + x2_vector.rows());
  //
  x.head(x1_vector.rows()) = x1_vector.block(0, 0, x1_vector.rows(), 1);
  y.head(x1_vector.rows()) = x1_vector.block(0, 1, x1_vector.rows(), 1);
  //
  x.tail(x2_vector.rows()) = x2_vector.block(0, 0, x2_vector.rows(), 1);
  y.tail(x2_vector.rows()) = x2_vector.block(0, 1, x2_vector.rows(), 1);
  double xx = x.dot(x);
  double xy = x.dot(y);
  double yy = y.dot(y);

  Eigen::Matrix2d d;
  d << xx, xy, xy, yy;
  Eigen::EigenSolver<Eigen::Matrix2d> es(d);
  Eigen::Vector2d singular = es.eigenvalues().real();
  auto eigenvector = es.eigenvectors().real();
  Eigen::Vector2d result = eigenvector.col(1);
  if (singular[0] < singular[1]) {
    result = eigenvector.col(0);
  }
  double a = result[0];
  double b = result[1];
  double c1 = -a * x1_mean[0] - b * x1_mean[1];
  double c2 = -a * x2_mean[0] - b * x2_mean[1];
  return {a, b, c1, c2};
}
//
double ParallelLIneFillting::Score(const ParallelParam& paraller) {
  double inlier_num = 0;
  for (auto point : sengment_[0]) {
    double dis = paraller.a * point.position.x() +
                 paraller.b * point.position.y() + paraller.c1;
    if (abs(dis) < 0.1) {
      inlier_num++;
    }
  }
  for (auto point : sengment_[1]) {
    double dis = paraller.a * point.position.x() +
                 paraller.b * point.position.y() + paraller.c2;
    if (abs(dis) < 0.1) {
      inlier_num++;
    }
  }
  return inlier_num / (sengment_[0].size() + sengment_[1].size());
}
//
std::vector<ParallelLineResult> ParallelLineDetect::Detect(
    const sensor::PointCloud& point_cloud) {
  auto senment_point_cloud = classify_->Sengment(point_cloud);
  if (senment_point_cloud.empty()) {
    return {};
  }
  using ScorePointType = ParallelLineResult;
  std::vector<ScorePointType> score_point_cloud;
  for (const auto& sengment : senment_point_cloud) {
    paraller_fit_ = std::make_unique<ParallelLIneFillting>(*sengment);
    ParallelParam param = paraller_fit_->Fit();
    double score = paraller_fit_->Score(param);
    if (score > paraller_score) {
      ScorePointType score_point{score, sengment, param};
      score_point_cloud.push_back(score_point);
    }
  }
  if (score_point_cloud.empty()) return {};
  LOG(INFO)<<score_point_cloud.begin()->score;
  std::sort(score_point_cloud.begin(), score_point_cloud.end(),
            [](const ScorePointType& lhs, const ScorePointType& rhs) {
              return lhs.score > rhs.score;
            });
  return {score_point_cloud[0]};
}

double ParallelLineDetect::Score(
    const std::vector<cartographer::sensor::PointCloud>& point_clouds) {
  return 0.0;
}
