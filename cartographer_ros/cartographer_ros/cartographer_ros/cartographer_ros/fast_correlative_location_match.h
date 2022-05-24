#ifndef  _FASET_CORRELATION_LOCATION_MATCH_H
#define   _FASET_CORRELATION_LOCATION_MATCH_H
#include <future>

namespace modules {

class FastCorrelativeLocationMatch  {
 public:
  FastCorrelativeLocationMatch(const PreLocationMatchOption& option,
                               std::unique_ptr<CheckLocation> check_location)
      : option_(option), check_location_(std::move(check_location)) {
    parent_ = option.check_location_option.node;
  }

  virtual std::unique_ptr<PreLocationMatchResult> Process(
      const RangeData& range_data, const Rigid3d& pose = Rigid3d{});
  ~FastCorrelativeLocationMatch() {}

 private:
  std::unique_ptr<CheckLocation> check_location_;
  PreLocationMatchOption option_;
  Node* parent_;
};
}  // namespace modules
#endif