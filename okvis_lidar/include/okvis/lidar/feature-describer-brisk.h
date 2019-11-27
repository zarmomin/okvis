#ifndef OKVIS_LIDAR_FEATURE_DESCRIBER_BRISK_H_
#define OKVIS_LIDAR_FEATURE_DESCRIBER_BRISK_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "okvis/lidar/feature-describer-base.h"

namespace okvis {
namespace lidar {

struct FeatureDescriberBriskSettings {};

FeatureDescriberBriskSettings InitFeatureDescriberBriskSettingsFromGFlags() {
  FeatureDescriberBriskSettings settings;

  return settings;
}

class FeatureDescriberBrisk : public FeatureDescriberBase {
public:
  explicit FeatureDescriberBrisk(const FeatureDescriberBriskSettings &settings)
      : settings_(settings) {}
  virtual ~FeatureDescriberBrisk() {}

  virtual void describeFeatures(const cv::Mat &image,
                                KeyframeFeatures *keyframe);

  virtual bool
  hasNonDescribableFeatures(const Eigen::Matrix2Xd &keypoint_measurements,
                            const Eigen::RowVectorXd &keypoint_scales,
                            std::vector<size_t> *non_describable_indices);

  virtual std::string getDescriptorName() const { return "brisk"; }

private:
  const FeatureDescriberBriskSettings settings_;
};

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_DESCRIBER_BRISK_H_
