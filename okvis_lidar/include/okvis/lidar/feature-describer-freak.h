#ifndef OKVIS_LIDAR_FEATURE_DESCRIBER_FREAK_H_
#define OKVIS_LIDAR_FEATURE_DESCRIBER_FREAK_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "okvis/lidar/feature-describer-base.h"

namespace okvis {
namespace lidar {

struct FeatureDescriberFreakSettings {
  bool rotation_invariant;
  bool scale_invariant;
  float freak_pattern_scale;
  unsigned int orb_pyramid_levels;
};

FeatureDescriberFreakSettings InitFeatureDescriberFreakSettingsFromGFlags() {
  FeatureDescriberFreakSettings settings;

  // TODO(lbern): create gflags
  settings.rotation_invariant = false;
  settings.scale_invariant = true;
  settings.freak_pattern_scale = 22.0f;
  settings.orb_pyramid_levels = 1u;
  return settings;
}

class FeatureDescriberFreak : public FeatureDescriberBase {
public:
  explicit FeatureDescriberFreak(const FeatureDescriberFreakSettings &settings);
  virtual ~FeatureDescriberFreak() = default;

  virtual void describeFeatures(const cv::Mat &image,
                                KeyframeFeatures *keyframe);

  virtual bool
  hasNonDescribableFeatures(const Eigen::Matrix2Xd &keypoint_measurements,
                            const Eigen::RowVectorXd &keypoint_scales,
                            std::vector<size_t> *non_describable_indices);

  virtual std::string getDescriptorName() const { return "freak"; }

private:
  void
  updateKeyframeWithDescriptors(KeyframeFeatures *keyframe,
                                cv::Mat &descriptors_cv,
                                const std::vector<cv::KeyPoint> &keypoints_cv);

  const FeatureDescriberFreakSettings settings_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_DESCRIBER_FREAK_H_
