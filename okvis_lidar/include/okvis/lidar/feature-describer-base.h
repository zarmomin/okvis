#ifndef OKVIS_LIDAR_FEATURE_DESCRIBER_BASE_H_
#define OKVIS_LIDAR_FEATURE_DESCRIBER_BASE_H_

#include <string>
#include <vector>

#include "okvis/lidar/keyframe-features.h"
#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace okvis {
namespace lidar {

class FeatureDescriberBase {
public:
  FeatureDescriberBase() = default;
  virtual ~FeatureDescriberBase() {}

  virtual void describeFeatures(const cv::Mat &image,
                                KeyframeFeatures *keyframe) = 0;

  virtual bool
  hasNonDescribableFeatures(const Eigen::Matrix2Xd &keypoint_measurements,
                            const Eigen::RowVectorXd &keypoint_scales,
                            std::vector<size_t> *non_describable_indices) {
    CHECK_NOTNULL(non_describable_indices)->clear();
    return false;
  }

  virtual std::string getDescriptorName() const = 0;
};

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_DESCRIBER_BASE_H_
