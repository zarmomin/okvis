#include "okvis/lidar/feature-describer-brisk.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "okvis/lidar/helpers.h"
#include "okvis/lidar/keyframe-features.h"

namespace okvis {
namespace lidar {

void FeatureDescriberBrisk::describeFeatures(const cv::Mat &image,
                                             KeyframeFeatures *keyframe) {
  CHECK_NOTNULL(keyframe);

  LOG(WARNING) << "IMPL";
  const std::size_t n_keypoints = keyframe->keypoint_measurements.cols();
  keyframe->keypoint_descriptors.resize(2, n_keypoints);
  keyframe->keypoint_descriptors.setZero();
}

bool FeatureDescriberBrisk::hasNonDescribableFeatures(
    const Eigen::Matrix2Xd &keypoint_measurements,
    const Eigen::RowVectorXd &keypoint_scales,
    std::vector<size_t> *non_describable_indices) {
  LOG(WARNING) << "IMPL";
  return false;
}

} // namespace lidar
} // namespace okvis
