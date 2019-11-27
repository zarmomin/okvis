#ifndef OKVIS_LIDAR_FEATURE_DETECTOR_BASE_H_
#define OKVIS_LIDAR_FEATURE_DETECTOR_BASE_H_
#include <string>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <okvis/lidar/keyframe-features.h>

namespace okvis {
namespace lidar {

class FeatureDetectorBase {
public:
  FeatureDetectorBase() = default;
  virtual ~FeatureDetectorBase() = default;

  virtual void detectFeatures(const cv::Mat &image, size_t num_features,
                              const cv::Mat &mask,
                              KeyframeFeatures *keyframe) = 0;

  virtual std::string getDetectorName() const = 0;
};

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_DETECTOR_BASE_H_
