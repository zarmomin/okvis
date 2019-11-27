#ifndef OKVIS_LIDAR_FEATURE_DETECTOR_GFTT_H_
#define OKVIS_LIDAR_FEATURE_DETECTOR_GFTT_H_

#include <string>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "okvis/lidar/feature-detector-base.h"
#include "okvis/lidar/flags.h"

namespace okvis {
namespace lidar {

class FeatureDetectorGFTT : public FeatureDetectorBase {
public:
  FeatureDetectorGFTT();
  virtual ~FeatureDetectorGFTT() {}

  virtual void detectFeatures(const cv::Mat &image, size_t num_features,
                              const cv::Mat &mask, KeyframeFeatures *keyframe);

  virtual std::string getDetectorName() const { return "gfft"; }
};

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_DETECTOR_GFTT_H_
