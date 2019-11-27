#ifndef OKVIS_LIDAR_FEATURE_PIPELINE_BASE_H_
#define OKVIS_LIDAR_FEATURE_PIPELINE_BASE_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/matcher/match.h>
#include <glog/logging.h>

#include "okvis/lidar/feature-describer-base.h"
#include "okvis/lidar/feature-detector-base.h"
#include "okvis/lidar/keyframe-features.h"
#include "okvis/lidar/pnp-ransac.h"

namespace okvis {
namespace lidar {

struct FeaturePipelineDebugData {
  int64_t timestamp_nframe_kp1;
  int64_t timestamp_nframe_k;

  // Inlier/outlier matches for each camera.
  std::vector<aslam::FrameToFrameMatches> inlier_matches_kp1_k;
  std::vector<aslam::FrameToFrameMatches> outlier_matches_kp1_k;
};

class FeatureTrackingPipelineBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void
  processImages(const aslam::NCamera &ncamera,
                const aslam::Quaternion &q_Icurr_Iprev,
                const std::vector<cv::Mat> &curr_camera_images,
                const std::vector<cv::Mat> &prev_camera_images,
                const std::vector<KeyframeFeatures> &previous_keyframe,
                std::vector<KeyframeFeatures> *current_keyframe_ptr,
                FeaturePipelineDebugData *optional_debug_data) {}

  virtual void
  processImages2(const aslam::Quaternion &q_Icurr_Iprev,
                 const std::vector<cv::Mat> &curr_camera_images,
                 const std::vector<cv::Mat> &prev_camera_images,
                 const std::vector<KeyframeFeatures> &previous_keyframe,
                 std::vector<KeyframeFeatures> *current_keyframe_ptr,
                 pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                 FeaturePipelineDebugData *optional_debug_data) {}

  virtual ~FeatureTrackingPipelineBase() {}

protected:
  FeatureTrackingPipelineBase(
      const RansacSettings &ransac_settings,
      std::shared_ptr<FeatureDetectorBase> feature_detector,
      std::shared_ptr<FeatureDescriberBase> feature_describer)
      : ransac_settings_(ransac_settings), feature_detector_(feature_detector),
        feature_describer_(feature_describer) {
    CHECK_NOTNULL(feature_detector);
    CHECK_NOTNULL(feature_describer);
  }

protected:
  const RansacSettings ransac_settings_;

  const std::shared_ptr<FeatureDetectorBase> feature_detector_;
  const std::shared_ptr<FeatureDescriberBase> feature_describer_;
};

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_PIPELINE_BASE_H_
