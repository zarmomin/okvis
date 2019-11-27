#ifndef OKVIS_LIDAR_PNP_RANSAC_H_
#define OKVIS_LIDAR_PNP_RANSAC_H_

#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/pose-types.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/geometric-vision/match-outlier-rejection-twopt.h>
#include <aslam/matcher/match-helpers.h>
#include <maplab-common/conversions.h>
#include <sensors/lidar.h>

#include "okvis/lidar/flags.h"
#include "okvis/lidar/keyframe-features.h"

namespace okvis {
namespace lidar {

struct RansacSettings {
  double ransac_threshold = 1.0 - cos(0.5 * kDegToRad);
  size_t ransac_max_iterations = 200;
  bool fix_random_seed = false;
};

RansacSettings InitRansacSettingsFromGFlags();

bool PerformTemporalFrameToFrameRansac(
    const aslam::Camera &camera, const KeyframeFeatures &keyframe_features_kp1,
    const KeyframeFeatures &keyframe_features_k, const RansacSettings &settings,
    const aslam::Quaternion &q_Ckp1_Ck,
    aslam::FrameToFrameMatches *inlier_matches_kp1_k,
    aslam::FrameToFrameMatches *outlier_matches_kp1_k);

bool PerformTemporalFrameToFrameRansac(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const KeyframeFeatures &keyframe_features_kp1,
    const KeyframeFeatures &keyframe_features_k, const RansacSettings &settings,
    const aslam::Quaternion &q_Ckp1_Ck,
    aslam::FrameToFrameMatches *inlier_matches_kp1_k,
    aslam::FrameToFrameMatches *outlier_matches_kp1_k);

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_PNP_RANSAC_H_
