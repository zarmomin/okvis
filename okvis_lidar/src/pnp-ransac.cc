#include "okvis/lidar/pnp-ransac.h"

namespace okvis {
namespace lidar {

RansacSettings InitRansacSettingsFromGFlags() {
  RansacSettings settings;
  settings.ransac_threshold = FLAGS_NEW_feature_tracker_ransac_threshold;
  settings.ransac_max_iterations = FLAGS_NEW_feature_tracker_ransac_max_iters;
  settings.fix_random_seed = FLAGS_NEW_feature_tracker_ransac_fix_seed;
  return settings;
}

bool PerformTemporalFrameToFrameRansac(
    const aslam::Camera &camera, const KeyframeFeatures &keyframe_features_kp1,
    const KeyframeFeatures &keyframe_features_k, const RansacSettings &settings,
    const aslam::Quaternion &q_Ckp1_Ck,
    aslam::FrameToFrameMatches *inlier_matches_kp1_k,
    aslam::FrameToFrameMatches *outlier_matches_kp1_k) {
  CHECK_GT(settings.ransac_threshold, 0.0);
  CHECK_GT(settings.ransac_max_iterations, 0u);

  inlier_matches_kp1_k->clear();
  outlier_matches_kp1_k->clear();

  // Extract correspondences and convert to bearing vectors.
  aslam::FrameToFrameMatches matches_kp1_k;
  aslam::extractMatchesFromTrackIdChannel(
      keyframe_features_kp1.keypoint_track_ids.transpose(),
      keyframe_features_k.keypoint_track_ids.transpose(), &matches_kp1_k);

  aslam::geometric_vision::BearingVectors bearing_vectors_kp1;
  aslam::geometric_vision::BearingVectors bearing_vectors_k;
  bearing_vectors_kp1.reserve(matches_kp1_k.size());
  bearing_vectors_k.reserve(matches_kp1_k.size());

  for (const aslam::FrameToFrameMatch &match : matches_kp1_k) {
    const int keypoint_idx_kp1 = match.getKeypointIndexAppleFrame();
    const int keypoint_idx_k = match.getKeypointIndexBananaFrame();
    CHECK_LT(keypoint_idx_kp1,
             keyframe_features_kp1.keypoint_measurements.cols());
    CHECK_LT(keypoint_idx_k, keyframe_features_k.keypoint_measurements.cols());

    bool projection_successul = true;
    Eigen::Vector3d bearing_vector_kp1, bearing_vector_k;
    projection_successul &= camera.backProject3(
        keyframe_features_kp1.keypoint_measurements.col(keypoint_idx_kp1),
        &bearing_vector_kp1);
    projection_successul &= camera.backProject3(
        keyframe_features_k.keypoint_measurements.col(keypoint_idx_k),
        &bearing_vector_k);

    if (projection_successul) {
      bearing_vectors_kp1.emplace_back(bearing_vector_kp1.normalized());
      bearing_vectors_k.emplace_back(bearing_vector_k.normalized());
    }
  }

  // Run 2-pt RANSAC and prepare outputs.
  std::unordered_set<int> inlier_indices;
  bool ransac_success = aslam::geometric_vision::
      rejectOutlierFeatureMatchesTranslationRotationSAC(
          bearing_vectors_kp1, bearing_vectors_k, q_Ckp1_Ck,
          settings.fix_random_seed, settings.ransac_threshold,
          settings.ransac_max_iterations, &inlier_indices);

  int match_index = 0;
  for (const aslam::FrameToFrameMatch &match : matches_kp1_k) {
    if (inlier_indices.count(match_index)) {
      inlier_matches_kp1_k->emplace_back(match);
    } else {
      outlier_matches_kp1_k->emplace_back(match);
    }
    ++match_index;
  }
  CHECK_EQ(inlier_matches_kp1_k->size() + outlier_matches_kp1_k->size(),
           matches_kp1_k.size());
  return ransac_success;
}

static std::vector<int> getPxOffset(int lidar_mode) {
  auto repeat = [](int n, const std::vector<int> &v) {
    std::vector<int> res{};
    for (int i = 0; i < n; i++)
      res.insert(res.end(), v.begin(), v.end());
    return res;
  };

  switch (lidar_mode) {
  case 512:
    return repeat(16, {0, 3, 6, 9});
  case 1024:
    return repeat(16, {0, 6, 12, 18});
  case 2048:
    return repeat(16, {0, 12, 24, 36});
  default:
    return std::vector<int>{64, 0};
  }
}

static bool backProject3d(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                          const Eigen::Ref<const Eigen::Vector2d> &keypoint,
                          Eigen::Vector3d *out_point_3d) {
  CHECK_NOTNULL(out_point_3d);
  const auto W = 1024;
  const auto H = 64;
  const int v = keypoint[0];
  const int u = keypoint[1];
  const auto px_offset = getPxOffset(W);
  const std::size_t offset = px_offset[u];
  const std::size_t index = ((v + offset) % W) * H + u;
  pcl::PointXYZI projected = cloud->points[index];
  (*out_point_3d)[0] = projected.x;
  (*out_point_3d)[1] = projected.y;
  (*out_point_3d)[2] = projected.z;
}

bool PerformTemporalFrameToFrameRansac(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const KeyframeFeatures &keyframe_features_kp1,
    const KeyframeFeatures &keyframe_features_k, const RansacSettings &settings,
    const aslam::Quaternion &q_Ckp1_Ck,
    aslam::FrameToFrameMatches *inlier_matches_kp1_k,
    aslam::FrameToFrameMatches *outlier_matches_kp1_k) {
  CHECK_GT(settings.ransac_threshold, 0.0);
  CHECK_GT(settings.ransac_max_iterations, 0u);

  inlier_matches_kp1_k->clear();
  outlier_matches_kp1_k->clear();

  // Extract correspondences and convert to bearing vectors.
  aslam::FrameToFrameMatches matches_kp1_k;
  aslam::extractMatchesFromTrackIdChannel(
      keyframe_features_kp1.keypoint_track_ids.transpose(),
      keyframe_features_k.keypoint_track_ids.transpose(), &matches_kp1_k);

  aslam::geometric_vision::BearingVectors bearing_vectors_kp1;
  aslam::geometric_vision::BearingVectors bearing_vectors_k;
  bearing_vectors_kp1.reserve(matches_kp1_k.size());
  bearing_vectors_k.reserve(matches_kp1_k.size());

  for (const aslam::FrameToFrameMatch &match : matches_kp1_k) {
    const int keypoint_idx_kp1 = match.getKeypointIndexAppleFrame();
    const int keypoint_idx_k = match.getKeypointIndexBananaFrame();
    CHECK_LT(keypoint_idx_kp1,
             keyframe_features_kp1.keypoint_measurements.cols());
    CHECK_LT(keypoint_idx_k, keyframe_features_k.keypoint_measurements.cols());

    /*
    bool projection_successul = true;
    Eigen::Vector3d bearing_vector_kp1, bearing_vector_k;
    projection_successul &= camera.backProject3(
        keyframe_features_kp1.keypoint_measurements.col(keypoint_idx_kp1),
        &bearing_vector_kp1);
    projection_successul &= camera.backProject3(
        keyframe_features_k.keypoint_measurements.col(keypoint_idx_k),
        &bearing_vector_k);
    */
    Eigen::Vector3d bearing_vector_kp1, bearing_vector_k;
    bool projection_successul = true;
    backProject3d(
        cloud,
        keyframe_features_kp1.keypoint_measurements.col(keypoint_idx_kp1),
        &bearing_vector_kp1);
    backProject3d(cloud,
                  keyframe_features_k.keypoint_measurements.col(keypoint_idx_k),
                  &bearing_vector_k);

    if (projection_successul) {
      bearing_vectors_kp1.emplace_back(bearing_vector_kp1.normalized());
      bearing_vectors_k.emplace_back(bearing_vector_k.normalized());
    }
  }

  // Run 2-pt RANSAC and prepare outputs.
  std::unordered_set<int> inlier_indices;
  bool ransac_success = aslam::geometric_vision::
      rejectOutlierFeatureMatchesTranslationRotationSAC(
          bearing_vectors_kp1, bearing_vectors_k, q_Ckp1_Ck,
          settings.fix_random_seed, settings.ransac_threshold,
          settings.ransac_max_iterations, &inlier_indices);

  int match_index = 0;
  for (const aslam::FrameToFrameMatch &match : matches_kp1_k) {
    if (inlier_indices.count(match_index)) {
      inlier_matches_kp1_k->emplace_back(match);
    } else {
      outlier_matches_kp1_k->emplace_back(match);
    }
    ++match_index;
  }
  CHECK_EQ(inlier_matches_kp1_k->size() + outlier_matches_kp1_k->size(),
           matches_kp1_k.size());
  return ransac_success;
}

} // namespace lidar
} // namespace okvis
