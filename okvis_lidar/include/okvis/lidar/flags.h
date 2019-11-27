#ifndef OKVIS_LIDAR_FLAGS_H_
#define OKVIS_LIDAR_FLAGS_H_

#include <gflags/gflags.h>

namespace okvis {
namespace lidar {
// General detector settings.
DECLARE_int32(NEW_feature_tracker_num_features);
DECLARE_int32(NEW_feature_tracker_num_pyramid_levels);
DECLARE_double(NEW_feature_tracker_pyramid_scale_decimation);

// RANSAC.
DECLARE_double(NEW_feature_tracker_ransac_threshold);
DECLARE_int32(NEW_feature_tracker_ransac_max_iters);
DECLARE_bool(NEW_feature_tracker_ransac_fix_seed);

// ORB detector specific settings.
DECLARE_int32(NEW_feature_tracker_detector_orb_edge_threshold);
DECLARE_int32(NEW_feature_tracker_detector_orb_fast_threshold);

// BRISK.
DECLARE_double(NEW_feature_tracker_brisk_uniformity_radius);
DECLARE_double(NEW_feature_tracker_detector_brisk_absolute_threshold);

// SIFT
DECLARE_int32(NEW_feature_tracker_sift_n_features);
DECLARE_int32(NEW_feature_tracker_sift_n_octave_layers);
DECLARE_double(NEW_feature_tracker_sift_contrast_threshold);
DECLARE_double(NEW_feature_tracker_sift_edge_threshold);
DECLARE_double(NEW_feature_tracker_sift_sigma);

// SURF
DECLARE_double(NEW_feature_tracker_surf_hessian_threshold);
DECLARE_int32(NEW_feature_tracker_surf_n_octaves);
DECLARE_int32(NEW_feature_tracker_surf_octave_layers);
DECLARE_bool(NEW_feature_tracker_surf_extended);
DECLARE_bool(NEW_feature_tracker_surf_upright);

// Feature factory.
DECLARE_string(NEW_feature_detector_type);
DECLARE_string(NEW_feature_descriptor_type);
DECLARE_string(NEW_feature_pipeline_type);

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FLAGS_H_
