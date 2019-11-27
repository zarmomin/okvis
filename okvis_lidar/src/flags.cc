#include "okvis/lidar/flags.h"

#include <gflags/gflags.h>
// #include <maplab-common/conversions.h>

#include <cmath>

namespace okvis {
namespace lidar {

constexpr double kDegToRad = M_PI / 180.0;

DEFINE_int32(NEW_feature_tracker_num_features, 300, "Number of features.");
DEFINE_int32(NEW_feature_tracker_num_pyramid_levels, 3,
             "Number of pyramid levels.");
DEFINE_double(NEW_feature_tracker_pyramid_scale_decimation, 1.5,
              "Pyramid decimation ratio, greater than 1.");

// Ransac.
DEFINE_double(
    NEW_feature_tracker_ransac_threshold, 1.0 - cos(0.5 * kDegToRad),
    "Threshold for the RANSAC; defined as (1 - cos(alpha)) where alpha is "
    "the angle between the predicted and measured bearing vectors.");
DEFINE_int32(NEW_feature_tracker_ransac_max_iters, 200,
             "Max ransac iterations");
DEFINE_bool(NEW_feature_tracker_ransac_fix_seed, false,
            "Fix ransac random seed?");

// ORB detector specific settings.
DEFINE_int32(
    NEW_feature_tracker_detector_orb_edge_threshold, 31,
    "Edge threshold of the detector. Should roughly match the descriptor"
    "pattern size.");
DEFINE_int32(
    NEW_feature_tracker_detector_orb_fast_threshold, 20,
    "Threshold on difference between intensity of the central pixel and pixels "
    "of a circle around this pixel.");

// Brisk.
DEFINE_double(NEW_feature_tracker_brisk_uniformity_radius, 5.0,
              "BRISK uniformity radius parameter.");
DEFINE_double(NEW_feature_tracker_detector_brisk_absolute_threshold, 35.0,
              "BRISK absolute threshold parameter.");

// SIFT
DEFINE_int32(NEW_feature_tracker_sift_n_features, 0, "Number of features.");
DEFINE_int32(NEW_feature_tracker_sift_n_octave_layers, 3, "Number of layers.");
DEFINE_double(NEW_feature_tracker_sift_contrast_threshold, 0.04,
              "Contrast threshold.");
DEFINE_double(NEW_feature_tracker_sift_edge_threshold, 10, "edge threshold.");
DEFINE_double(NEW_feature_tracker_sift_sigma, 1.6,
              "The sigma of the Gaussian.");

// SURF
DEFINE_double(NEW_feature_tracker_surf_hessian_threshold, 100,
              "Hessian threshold.");
DEFINE_int32(NEW_feature_tracker_surf_n_octaves, 4, "Number of octaves.");
DEFINE_int32(NEW_feature_tracker_surf_octave_layers, 3,
             "Numer of octave layers.");
DEFINE_bool(NEW_feature_tracker_surf_extended, false, "Use 64 or 128 floats.");
DEFINE_bool(NEW_feature_tracker_surf_upright, false,
            "Computation of the feature orientation.");

// Feature factory.
DEFINE_string(NEW_feature_detector_type, "brisk", "Feature detector type.");
DEFINE_string(NEW_feature_descriptor_type, "freak",
              "descriptor pipeline type.");
DEFINE_string(NEW_feature_pipeline_type, "lk", "Feature pipeline type.");

} // namespace lidar
} // namespace okvis
