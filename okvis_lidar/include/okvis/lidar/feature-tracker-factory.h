#ifndef OKVIS_LIDAR_FEATURE_TRACKER_FACTORY_H_
#define OKVIS_LIDAR_FEATURE_TRACKER_FACTORY_H_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "okvis/lidar/feature-describer-brisk.h"
#include "okvis/lidar/feature-describer-freak.h"
#include "okvis/lidar/feature-describer-sift.h"
#include "okvis/lidar/feature-describer-surf.h"
#include "okvis/lidar/feature-detector-brisk.h"
#include "okvis/lidar/feature-detector-gftt.h"
#include "okvis/lidar/feature-detector-orb.h"
#include "okvis/lidar/feature-detector-sift.h"
#include "okvis/lidar/feature-detector-surf.h"
#include "okvis/lidar/feature-pipeline-lk-tracking-laser.h"
#include "okvis/lidar/feature-pipeline-matching-based.h"
#include "okvis/lidar/feature-tracker-gyro-aided.h"
#include "okvis/lidar/flags.h"

namespace okvis {
namespace lidar {

FeatureTrackingPipelineBase *CreateFeaturePipelineFromGFlags() {
  // Initialize the feature detector.
  std::shared_ptr<FeatureDetectorBase> feature_detector;

  if (FLAGS_NEW_feature_detector_type == "brisk") {
    FeatureDetectorBriskSettings brisk_settings =
        InitFeatureDetectorBriskSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorBrisk(brisk_settings));
  } else if (FLAGS_NEW_feature_detector_type == "gfft") {
    feature_detector.reset(new FeatureDetectorGFTT());
  } else if (FLAGS_NEW_feature_detector_type == "orb") {
    FeatureDetectorOrbSettings orb_settings =
        InitFeatureDetectorOrbSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorOrb(orb_settings));
  } else if (FLAGS_NEW_feature_detector_type == "sift") {
    FeatureDetectorSiftSettings sift_settings =
        InitFeatureDetectorSiftSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorSift(sift_settings));
  } else if (FLAGS_NEW_feature_detector_type == "surf") {
    FeatureDetectorSurfSettings surf_settings =
        InitFeatureDetectorSurfSettingsFromGFlags();
    feature_detector.reset(new FeatureDetectorSurf(surf_settings));
  } else {
    LOG(FATAL) << "Unknown feature detector set with --feature_tracker_type: "
               << FLAGS_NEW_feature_detector_type;
  }
  CHECK(feature_detector);

  // Initialize the feature describer.
  std::shared_ptr<FeatureDescriberBase> feature_describer;
  if (FLAGS_NEW_feature_descriptor_type == "brisk") {
    FeatureDescriberBriskSettings brisk_settings =
        InitFeatureDescriberBriskSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberBrisk(brisk_settings));
  } else if (FLAGS_NEW_feature_descriptor_type == "freak") {
    FeatureDescriberFreakSettings freak_settings =
        InitFeatureDescriberFreakSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberFreak(freak_settings));
  } else if (FLAGS_NEW_feature_descriptor_type == "sift") {
    FeatureDescriberSiftSettings sift_settings =
        InitFeatureDescriberSiftSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberSift(sift_settings));
  } else if (FLAGS_NEW_feature_descriptor_type == "surf") {
    FeatureDescriberSurfSettings surf_settings =
        InitFeatureDescriberSurfSettingsFromGFlags();
    feature_describer.reset(new FeatureDescriberSurf(surf_settings));
  } else {
    LOG(FATAL) << "Unknown feature pipeline set with --feature_tracker_type: "
               << FLAGS_NEW_feature_descriptor_type;
  }
  CHECK(feature_describer);

  // Initialize the method to establish temporal feature correspondances.
  if (FLAGS_NEW_feature_pipeline_type == "lk") {
    feature_tracking_pipelines::LkTrackingSettingsLaser lk_settings =
        InitLkLaserTrackingSettingsFromGFlags();
    feature_tracking_pipelines::RansacSettings ransac_settings;
    InitRansacSettingsFromGFlags();

    return new FeaturePipelineLkTrackingLaser(
        lk_settings, ransac_settings, feature_detector, feature_describer);
  } else if (FLAGS_NEW_feature_pipeline_type == "matching-based") {
    LOG(FATAL) << "TODO(schneith): impl.";
  } else {
    LOG(FATAL) << "Unknown feature pipeline set with --feature_tracker_type: "
               << FLAGS_NEW_feature_pipeline_type;
  }
  return nullptr;
}

} // namespace lidar
} // namespace okvis

#endif // OKVIS_LIDAR_FEATURE_TRACKER_FACTORY_H_
