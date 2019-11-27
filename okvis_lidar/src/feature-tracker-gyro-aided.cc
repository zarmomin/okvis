#include "okvis/lidar/feature-tracker-gyro-aided.h"

#include <limits>
#include <mutex>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <glog/logging.h>
#include <maplab-common/conversions.h>
#include <okvis/lidar/feature-pipeline-base.h>
#include <sensors/imu.h>
#include <vio-common/vio-types.h>

namespace okvis {
namespace lidar {

FeatureTrackerGyroAided::FeatureTrackerGyroAided(
    const aslam::NCamera &camera_system,
    FeatureTrackingPipelineBase *tracking_pipeline)
    : camera_system_(camera_system),
      current_imu_bias_(Eigen::Matrix<double, 6, 1>::Zero()),
      current_imu_bias_timestamp_nanoseconds_(aslam::time::getInvalidTime()),
      prev_nframe_timestamp_ns_(-1),
      tracker_pipeline_(CHECK_NOTNULL(tracking_pipeline)) {
  // Initialize an empty keyframe stucture and images.
  prev_nframe_images_.resize(camera_system_.numCameras());
  prev_nframe_features_.resize(camera_system_.numCameras());
}

bool FeatureTrackerGyroAided::processNFrame(
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> imu_timestamps_since_last_nframe,
    Eigen::Matrix<double, 6, Eigen::Dynamic> imu_measurements_since_last_nframe,
    aslam::VisualNFrame *in_out_nframe) {
  CHECK(in_out_nframe != nullptr);

  // Check if the IMU bias is up to date, if not - use zero.
  if (!hasUpToDateImuBias(in_out_nframe->getMinTimestampNanoseconds())) {
    LOG(WARNING) << "No bias from the estimator available. Assuming zero bias.";
    std::unique_lock<std::mutex> bias_lock(m_current_imu_bias_);
    current_imu_bias_.setZero();
  }

  // Pre-integrate the IMU measurements.
  aslam::Quaternion q_Ikp1_Ik;
  integrateInterframeImuRotation(imu_timestamps_since_last_nframe,
                                 imu_measurements_since_last_nframe,
                                 &q_Ikp1_Ik);

  // Detect, track, ransac the features.
  std::lock_guard<std::mutex> lock(m_prev_nframe_data_);
  CHECK_GT(in_out_nframe->getMinTimestampNanoseconds(),
           prev_nframe_timestamp_ns_);
  CHECK_EQ(prev_nframe_features_.size(), camera_system_.numCameras());
  CHECK_EQ(prev_nframe_images_.size(), camera_system_.numCameras());

  std::vector<cv::Mat> curr_nframe_images;
  for (size_t i = 0u; i < camera_system_.numCameras(); ++i) {
    curr_nframe_images.emplace_back(in_out_nframe->getFrame(i).getRawImage());
  }

  std::vector<feature_tracking_pipelines::KeyframeFeatures>
      curr_nframe_features;

  std::shared_ptr<feature_tracking_pipelines::FeaturePipelineDebugData>
      tracking_debug_output(
          new feature_tracking_pipelines::FeaturePipelineDebugData);
  tracking_debug_output->timestamp_nframe_kp1 =
      in_out_nframe->getMaxTimestampNanoseconds();
  tracking_debug_output->timestamp_nframe_k = prev_nframe_timestamp_ns_;

  CHECK(tracker_pipeline_);
  tracker_pipeline_->processImages(camera_system_, q_Ikp1_Ik,
                                   curr_nframe_images, prev_nframe_images_,
                                   prev_nframe_features_, &curr_nframe_features,
                                   tracking_debug_output.get());

  feature_tracking_pipelines::ApplyKeypointFeaturesToVisualNFrame(
      curr_nframe_features, in_out_nframe);

  if (debug_data_callback_) {
    debug_data_callback_(tracking_debug_output);
  }

  prev_nframe_timestamp_ns_ = in_out_nframe->getMinTimestampNanoseconds();
  prev_nframe_features_.swap(curr_nframe_features);
  prev_nframe_images_.swap(curr_nframe_images);
  return true;
}

void FeatureTrackerGyroAided::setCurrentImuBias(
    int64_t timestamp_ns, const Eigen::Matrix<double, 6, 1> &imu_bias) {
  // Only update the bias if we have a newer measurement.
  std::unique_lock<std::mutex> lock(m_current_imu_bias_);
  if (timestamp_ns > current_imu_bias_timestamp_nanoseconds_) {
    current_imu_bias_timestamp_nanoseconds_ = timestamp_ns;
    current_imu_bias_ = imu_bias;
    VLOG(5) << "Updated IMU bias in Pipeline node.";
  } else {
    LOG(WARNING) << "Received an IMU bias estimate that has an earlier "
                 << "timestamp than the previous one. Previous timestamp: "
                 << current_imu_bias_timestamp_nanoseconds_
                 << "ns, received timestamp: " << timestamp_ns << "ns.";
  }
}

bool FeatureTrackerGyroAided::hasUpToDateImuBias(
    const int64_t current_timestamp_ns) const {
  std::unique_lock<std::mutex> lock(m_current_imu_bias_);
  if (current_imu_bias_timestamp_nanoseconds_ == -1) {
    // No bias was ever set.
    return false;
  }
  constexpr int64_t kImuBiasAgeThresholdNs = 10 * kSecondsToNanoSeconds;
  if (current_timestamp_ns - current_imu_bias_timestamp_nanoseconds_ >
      kImuBiasAgeThresholdNs) {
    // The bias estimate is not up to date.
    return false;
  }
  return true;
}

void FeatureTrackerGyroAided::integrateInterframeImuRotation(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic> &imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic> &imu_measurements,
    aslam::Quaternion *q_Ikp1_Ik) const {
  CHECK_NOTNULL(q_Ikp1_Ik);
  CHECK_GT(imu_timestamps.cols(), 2);
  CHECK_EQ(imu_measurements.cols(), imu_timestamps.cols());

  q_Ikp1_Ik->setIdentity();
  for (int i = 1; i < imu_measurements.cols(); ++i) {
    const double delta_s =
        (imu_timestamps(i) - imu_timestamps(i - 1)) * kNanosecondsToSeconds;
    CHECK_GT(delta_s, 0);
    std::unique_lock<std::mutex> bias_lock(m_current_imu_bias_);
    const Eigen::Vector3d gyro_measurement =
        imu_measurements.col(i).tail<3>() - current_imu_bias_.tail<3>();
    bias_lock.unlock();

    *q_Ikp1_Ik =
        *q_Ikp1_Ik * aslam::Quaternion::exp(gyro_measurement * delta_s);
  }
  // We actually need to inverse the rotation so that transform from Ikp1 to Ik.
  *q_Ikp1_Ik = q_Ikp1_Ik->inverse();
}

} // namespace lidar
} // namespace okvis
