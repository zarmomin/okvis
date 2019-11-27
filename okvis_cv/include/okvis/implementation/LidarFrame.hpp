/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Mar 31, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file implementation/LidarFrame.hpp
 * @brief Header implementation file for the LidarFrame class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/lidar/helpers.h>

/// \brief okvis Main namespace of this package.
namespace okvis {

// a constructor that uses the specified geometry,
/// detector and extractor
LidarFrame::LidarFrame(const cv::Mat & image,
             std::shared_ptr<cameras::CameraBase> & cameraGeometry,
             std::shared_ptr<okvis::lidar::FeatureDetectorBase> & detector,
             std::shared_ptr<okvis::lidar::FeatureDescriberBase> & extractor)
    : image_(image),
      cameraGeometry_(cameraGeometry),
      detector_(detector),
      extractor_(extractor)
{
}

// set the frame image;
void LidarFrame::setImage(const cv::Mat & image)
{
  image_ = image;
}

// set the geometry
void LidarFrame::setGeometry(std::shared_ptr<const cameras::CameraBase> cameraGeometry)
{
  cameraGeometry_ = cameraGeometry;
}

// set the detector
void LidarFrame::setDetector(std::shared_ptr<okvis::lidar::FeatureDetectorBase> detector)
{
  detector_ = detector;
}

// set the extractor
void LidarFrame::setExtractor(std::shared_ptr<okvis::lidar::FeatureDescriberBase> extractor)
{
  extractor_ = extractor;
}

// obtain the image
const cv::Mat & LidarFrame::image() const
{
  return image_;
}

// get the base class geometry (will be slow to use)
std::shared_ptr<const cameras::CameraBase> LidarFrame::geometry() const
{
  return cameraGeometry_;
}

// detect keypoints. This uses virtual function calls.
///        That's a negligibly small overhead for many detections.
///        returns the number of detected points.
int LidarFrame::detect()
{
  // currently all points allowed
  cv::Mat detection_mask(image_.rows, image_.cols, CV_8UC1, cv::Scalar(255));

  // make sure things are set to zero for safety
  keypoints_;

  // run the detector
  OKVIS_ASSERT_TRUE_DBG(Exception, detector_ != NULL,
                        "Detector not initialised!");
  detector_->detectFeatures(image_, 300, detection_mask, &keypoints_);
  return numKeypoints();
}

// describe keypoints. This uses virtual function calls.
///        That's a negligibly small overhead for many detections.
///        \param extractionDirection the extraction direction in camera frame
///        returns the number of detected points.
int LidarFrame::describe(const Eigen::Vector3d & extractionDirection)
{
  // check initialisation
  OKVIS_ASSERT_TRUE_DBG(Exception, extractor_ != NULL,
                        "Detector not initialised!");

  std::vector<cv::KeyPoint> keypoints_cv;
  KeyframeFeaturesToCvPoints(keypoints_, &keypoints_cv);
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints_cv.size(); ++k) {
    cv::KeyPoint& ckp = keypoints_cv[k];
    // project ray
    cameraGeometry_->backProject(Eigen::Vector2d(ckp.pt.x, ckp.pt.y), &ep);
    // obtain image Jacobian
    cameraGeometry_->project(ep, &reprojection, &Jacobian);
    // multiply with gravity direction
    eg_projected = Jacobian * extractionDirection;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  CvKeypointsToKeyframeFeatures(keypoints_cv, &keypoints_);
  extractor_->describeFeatures(image_, &keypoints_);

}

int LidarFrame::describe() {
  extractor_->describeFeatures(image_, &keypoints_);
  return numKeypoints();
}

size_t LidarFrame::numKeypoints() const {
  return keypoints_.keypoint_measurements.cols();
}

}  // namespace okvis
