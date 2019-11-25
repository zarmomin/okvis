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
 *  Created on: Apr 1, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file implementation/NCameraSystemWithLidar.hpp
 * @brief Header implementation file for the NCameraSystemWithLidar class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */


/// \brief okvis Main namespace of this package.
#include <okvis/cameras/NCameraSystemWithLidar.hpp>
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// Default constructor
NCameraSystemWithLidar::NCameraSystemWithLidar() : lidarCameraId_(0u)
{
}
// Construct with vector of extrinsics and geometries
NCameraSystemWithLidar::NCameraSystemWithLidar(
    const std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> & T_SC,
    const std::vector<std::shared_ptr<const cameras::CameraBase>> & cameraGeometries,
    const std::vector<DistortionType>& distortionTypes,
    bool computeOverlaps, uint64_t lidarCameraId)
    : lidarCameraId_(lidarCameraId)
{
  if (lidarCameraId_ > 0u) {
    // split into 'regular' cameras and lidar camera
    std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> T_SC_copies;
    std::vector<std::shared_ptr<const cameras::CameraBase>> cameraGeometries_copies;
    std::vector<DistortionType> distortionTypes_copies;

    for (size_t i = 0; i < cameraGeometries.size();++i) {
      const std::shared_ptr<const cameras::CameraBase> c = cameraGeometries[i];
      if (c->id() == lidarCameraId) {
        lidarCamera_ = c;
        T_SL_ = T_SC[i];
        lidarCameraDistortionType_ = distortionTypes[i];
      } else {
        T_SC_copies.emplace_back(T_SC[i]);
        cameraGeometries_copies.emplace_back(c);
        distortionTypes_copies.emplace_back(distortionTypes[i]);
      }
    }
    NCameraSystem::reset(T_SC_copies, cameraGeometries_copies, distortionTypes_copies, computeOverlaps);
  } else {
    NCameraSystem::reset(T_SC, cameraGeometries, distortionTypes, computeOverlaps);
  }
}

// Reset with vector of extrinsics and geometries
void NCameraSystemWithLidar::reset(
    const std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> & T_SC,
    const std::vector<std::shared_ptr<const cameras::CameraBase>> & cameraGeometries,
    const std::vector<DistortionType>& distortionTypes,
    bool computeOverlaps, uint64_t lidarCameraId) {
  lidarCameraId_ = lidarCameraId;
  if (lidarCameraId_ > 0u) {
    // split into 'regular' cameras and lidar camera
    std::vector<std::shared_ptr<const okvis::kinematics::Transformation>> T_SC_copies;
    std::vector<std::shared_ptr<const cameras::CameraBase>> cameraGeometries_copies;
    std::vector<DistortionType> distortionTypes_copies;

    for (size_t i = 0; i < cameraGeometries.size();++i) {
      const std::shared_ptr<const cameras::CameraBase> c = cameraGeometries[i];
      if (c->id() == lidarCameraId) {
        lidarCamera_ = c;
        T_SL_ = T_SC[i];
        lidarCameraDistortionType_ = distortionTypes[i];
      } else {
        T_SC_copies.emplace_back(T_SC[i]);
        cameraGeometries_copies.emplace_back(c);
        distortionTypes_copies.emplace_back(distortionTypes[i]);
      }
    }
    NCameraSystem::reset(T_SC_copies, cameraGeometries_copies, distortionTypes_copies, computeOverlaps);
  } else {
    NCameraSystem::reset(T_SC, cameraGeometries, distortionTypes,
                         computeOverlaps);
  }
}

// Reset with vector of extrinsics and geometries
void NCameraSystemWithLidar::addCamera(
    std::shared_ptr<const okvis::kinematics::Transformation> T_SC,
    std::shared_ptr<const cameras::CameraBase> cameraGeometry,
    DistortionType distortionType,
    bool computeOverlaps, bool isLidarCamera)
{
  if (isLidarCamera) {
    lidarCameraId_ = cameraGeometry->id();
    lidarCamera_ = cameraGeometry;
    T_SL_ = T_SC;
    lidarCameraDistortionType_ = distortionType;
  } else {
    NCameraSystem::addCamera(T_SC, cameraGeometry, distortionType, computeOverlaps);
  }
}

// Get the overlap mask
const cv::Mat NCameraSystemWithLidar::overlap(size_t cameraIndexSeenBy,
                                      size_t cameraIndex) const
{
  if (lidarCameraId_ == cameraIndex || lidarCameraId_ == cameraIndexSeenBy) {
    return cv::Mat();
  } else {
   return NCameraSystem::overlap(cameraIndexSeenBy, cameraIndex);
  }
}

// Can the first camera see parts of the FOV of the second camera?
bool NCameraSystemWithLidar::hasOverlap(size_t cameraIndexSeenBy,
                                      size_t cameraIndex) const
{
  if (lidarCameraId_ == cameraIndex || lidarCameraId_ == cameraIndexSeenBy) {
    return false;
  } else {
    return NCameraSystem::hasOverlap(cameraIndexSeenBy, cameraIndex);
  }
}

size_t NCameraSystemWithLidar::numLidarCameras() const {
  return (lidarCameraId_ > 0) ? 1u : 0u;
}

}  // namespace cameras
}  // namespace okvis
