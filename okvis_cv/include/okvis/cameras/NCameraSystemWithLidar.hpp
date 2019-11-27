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
 *  Created on: Mar 30, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file cameras/NCameraSystemWithLidar.hpp
 * @brief Header file for the NCameraSystemWithLidar class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_NCAMERASYSTEMWITHLIDAR_HPP_
#define INCLUDE_OKVIS_NCAMERASYSTEMWITHLIDAR_HPP_

#include <memory>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <opencv2/core/core.hpp> // Code that causes warning goes here
#pragma GCC diagnostic pop
#include "okvis/cameras/CameraBase.hpp"
#include "okvis/cameras/NCameraSystem.hpp"
#include <okvis/assert_macros.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

/// \class NCameraSystemWithLidar
/// \brief A class that assembles multiple cameras into a system of
/// (potentially different) cameras.
class NCameraSystemWithLidar : public NCameraSystem {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception, std::runtime_error)

  /// \brief Default constructor
  inline NCameraSystemWithLidar();
  /// \brief Construct with vector of extrinsics and geometries
  /// @param[in] T_SC a vector of extrinsics.
  /// @param[in] cameraGeometries a vector of camera geometries (same length as
  /// T_SC).
  /// @param[in] distortionTypes a vector of distortion types (same length as
  /// T_SC).
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take
  /// a while) should be performed.
  /// @param[in] lidarCameraId Indicates which camera model corresponds to the
  /// (hacky) lidar camera model
  inline NCameraSystemWithLidar(
      const std::vector<
          std::shared_ptr<const okvis::kinematics::Transformation>> &T_SC,
      const std::vector<std::shared_ptr<const cameras::CameraBase>>
          &cameraGeometries,
      const std::vector<DistortionType> &distortionTypes, bool computeOverlaps,
      uint64_t lidarCameraId = 0u);

  /// \brief Reset with vector of extrinsics and geometries
  /// @param[in] T_SC a vector of extrinsics.
  /// @param[in] cameraGeometries a vector of camera geometries (same length as
  /// T_SC).
  /// @param[in] distortionTypes a vector of distortion types (same length as
  /// T_SC).
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take
  /// a while) should be performed.
  /// @param[in] lidarCameraId Indicates which camera model corresponds to the
  /// (hacky) lidar camera model
  inline void
  reset(const std::vector<
            std::shared_ptr<const okvis::kinematics::Transformation>> &T_SC,
        const std::vector<std::shared_ptr<const cameras::CameraBase>>
            &cameraGeometries,
        const std::vector<DistortionType> &distortionTypes,
        bool computeOverlaps, uint64_t lidarCameraId = 0u);

  /// \brief Append with a single camera.
  /// @param[in] T_SC extrinsics.
  /// @param[in] cameraGeometry Camera geometry.
  /// @param[in] distortionType Distortion type.
  /// @param[in] computeOverlaps Indicate, if the overlap computation (can take
  /// a while) should be performed.
  /// @param[in] isLidarCamera Indicates, if the camera is in fact a Lidar
  inline void
  addCamera(std::shared_ptr<const okvis::kinematics::Transformation> T_SC,
            std::shared_ptr<const cameras::CameraBase> cameraGeometry,
            DistortionType distortionType, bool computeOverlaps,
            bool isLidarCamera);

  /// \brief Get the overlap mask. Sorry for the weird syntax, but remember that
  /// cv::Mat is essentially a shared pointer.
  /// @param[in] cameraIndexSeenBy The camera index for one camera.
  /// @param[in] cameraIndex The camera index for the other camera.
  /// @return The overlap mask image.
  inline const cv::Mat overlap(size_t cameraIndexSeenBy,
                               size_t cameraIndex) const override;

  /// \brief Can the first camera see parts of the FOV of the second camera?
  /// @param[in] cameraIndexSeenBy The camera index for one camera.
  /// @param[in] cameraIndex The camera index for the other camera.
  /// @return True, if there is at least one pixel of overlap.
  inline bool hasOverlap(size_t cameraIndexSeenBy,
                         size_t cameraIndex) const override;

  /// \brief Obtatin the number of Lidar-cameras currently added.
  /// @return The number of Lidar cameras.
  inline size_t numLidarCameras() const;

  inline std::shared_ptr<const cameras::CameraBase>
  lidarCameraGeometry() const {
    return lidarCamera_;
  }
  inline std::shared_ptr<const okvis::kinematics::Transformation> T_SL() const {
    return T_SL_;
  }

protected:
  uint64_t lidarCameraId_;
  std::shared_ptr<const cameras::CameraBase> lidarCamera_;
  std::shared_ptr<const okvis::kinematics::Transformation> T_SL_;
  DistortionType lidarCameraDistortionType_;
};

} // namespace cameras
} // namespace okvis

#include "implementation/NCameraSystemWithLidar.hpp"

#endif /* INCLUDE_OKVIS_NCAMERASYSTEMWITHLIDAR_HPP_ */
