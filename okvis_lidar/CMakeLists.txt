cmake_minimum_required(VERSION 2.8.3)
project(okvis_lidar)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/feature-describer-brisk.cc
  src/feature-describer-freak.cc
  src/feature-describer-sift.cc
  src/feature-describer-surf.cc
  src/feature-detector-brisk.cc
  src/feature-detector-gftt.cc
  src/feature-detector-orb.cc
  src/feature-detector-sift.cc
  src/feature-detector-surf.cc
  #src/feature-tracker-gyro-aided.cc
  #src/feature-tracker-gyro-aided-laser.cc
  src/flags.cc
  src/keyframe-features.cc
  src/helpers.cc
  #src/pnp-ransac.cc
)

# and link it
target_link_libraries(${PROJECT_NAME}
        PRIVATE ${OpenCV_LIBRARIES}
        PRIVATE ${BRISK_LIBRARIES}
        )

# installation if required
install(TARGETS ${PROJECT_NAME}
        EXPORT okvisTargets
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
        )
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.h")
