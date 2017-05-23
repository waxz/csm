cmake_minimum_required(VERSION 2.8)
project(csm)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
)

find_package(Eigen REQUIRED)

if(WIN32)
  add_definitions(-DWINDOWS)
  if(MSVC)
    # SSE2 optimizations
    add_definitions("/arch:SSE2")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
  DEPENDS Eigen
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})

set(CSM_SRC
  src/csm/laser_data.cpp
  src/csm/math_utils.cpp
  src/csm/math_utils_gsl.cpp
  src/csm/utils.cpp
  src/csm/logging.cpp
  src/csm/sm_options.cpp
  src/csm/orientation.cpp
  src/csm/clustering.cpp)

set(EGSL_SRC
  src/egsl/egsl.cpp
  src/egsl/egsl_conversions.cpp
  src/egsl/egsl_misc.cpp
  src/egsl/egsl_ops.cpp)

set(GPC_SRC
  src/gpc/gpc.cpp
  src/gpc/gpc_utils.cpp)

set(ICP_SRC
  src/icp/icp.cpp
  src/icp/icp_corr_dumb.cpp
  src/icp/icp_corr_tricks.cpp
  src/icp/icp_covariance.cpp
  src/icp/icp_debug.cpp
  src/icp/icp_loop.cpp
  src/icp/icp_outliers.cpp)

set(OPTIONS_SRC
  src/options/options.cpp
  src/options/options_interface.cpp)

add_library(${PROJECT_NAME}
  ${CSM_SRC}
  ${EGSL_SRC}
  ${GPC_SRC}
  ${ICP_SRC}
  ${OPTIONS_SRC})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/gsl_eigen/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../gsl_eigen
  FILES_MATCHING PATTERN "*.h"
)


## fix error: conflicting declaration bug
## inlude wrong header
## install all source header to /opt/ros/kinetic/include
install(DIRECTORY src/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY src/egsl/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../egsl
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY src/gpc/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../gpc
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY src/icp/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../icp
        FILES_MATCHING PATTERN "*.h"
        )

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})