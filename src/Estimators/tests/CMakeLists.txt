# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation)

  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

  add_bipedal_test(
    NAME RecursiveLeastSquare
    SOURCES RecursiveLeastSquareTest.cpp
    LINKS BipedalLocomotion::Estimators BipedalLocomotion::ParametersHandlerYarpImplementation Eigen3::Eigen)

endif()
