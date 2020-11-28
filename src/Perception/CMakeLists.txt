# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_Perception)
  set(H_PREFIX include/BipedalLocomotion/Perception/Features)
  add_bipedal_locomotion_library(
    NAME                   PerceptionFeatures
    SOURCES                src/ArucoDetector.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/ArucoDetector.h
#    SUBDIRECTORIES         tests/Perception/Features
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::GenericContainer BipedalLocomotion::CommonConversions ${OpenCV_LIBS})
endif()

