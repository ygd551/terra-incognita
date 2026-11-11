# Sophus for FAST-LIVO2 and rpg_vikit (both call find_package(Sophus); ros1.sh build passes -DSophus_DIR=/ws/cmake).
# They need the old non-templated Sophus at commit a621ff (2012-08-25), built as a shared library, from the shared
# third-party tree (THIRDPARTY_ROOT, passed by ros1.sh build; default ~/thirdparty there): headers in src/, the library
# in src/build/ (never installed, so its own build-tree SophusConfig.cmake is not used). ros1.sh mounts the tree into
# the container at the same path. Missing -> stop here instead of falling back to some other Sophus.
set(ISAAC_EXPLORE_SOPHUS_VERSION "2012.08.25-ga621ff2")
set(_sophus_root "${THIRDPARTY_ROOT}/Sophus/${ISAAC_EXPLORE_SOPHUS_VERSION}/src")
if(NOT EXISTS "${_sophus_root}/build/libSophus.so")
  message(FATAL_ERROR "Sophus ${ISAAC_EXPLORE_SOPHUS_VERSION} not found at ${_sophus_root} (build/libSophus.so)")
endif()
find_package(Eigen3 REQUIRED)
set(Sophus_FOUND TRUE)
set(Sophus_INCLUDE_DIRS "${_sophus_root};${EIGEN3_INCLUDE_DIR}")
set(Sophus_INCLUDE_DIR "${Sophus_INCLUDE_DIRS}")
set(Sophus_LIBRARIES "${_sophus_root}/build/libSophus.so")
set(Sophus_LIBRARY "${Sophus_LIBRARIES}")
