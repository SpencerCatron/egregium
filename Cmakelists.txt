cmake_minimum_required(VERSION 3.20)
project(egregium LANGUAGES CXX)

# ---- Global settings -------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies --------------------------- -------------
include(FetchContent)

# Eigen: linear algebra (vectors, sparse matrices, solvers)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
)

# Polyscope: 3D viewer with ImGui built in
FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG        v2.3.0
  GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(eigen polyscope)

# ---- The library -----------------------------------------------------------
add_library(egregium src/mesh/surface_mesh.cpp)
target_include_directories(egregium PUBLIC include)



# ---- Google Tests -----------------------------------------------------------------
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# for windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
 

#testing--------------------
enable_testing()
add_executable(tests tests/test_surface_mesh.cpp)
target_link_libraries(tests PRIVATE egregium GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(tests)
 
