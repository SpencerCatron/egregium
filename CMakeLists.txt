cmake_minimum_required(VERSION 3.20)
project(egregium LANGUAGES CXX)

# ---- Global settings -------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----------------------------------------------------------
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
add_library(egregium src/mesh/surface_mesh.cpp
src/geometry/geometry.cpp
src/io/obj.cpp
src/operators/laplacian.cpp
src/algorithms/heat_diffusion.cpp
src/algorithms/mean_curvature_flow.cpp)
target_include_directories(egregium PUBLIC include)
target_link_libraries(egregium PUBLIC Eigen3::Eigen)
# ---- GoogleTest ------------------------------------------------------------
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# Windows: use the same CRT as the parent project.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ---- Tests --------------------------------------
enable_testing()
add_executable(tests tests/test_surface_mesh.cpp 
tests/test_geometry.cpp 
tests/test_obj.cpp
tests/test_laplacian.cpp)
target_link_libraries(tests PRIVATE egregium GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(tests)


# --- Viewer ------------------------------------
add_executable(viewer viewer/main.cpp)
target_link_libraries(viewer PRIVATE egregium polyscope)
target_compile_definitions(viewer PRIVATE EGREGIUM_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
