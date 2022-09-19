cmake_minimum_required(VERSION 3.5)
project(pbvs_halcon VERSION 0.0.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall )
endif()

#### Find dependencies ####
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)

# Set realsense dependency
set(realsense2_FOUND true)
set(realsense2_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu")
set(realsense2_LIBRARIES "${realsense2_LIBRARY_DIRS}/librealsense2.so")


add_library(halcon_tracker SHARED
            src/HalconTracker.cpp
            src/pid.cpp
            )   

# Include directories
target_include_directories(halcon_tracker PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        /home/fruit/tools/vcpkg/installed/x64-linux/include # realsense include dir problem fix
        ${EIGEN3_INCLUDE_DIRS}
)

# Target dependencies 
ament_target_dependencies(halcon_tracker
  realsense2
  OpenCV
  Boost
)

ament_export_dependencies(realsense2 OpenCV Boost Eigen3)

install(TARGETS halcon_tracker
        DESTINATION lib)

# executable for testing
add_executable(halconTrackerTester test/HalconTrackerTester.cpp)
target_link_libraries(halconTrackerTester halcon_tracker ur_rtde::rtde)

add_executable(pidTester test/pidTester.cpp)
target_link_libraries(pidTester halcon_tracker)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
