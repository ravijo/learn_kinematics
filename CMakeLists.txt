cmake_minimum_required(VERSION 2.8.3)
project(learn_kinematics)

find_package(catkin REQUIRED COMPONENTS trac_ik_lib eigen_conversions message_generation)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_service_files(
FILES
fast_kinematics_service.srv
)

generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(
LIBRARIES
CATKIN_DEPENDS
trac_ik_lib
fast_kinematics
)

include_directories(include
${catkin_INCLUDE_DIRS}
${eigen_INCLUDE_DIRS}
)

add_library(util src/util.cpp)
add_library(fast_kinematics src/fast_kinematics.cpp)

target_link_libraries(util ${catkin_LIBRARIES})
target_link_libraries(fast_kinematics ${catkin_LIBRARIES})

add_executable(compare_ik nodes/tests/compare_ik.cpp src/util.cpp)
add_executable(fast_kinematics_test nodes/tests/fast_kinematics_test.cpp src/fast_kinematics.cpp src/util.cpp)
add_executable(fast_kinematics_pub nodes/fast_kinematics_pub.cpp src/fast_kinematics.cpp src/util.cpp)
add_executable(fast_kinematics_srv nodes/fast_kinematics_srv.cpp src/fast_kinematics.cpp src/util.cpp)
add_executable(fast_kinematics_pub_test nodes/tests/fast_kinematics_pub_test.cpp)
add_executable(fast_kinematics_srv_test nodes/tests/fast_kinematics_srv_test.cpp)

target_link_libraries(compare_ik ${catkin_LIBRARIES})
target_link_libraries(fast_kinematics_test ${catkin_LIBRARIES})
target_link_libraries(fast_kinematics_pub ${catkin_LIBRARIES})
target_link_libraries(fast_kinematics_srv ${catkin_LIBRARIES})
target_link_libraries(fast_kinematics_pub_test ${catkin_LIBRARIES})
target_link_libraries(fast_kinematics_srv_test ${catkin_LIBRARIES})

add_dependencies(fast_kinematics_srv learn_kinematics)
add_dependencies(fast_kinematics_srv_test learn_kinematics)

install(TARGETS compare_ik
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS fast_kinematics_pub
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
PATTERN ".svn" EXCLUDE)
