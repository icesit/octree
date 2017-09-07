project(octree)
cmake_minimum_required(VERSION 2.8)

# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
endif()

find_package(Boost)
# test pcl
find_package(PCL REQUIRED COMPONENTS common io filters)
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )
find_package(eigen)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
#  ADD_EXECUTABLE(example1 examples/example1.cpp)
#  ADD_EXECUTABLE(example2 examples/example2.cpp)
#  ADD_EXECUTABLE(example3 examples/example3.cpp)
  ADD_EXECUTABLE(testpcl examples/testpcl.cpp)
  ADD_EXECUTABLE(testkeyframe
    examples/testkeyframe.cpp
    examples/keyframehandle.cpp
  )

  # test pcl
  target_link_libraries(testpcl
    ${PCL_LIBRARIES}
  )
  target_link_libraries(testkeyframe
    ${PCL_LIBRARIES}
  )
endif()

# find gtest ...
IF(IS_DIRECTORY "/usr/src/gtest/")
  MESSAGE("Found google test sources in /usr/src/gtest/")
  
  ADD_SUBDIRECTORY(/usr/src/gtest/ gtest)
  
  # mimick the behaviour of find_package(GTest)
  SET(GTEST_FOUND TRUE)
  SET(GTEST_BOTH_LIBRARIES gtest gtest_main)
  SET(GTEST_LIBRARIES gtest)
  SET(GTEST_MAIN_LIBRARIES gtest_main)
ELSE()
  FIND_PACKAGE(GTest)
ENDIF()

# build gtests if possible
#IF(GTEST_FOUND)
#  MESSAGE("Building tests with google test framework")
#	ENABLE_TESTING()
#	INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})
#	ADD_EXECUTABLE(octree-test test/octree-test.cpp)
#	TARGET_LINK_LIBRARIES(octree-test ${GTEST_MAIN_LIBRARIES})
#	ADD_TEST(octree-test octree-test)
#ENDIF()
