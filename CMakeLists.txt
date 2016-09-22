cmake_minimum_required(VERSION 2.8)
project(ceilingLocalize)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED)
# CSparse
FIND_PACKAGE( CSparse)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE( Eigen3 REQUIRED )

ADD_LIBRARY(voBase ${PROJECT_SOURCE_DIR}/src/voBase.cpp)
TARGET_LINK_LIBRARIES(voBase ${OpenCV_LIBS})

ADD_LIBRARY(voEnd ${PROJECT_SOURCE_DIR}/src/voEnd.cpp)
TARGET_LINK_LIBRARIES(voEnd ${OpenCV_LIBS} g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

ADD_LIBRARY(camera ${PROJECT_SOURCE_DIR}/src/camera.cpp)
TARGET_LINK_LIBRARIES(camera flycapture)

ADD_EXECUTABLE(testcode ${PROJECT_SOURCE_DIR}/src/testcode.cpp)
TARGET_LINK_LIBRARIES(testcode ${OpenCV_LIBS} voBase voEnd camera)

ADD_EXECUTABLE(main ${PROJECT_SOURCE_DIR}/src/main.cpp)
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS} ${CSPARSE_LIBRARY} voBase voEnd camera g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension )

# ADD_EXECUTABLE(kl_optical_flow ${PROJECT_SOURCE_DIR}/src/kl_optical_flow.cpp)
# TARGET_LINK_LIBRARIES(kl_optical_flow ${OpenCV_LIBS})
#ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)

