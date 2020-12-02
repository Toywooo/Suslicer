cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# add our macro directory to cmake search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# add project setting
set (OPTION_TEST_UNIT true)

set(APP_NAME "suSlicer")
project(${APP_NAME})

set(PROJECT_LIB_DIR  ${${APP_NAME}__SOURCE_DIR}lib)  #empty now
set(PROJECT_INC_DIR  ${${APP_NAME}__SOURCE_DIR}include)

FILE(GLOB SRC_SOURCES *.cpp)
FILE(GLOB SRC_HEADERS *.h )


set(SRC_LIST ${SRC_SOURCES} 
             ${SRC_HEADERS}
)
#https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
              
include_directories(
  ${PROJECT_INC_DIR}  
)
link_directories(
  ${PROJECT_LIB_DIR}  
)              
              
######################################################################			      
set(CMAKE_BUILD_TYPE Release)

add_executable (${APP_NAME} ${SRC_LIST} )

#target_link_libraries (${APP_NAME}
#    ${OPENMESH_LIBRARIES}    	
#)

# add sub projects
IF(OPTION_TEST_UNIT)
  add_subdirectory("unit_test")  
ENDIF(OPTION_TEST_UNIT)
