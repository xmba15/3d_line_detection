cmake_minimum_required(VERSION 3.10)

set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/Sphere.cpp
)

add_library(${LIBRARY_NAME}
  SHARED
    ${SOURCE_FILES}
)

target_include_directories(${LIBRARY_NAME}
  SYSTEM PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${LIBRARY_NAME}
  PUBLIC
    ${PCL_LIBRARIES}
)
