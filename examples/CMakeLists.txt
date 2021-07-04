cmake_minimum_required(VERSION 3.10)

add_executable(3d_line_detection_app
  ${CMAKE_CURRENT_LIST_DIR}/LineDetectionApp.cpp
)

target_link_libraries(3d_line_detection_app
  PUBLIC
    ${LIBRARY_NAME}
)

add_executable(draw_sphere_app
  ${CMAKE_CURRENT_LIST_DIR}/DrawSphereApp.cpp
)

target_link_libraries(draw_sphere_app
  PUBLIC
    ${LIBRARY_NAME}
)
