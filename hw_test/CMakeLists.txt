cmake_minimum_required(VERSION 3.22)

# Shared drivers (dxl, mpu6050) live in Software/components
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hw_test)
