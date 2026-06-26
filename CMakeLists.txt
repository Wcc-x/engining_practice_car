# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# 显式指定自定义组件目录，避免 ESP-IDF 仅按父目录扫描时漏掉组件
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/LEDC"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/PCNT"
    "${CMAKE_CURRENT_SOURCE_DIR}/Robot"
)

include("E:/esp32/.espressif/v6.0.1/esp-idf/tools/cmake/project.cmake")
project(sample_project)
