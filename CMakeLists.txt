# Minimum CMake version
cmake_minimum_required(VERSION 3.15)

# Project name and version
project(EKF_Localizer VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

# Define source files
set(SOURCE_FILES
  src/main.cpp
)

# Define include directories
include_directories(
  kalman/include
  /usr/include/eigen3/
  rapidcsv/src
)

#### C-M-j Ivy immediate done. remembre this

# Add executable (or replace with add_library for libraries)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Optional: Link libraries (replace `example_library` with actual libraries)
target_link_libraries(${PROJECT_NAME})

# Optional: Add compile definitions or options
target_compile_definitions(${PROJECT_NAME} PRIVATE PRINT_DEBUG)
target_compile_options(${PROJECT_NAME} PRIVATE -g -O3 -Wall -Wextra -Wconversion)

# # Install target (optional)
# install(TARGETS ${PROJECT_NAME}
#     RUNTIME DESTINATION bin
# )
