# Set minimum CMake required version for this project.
cmake_minimum_required(VERSION 3.3)

# Create C++ project
# Define a C++ project.
project(BRTLibrary_Examples LANGUAGES CXX VERSION 1.0)

# Donwload the BRT Library
# FIXME: take to separate dependecies.cmake file 
# with include(dependencies.cmake)
include(FetchContent)
FetchContent_Declare(
    brt
    GIT_REPOSITORY "https://github.com/GrupoDiana/BRTLibrary.git"
    GIT_TAG origin/cmake
    GIT_PROGRESS true
)
FetchContent_MakeAvailable(brt)

# Add subdirectories for examples
add_subdirectory(example_1_basic_spatialisation_rtaudio)
add_subdirectory(example_2_basic_spatialisation_juce)
