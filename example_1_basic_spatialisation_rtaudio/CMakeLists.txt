# The example depends on rtaudio 
find_package(RtAudio CONFIG REQUIRED)

# The executable target depends on the following source files
add_executable(example1 src/BasicSpatialisationRTAudio.cpp)

# The executable target depends on the RtAudio library and the brt library
target_link_libraries(example1 PUBLIC brt RtAudio::rtaudio)

