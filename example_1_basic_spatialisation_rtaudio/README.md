# Folder Content

- resources: files needed by the example program to work (HRTF, BRIR and audio files). These files must be copied into the same folder as executable file.
- src: source files of the BasicSpatialisationWithPortaudio example project (`BasicSpatialisationRTAudio.cpp` and `BasicSpatialisationRTAudio.h`).

# How to Build and Run

This example depends on the BRTLibrary and rtaudio. 

This version of the repository requires [cmake](https://cmake.org/) to build. It has been tested on Windows 11 and macOS Ventura 13.4. 

1. Install rtaudio from [https://github.com/rtaudio/rtaudio](https://github.com/rtaudio/rtaudio) via cmake. In windows define RTAUDIO_STATIC_MSVCRT to OFF in the cmake configuration.
   The cmake configuration allows the developer to specify which APIs they want to use in order to RtAudio to compile them and be able to use them. By default, the rtAudio cmake provides RTAUDIO_API_WASAPI for Windows and RTAUDIO_API_CORE for macOS. Depending on the audio output device and the OS used, this may be subject to change by the user. Further information can be found in: https://www.music.mcgill.ca/~gary/rtaudio/compiling.html.

3. Clone the whole repository [https://github.com/GrupoDiana/BRTLibrary_Examples.git](https://github.com/GrupoDiana/BRTLibrary_Examples.git)
   
4. Create subdirectory build; Configure using cmake or cmake-gui. The configuration will fetch the BRTLibrary from github. 

5. Compile the project.

6. Run the project. Remember to copy the files in resources to the folder where your executable is. 
