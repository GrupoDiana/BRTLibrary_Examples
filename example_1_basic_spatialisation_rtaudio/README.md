Folder Content
-

- projects: project files for Windows (Visual Studio).
- resources: files needed by the example program to work (HRTF, BRIR and audio files). These files must be copied into the same folder as executable file.
- src: source files of the BasicSpatialisationWithPortaudio example project (`BasicSpatialisationRTAudio.cpp` and `BasicSpatialisationRTAudio.h`).

How to Build and Run in Windows
-
1. Clone the repository and its submodules ("BRTLibrary" and "third_party_libraries/rtaudio").

2. Before using RtAudio:

The developer needs to specify by the use of preprocessor definitions which APIs they want to use in order to RtAudio to compile them and be able to use them. By default, the Visual Studio solution provided compiles WASAPI by defining ____WINDOWS_WASAPI____. Depending on the audio output device and the OS used, this may be subject to change by the user. Further information can be found in: https://www.music.mcgill.ca/~gary/rtaudio/compiling.html

3. Open the solution `BasicSpatializationRTAudio.sln` located at 
`localPath\BRTLibrary_Examples\example_1_basic_spatialisation_rtaudio\projects\vstudio` 
This has been tested with Visual Studio 2022 (v143) and Windows SDK 10.0.17763.0. 

4. Compile the project. 

5. Run the project
