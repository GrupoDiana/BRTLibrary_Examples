/**
*
* \brief This is the header file of the example project 1 using BRT Library
* \date	June 2023
*
* \authors 3DI-DIANA Research Group (University of Malaga), in alphabetical order: M. Cuevas-Rodriguez, D. Gonzalez-Toledo, L. Molina-Tanco, F. Morales-Benitez ||
* Coordinated by , A. Reyes-Lecuona (University of Malaga)||
* \b Contact: areyes@uma.es
*
* \b Contributions: (additional authors/contributors can be added here)
*
* \b Project: SONICOM ||
* \b Website: https://www.sonicom.eu/
*
* \b Copyright: University of Malaga 2023. Code based in the 3DTI Toolkit library (https://github.com/3DTune-In/3dti_AudioToolkit) with Copyright University of Malaga and Imperial College London - 2018
*
* \b Licence: This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*
* \b Acknowledgement: This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement no.101017743
*/

#ifndef _BASICSPATIALISATIONRTAUDIO_H_
#define _BASICSPATIALISATIONRTAUDIO_H_

#define LISTENER_ID "listener1"

#define SAMPLERATE 44100

#define SOURCE1_FILEPATH "../../resources/speech.wav"
#define SOURCE2_FILEPATH "../../resources/steps.wav"
#define HRTFRESAMPLINGSTEP 15

#define SOURCE1_INITIAL_AZIMUTH     90
#define SOURCE1_INITIAL_ELEVATION   0
#define SOURCE1_INITIAL_DISTANCE    2

#define SOURCE2_INITIAL_AZIMUTH     0
#define SOURCE2_INITIAL_ELEVATION   0
#define SOURCE2_INITIAL_DISTANCE    2
#define SOURCE2_INITIAL_SPEED       0.001  

#include <cstdio>
#include <cstring>
#include <RtAudio.h>
#include <BRTLibrary.h>
#include "ConfigurationA.hpp"
#include "ConfigurationB.hpp"
#include "ConfigurationC.hpp"

std::shared_ptr<RtAudio>    audio;										// Pointer to RtAudio API
Common::CGlobalParameters globalParameters;                             // Class where the global BRT parameters are defined.
BRTBase::CBRTManager brtManager;                                        // BRT global manager interface
std::shared_ptr<BRTBase::CListener> listener;                           // Pointer to listener model

std::shared_ptr<BRTSourceModel::CSourceSimpleModel> source1BRT;           // Pointers to each audio source model
std::shared_ptr<BRTSourceModel::CSourceSimpleModel> source2BRT;           // Pointers to each audio source model

CConfigurationA configurationA;                                           // Configuration class for the example A
CConfigurationB configurationB;                                           // Configuration class for the example B
CConfigurationC configurationC;                                           // Configuration class for the example C

float source2Azimuth;
float source2Elevation;
float source2Distance;
int   showSource2PositionCounter;

Common::CEarPair<CMonoBuffer<float>>	outputBufferStereo;		// Stereo buffer containing processed audio
std::vector<float>						samplesVectorSource1;	// Storages the audio from the wav files
std::vector<float>						samplesVectorSource2;	// Storages the audio from the wav files

unsigned int wavSamplePositionSource1; // Storages, respectively, the starting and ending position of the frame being rendered for each source
unsigned int positionEndFrameSource1;	 
unsigned int wavSamplePositionSource2;
unsigned int positionEndFrameSource2;



/** \brief This method gathers all audio processing (spatialization and reverberation)
*	\param [out] bufferOutput output buffer processed
*	\param [in] bufferSize size of buffer in samples
*/
void audioProcess(Common::CEarPair<CMonoBuffer<float>>& bufferOutput, int bufferSize);

/** \brief This method shows the user a very simple menu that allows him to choose the audio interface to be used.
*	\param [out] int AudioDeviceID
*/
int ShowSelectAudioDeviceMenu();

void AudioSetup();

/** \brief Fills a buffer with the correct audio frame from the input float vector
*	\param [out] output output buffer
*	\param [in,out] position starting position of the frame to be processed
*	\param [in,out] endChunk ending position of the frame to be processed
*	\param [in] samplesVector float vector containing the whole audio
*/
void FillBuffer(CMonoBuffer<float>& output, unsigned int & position, unsigned int & endChunk, std::vector<float>& samplesVector);

/** \brief Loads a mono, 16-bit, 44.1kHz ".wav" file
*	\param [out] samplesVector float vector that will storage the whole audio
*	\param [in] stringIn name of the ".wav" file to open
*/
void LoadWav(std::vector<float>& samplesVector, const char* stringIn);

/** \brief This function is called each time RtAudio needs a buffer to output
*	\param [out] outputBuffer output buffer to be filled
*	\param [out] inputBuffer unused input buffer
*	\param [in] bufferSize size of buffer in samples
*	\param [in] streamTime time in seconds since the stream started
*	\param [in] status feedback that if does not equal zero indicates stream over/underflow
*	\param [in] data unused data pointer
*	\retval Integer value that equals 0 if we don't want the stream to stop
*/
static int rtAudioCallback(void *outputBuffer, void *inputBuffer, unsigned int bufferSize, double streamTime, RtAudioStreamStatus status, void *data);

void MoveSource_CircularHorizontalPath();
void ShowSource2Position();
Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius);
float rad2deg(float rad);


void ShowIntroduction();
char ShowConfigurationMenu();

std::shared_ptr<BRTSourceModel::CSourceSimpleModel> CreateSimpleSoundSource(std::string _soundSourceID);
#endif