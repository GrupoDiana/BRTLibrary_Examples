/**
*
* \brief This is the header file of the example project 1 using BRT Library
* \date	June 2023
*
* \authors 3DI-DIANA Research Group (University of Malaga), in alphabetical order: M. Cuevas-Rodriguez, D. Gonzalez-Toledo ||
* Coordinated by , A. Reyes-Lecuona (University of Malaga)||
* \b Contact: areyes@uma.es
*
* \b Copyright: University of Malaga
*
* \b Contributions: (additional authors/contributors can be added here)
*
* \b Project: SONICOM ||
* \b Website: https://www.sonicom.eu/
*
* \b Acknowledgement: This project has received funding from the European Union�s Horizon 2020 research and innovation programme under grant agreement no.101017743
*
* \b Licence: This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*/

#include "BasicSpatialisationRTAudio.h"

#if defined(__linux__) || defined(linux)
    #include <bits/stdc++.h>    
#endif

int iBufferSize;

int main()
{
    /// Configure BRT Error handler
    BRT_ERRORHANDLER.SetVerbosityMode(VERBOSITYMODE_ERRORSANDWARNINGS);
    BRT_ERRORHANDLER.SetErrorLogStream(&std::cout, true);

    /// Show introduction message
	ShowIntroduction(); 
    
    /// Select Buffer Size    
    std::cout << "Insert wished buffer size (256, 512, 1024, 2048, 4096...)\n(2048 at least recommended for linux)\t: ";
    std::cin >> iBufferSize; 
    std::cin.ignore();
        
    /// AUDIO setup            
    AudioSetup();

    /// BRT Global Parametert setup    
    globalParameters.SetSampleRate(SAMPLERATE);     // Setting sample rate
    globalParameters.SetBufferSize(iBufferSize);    // Setting buffer size
           
    //////////////////////////
    // Listener Setup
    //////////////////////////      
    brtManager.BeginSetup();
    listener = brtManager.CreateListener<BRTBase::CListener>(LISTENER_ID);
    brtManager.EndSetup();
    Common::CTransform listenerPosition = Common::CTransform();		 // Setting listener in (0,0,0)
    listenerPosition.SetPosition(Common::CVector3(0, 0, 0));
    listener->SetListenerTransform(listenerPosition);

    /////////////////////////////////////
    // Create and connnect BRT modules
    /////////////////////////////////////
    char selection = ShowConfigurationMenu();
    if (selection == 'A')
    {
        configurationA.Setup(&brtManager, LISTENER_ID);
        configurationA.LoadResources(&brtManager, LISTENER_ID);
	}
    else if (selection == 'B') {
        configurationB.Setup(&brtManager, LISTENER_ID);
        configurationB.LoadResources(&brtManager, LISTENER_ID);
        configurationB.ConfigureFreeFieldEnviromentModel(&brtManager, true, true);
	}
	else if (selection == 'C') {
		configurationC.Setup(&brtManager, LISTENER_ID);
		configurationC.LoadResources(&brtManager, LISTENER_ID);
    } else {
		std::cout << "Invalid option. Exiting program.\n";
		return 0;
    }
                            
    /////////////////////
    // Create Sources
    /////////////////////            
    source1BRT = CreateSimpleSoundSource("speech");
    source2BRT = CreateSimpleSoundSource("steps");
    
    if (selection == 'A') {
        configurationA.ConnectSoundSource(&brtManager, "speech");
        configurationA.ConnectSoundSource(&brtManager, "steps");
	}
	else if (selection == 'B') {
		configurationB.ConnectSoundSource(&brtManager, "speech");
		configurationB.ConnectSoundSource(&brtManager, "steps");
	}
    else if (selection == 'C') {
        configurationC.ConnectSoundSource(&brtManager, "speech");
        configurationC.ConnectSoundSource(&brtManager, "steps");
    }
            
    /////////////////////
    // Setup Sources
    /////////////////////      
    Common::CTransform source1 = Common::CTransform();
    source1.SetPosition(Spherical2Cartesians(SOURCE1_INITIAL_AZIMUTH, SOURCE1_INITIAL_ELEVATION, SOURCE1_INITIAL_DISTANCE));
    source1BRT->SetSourceTransform(source1);
    	
    source2Azimuth      = SOURCE2_INITIAL_AZIMUTH;
    source2Elevation    = SOURCE2_INITIAL_ELEVATION;
    source2Distance     = SOURCE2_INITIAL_DISTANCE;
    Common::CTransform source2 = Common::CTransform();
    source2.SetPosition(Spherical2Cartesians(source2Azimuth, source2Elevation, source2Distance));
    source2BRT->SetSourceTransform(source2);                                           // Set source2 position
    showSource2PositionCounter = 0;

    /////////////////////
    // Load Wav Files
    ///////////////////// 
    LoadWav(samplesVectorSource1, SOURCE1_FILEPATH);        // Loading .wav file            
    LoadWav(samplesVectorSource2, SOURCE2_FILEPATH);        // Loading .wav file

    /////////////////////
    // Start AUDIO Render
    /////////////////////     
    // Declaration and initialization of stereo buffer
  	outputBufferStereo.left.resize(iBufferSize);
  	outputBufferStereo.right.resize(iBufferSize);    
   
    // Informing user by the console to press any key to end the execution    
    std::cout << std::endl << std::endl;
    std::cout << "Press ENTER to start, and then press ENTER again when you want to finish..." << std::endl;
    std::cout << std::endl << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
    std::cin.get();  // Wait for a key press
    
    // Starting the stream
    audio->startStream();        
    
	// Wait enter to finish
    std::cin.ignore();
    char temp = getchar();

    // Stopping and closing the stream
    audio->stopStream();
    audio->closeStream();


    return 0;
}

/// Audio output configuration, using RtAudio (more info in https://www.music.mcgill.ca/~gary/rtaudio/)
void AudioSetup()
{   
    audio = std::shared_ptr<RtAudio>(new RtAudio());  // Initialization of RtAudio
    // It uses the first API it founds compiled and requires of preprocessor definitions
    // which depends on the OS used and the audio output device (more info in https://www.music.mcgill.ca/~gary/rtaudio/compiling.html)

    // Setting the output parameters
    RtAudio::StreamParameters outputParameters;
    outputParameters.nChannels = 2;									 // Setting output as stereo 

    //outputParameters.deviceId = audio->getDefaultOutputDevice();	 // Choosing default output device
    outputParameters.deviceId = ShowSelectAudioDeviceMenu();								// Give user the option to choose the output device	

    // Setting the audio stream options flags.
    RtAudio::StreamOptions options;
    options.flags = RTAUDIO_SCHEDULE_REALTIME;						 // Setting real-time audio output, comment this and uncomment next block to choose the flags of RTAudio.
    
    options.numberOfBuffers = 4;                // Setting number of buffers used by RtAudio
    options.priority = 1;                       // Setting stream thread priority
    unsigned int frameSize = iBufferSize;       // Declaring and initializing frame size variable because next statement needs it

    // Opening of audio stream
    try {
        audio->openStream(&outputParameters,     // Specified output parameters
            nullptr,			                  // Unspecified input parameters because there will not be input stream
            RTAUDIO_FLOAT32,	              // Output buffer will be 32-bit float
            SAMPLERATE,			                    // Sample rate will be 44.1 kHz
            &frameSize,		                // Frame size will be iBufferSize samples
            &rtAudioCallback,	            // Pointer to the function that will be called every time RtAudio needs the buffer to be filled
            nullptr,			                  // Unused pointer to get feedback
            &options			                  // Stream options (real-time stream, 4 buffers and priority)
        );
    }
    catch (int e) {        
        std::cout << "\nERROR RtAudio: \t" << '\n' << std::endl;
        exit(0);
    }   
}

/// Function to show the user a menu to choose the audio output device
int ShowSelectAudioDeviceMenu() {

    std::vector<unsigned int> deviceIds = audio->getDeviceIds();
	int connectedAudioDevices = audio->getDeviceCount();    
	std::string apiName = audio->getApiName(audio->getCurrentApi());

    std::cout << std::endl<< std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "     List of available audio outputs" << std::endl;    
    std::cout << "----------------------------------------" << std::endl << std::endl;
    std::cout << " Current API: " << apiName << std::endl;
	for (int i = 0; i < deviceIds.size(); i++) {
        //auto temp = audio->getDeviceInfo(deviceIds[i]);
        std::cout << "ID: " << i << " - " << audio->getDeviceInfo(deviceIds[i]).name << " ";
        std::cout << audio->getDeviceInfo(deviceIds[i]).outputChannels << " ";
        std::cout << audio->getDeviceInfo(deviceIds[i]).inputChannels << " ";        
        std::cout << std::endl;
	}
    std::cout << std::endl;
	int selectAudioDevice;	
	do {		
		std::cout << "Please choose which audio output you wish to use: ";
		std::cin >> selectAudioDevice;
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');
	} while (!(selectAudioDevice > -1 && selectAudioDevice <= connectedAudioDevices));
    std::cout << std::endl;
	return deviceIds[selectAudioDevice];
}

///
static int rtAudioCallback(void *outputBuffer, void *inputBuffer, unsigned int uiBufferSize, double streamTime, RtAudioStreamStatus status, void *data)
{
    // Setting the output buffer as float
    float * floatOutputBuffer = (float *)outputBuffer;

    // Checking if there is underflow or overflow
    if (status) std::cout << "stream over/underflow detected";

  	// Initializes buffer with zeros
	  outputBufferStereo.left.Fill(uiBufferSize, 0.0f);
	  outputBufferStereo.right.Fill(uiBufferSize, 0.0f);


    // Getting the processed audio
    audioProcess(outputBufferStereo, uiBufferSize);

    // Declaration and initialization of interlaced audio vector for correct stereo output
    CStereoBuffer<float> iOutput;
    iOutput.Interlace(outputBufferStereo.left, outputBufferStereo.right);

    // Buffer filling loop
    for (auto it = iOutput.begin(); it != iOutput.end(); it++)
    {
        floatOutputBuffer[0] = *it;						 // Setting of value in actual buffer position
        floatOutputBuffer = &floatOutputBuffer[1];				 // Updating pointer to next buffer position
    }

    // Moving the source2
    MoveSource_CircularHorizontalPath();
    ShowSource2Position();
    return 0;
}

/// Function to process audio
void audioProcess(Common::CEarPair<CMonoBuffer<float>> & bufferOutput, int uiBufferSize)
{
    // Declaration, initialization and filling mono buffers
    CMonoBuffer<float> source1Input(uiBufferSize);	FillBuffer(source1Input, wavSamplePositionSource1, positionEndFrameSource1, samplesVectorSource1);
    CMonoBuffer<float> source2Input (uiBufferSize);	FillBuffer(source2Input,  wavSamplePositionSource2,  positionEndFrameSource2,  samplesVectorSource2 );
    
    // Declaration of stereo buffer
    Common::CEarPair<CMonoBuffer<float>> bufferProcessed;
    
    source1BRT->SetBuffer(source1Input);           // Set samples in the sound source
    source2BRT->SetBuffer(source2Input);             // Set samples in the sound source        
    brtManager.ProcessAll();                        // Process all	      
    listener->GetBuffers(bufferProcessed.left, bufferProcessed.right);          // Get out buffers
    

    bufferOutput.left += bufferProcessed.left;
    bufferOutput.right += bufferProcessed.right;
    
}

/// Function to fill buffer with audio from the wav file
void FillBuffer(CMonoBuffer<float> &output, unsigned int& position, unsigned int& endFrame, std::vector<float>& samplesVector)
{
    position = endFrame + 1;							 // Set starting point as next sample of the end of last frame
    if (position >= samplesVector.size())				 // If the end of the audio is met, the position variable must return to the beginning
        position = 0;

    endFrame = position + output.size() - 1;			 // Set ending point as starting point plus frame size
    for (int i = 0; i < output.size(); i++) {
        if ((position + i) < samplesVector.size())
            output[i] = (samplesVector[position + i]);	 // Fill with audio
        else
            output[i] = 0.0f;							 // Fill with zeros if the end of the audio is met
    }
}

void LoadWav(std::vector<float>& samplesVector, const char* stringIn)
{
    struct WavHeader								 // Local declaration of wav header struct type (more info in http://soundfile.sapp.org/doc/WaveFormat/)
    {												 // We only need the number of samples, so the rest will be unused assuming file is mono, 16-bit depth and 44.1kHz sampling rate
        char		  fill[40];
        uint32_t	bytesCount;
    } wavHeader;

    FILE* wavFile = fopen(stringIn, "rb");											 // Opening of the wav file
    fread(&wavHeader, sizeof(wavHeader), 1, wavFile);								 // Reading of the 44 bytes of header to get the number of samples of the file
    fseek(wavFile, sizeof(wavHeader), SEEK_SET);									 // Moving of the file pointer to the start of the audio samples

    unsigned int samplesCount = wavHeader.bytesCount / 2;							 // Getting number of samples by dividing number of bytes by 2 because we are reading 16-bit samples
    int16_t *sample; sample = new int16_t[samplesCount];							 // Declaration and initialization of 16-bit signed integer pointer
    memset(sample, 0, sizeof(int16_t) * samplesCount);								 // Setting its size

    uint8_t *byteSample; byteSample = new uint8_t[2 * samplesCount];				 // Declaration and initialization of 8-bit unsigned integer pointer
    memset(byteSample, 0, sizeof(uint8_t) * 2 * samplesCount);						 // Setting its size

    fread(byteSample, 1, 2 * samplesCount, wavFile);								 // Reading the whole file byte per byte, needed for endian-independent wav parsing

    for (int i = 0; i < samplesCount; i++)
        sample[i] = int16_t(byteSample[2 * i] | byteSample[2 * i + 1] << 8);		 // Conversion from two 8-bit unsigned integer to a 16-bit signed integer

    samplesVector.reserve(samplesCount);											 // Reserving memory for samples vector

    for (int i = 0; i < samplesCount; i++)
        samplesVector.push_back((float)sample[i] / (float)INT16_MAX);				 // Converting samples to float to push them in samples vector
}


///////////////////////
// MENU
///////////////////////
// Function to display the introduction message
void ShowIntroduction() {
    std::cout << "============================================\n";
    std::cout << "     Welcome to the BRT Library\n";
    std::cout << "============================================\n\n";
    std::cout << "BRT is a modular library designed to provide highly configurable rendering adaptable to various needs.\n";
    std::cout << "Thanks to its flexible structure, it allows multiple modules to be interconnected to optimize performance according to the user's specific requirements.\n\n";
    std::cout << "This example demonstrates different ways to instantiate and configure the modules, although many more possibilities are not explored here, such as rendering with multiple listeners.\n\n";
    std::cout << "The purpose of this example is to serve as an introductory guide to using the library. You are free to use, copy, or modify the code as needed.\n\n";
    
    std::cout << "As a demonstration, in this example, you will hear a woman's voice coming from your left while footsteps move around you simultaneously.\n";
    std::cout << "The screen will display the position of these footsteps in real-time.\n\n";

    std::cout << "============================================\n\n";
}

// Function to display the configuration menu and get the user’s choice
char ShowConfigurationMenu() {
    char option;

    do {
        std::cout << "===== CONFIGURATION MENU =====\n";
        std::cout << "A) Sources --> Listener HRTF Model (Nearfield + Convolution) --> Listener\n";
        std::cout << "B) Sources --> Environment Model (FreeField) --> ListenerHRTFModel (Nearfield + Convolution) --> Listener\n";
        std::cout << "C) Sources|--> Listener HRTF Model (Nearfield + Convolution) --> |Listener\n";
        std::cout << "          |--> Listener BRIR Model (Ambisonic)               --> |\n";
        std::cout << "D) No yet implemented\n";
        std::cout << "Select an option (A-D): ";
        std::cin >> option;

        // Convert to uppercase to avoid issues with lowercase input
        option = std::toupper(option);

        if (option < 'A' || option > 'D') {
            std::cout << "❌ Invalid option. Please try again.\n";
        }
    } while (option < 'A' || option > 'D');  // Repeat until a valid option is chosen

    return option;
}

///////////////////////
// SOURCE MOVEMENT
///////////////////////
void MoveSource_CircularHorizontalPath() {

    Common::CVector3 newPosition;
    source2Azimuth += SOURCE2_INITIAL_SPEED;
    if (source2Azimuth > 2* M_PI) source2Azimuth = 0;
        
    // Calcule new position
    newPosition = Spherical2Cartesians(source2Azimuth, source2Elevation, source2Distance);
	// Just in case Listener is not in (0,0,0)
    newPosition.x = newPosition.x + listener->GetListenerTransform().GetPosition().x;
	newPosition.y = newPosition.y + listener->GetListenerTransform().GetPosition().y;
	newPosition.z = newPosition.z + listener->GetListenerTransform().GetPosition().z;
	
    // Apply new position to source2
    Common::CTransform sourcePosition = source2BRT->GetSourceTransform();
    sourcePosition.SetPosition(newPosition);
    source2BRT->SetSourceTransform(sourcePosition);    
}

Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius) {

    float x = radius * cos(azimuth) * cos(elevation);
    float y = radius * sin(azimuth) * cos(elevation);
    float z = radius * sin(elevation);

    Common::CVector3 pos(x, y, z);

    return pos;
}

void ShowSource2Position() {

    showSource2PositionCounter++;
    if (showSource2PositionCounter == 25) {
        showSource2PositionCounter = 0;
        std::cout << "Source 2 --> Azimuth (" << rad2deg(source2Azimuth) << "), Elevation (" << rad2deg(source2Elevation) << "), Distance (" << source2Distance << ")." << std::endl;
    }
}

float rad2deg(float rad) {

    return (rad * 180.0) / M_PI;
}

///////////////////////
// BRT SETUP
///////////////////////
std::shared_ptr<BRTSourceModel::CSourceSimpleModel> CreateSimpleSoundSource(std::string _soundSourceID) {
    brtManager.BeginSetup();
        std::shared_ptr<BRTSourceModel::CSourceSimpleModel> _brtSoundSource = brtManager.CreateSoundSource<BRTSourceModel::CSourceSimpleModel>(_soundSourceID);
    brtManager.EndSetup();
    if (_brtSoundSource == nullptr) {        
		std::cout << "Error creating sound source" << std::endl;
    }
	return _brtSoundSource;
}
