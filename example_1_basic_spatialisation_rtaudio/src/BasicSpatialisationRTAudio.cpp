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

#include "BasicSpatialisationRTAudio.h"
#if defined(__linux__) || defined(linux)
    #include <bits/stdc++.h>
#endif

int iBufferSize;
bool bEnableReverb;
int main()
{
    //Input buffer size and reverb enable
    std::cout << "Insert wished buffer size (256, 512, 1024, 2048, 4096...)\n(2048 at least recommended for linux)\t: ";
    std::cin >> iBufferSize; std::cin.ignore();

    /*char cInput;
    do{  	std::cout << "\nDo you want reverb? (Y/n) : "; cInput=getchar();
    }while(cInput != 'y' && cInput != 'n' && cInput != '\n');

    if(cInput=='y' || cInput == '\n') bEnableReverb = true;
    else       */                       bEnableReverb = false;
    
    // Configure BRT Error handler
    BRT_ERRORHANDLER.SetVerbosityMode(VERBOSITYMODE_ERRORSANDWARNINGS);
    BRT_ERRORHANDLER.SetErrorLogStream(&std::cout, true);

    // Global Parametert setup    
    globalParameters.SetSampleRate(SAMPLERATE);     // Setting sample rate
    globalParameters.SetBufferSize(iBufferSize);    // Setting buffer size

    /////////////////////
    // Listener setup
    /////////////////////
    brtManager.BeginSetup();
    listener = brtManager.CreateListener<BRTListenerModel::CListenerHRTFbasedModel>("listener1");
    brtManager.EndSetup();    
    Common::CTransform listenerPosition = Common::CTransform();		 // Setting listener in (0,0,0)
    listenerPosition.SetPosition(Common::CVector3(0, 0, 0));
    listener->SetListenerTransform(listenerPosition);
    
    // We can activate/deactivate different parameters of the listener in the following way
    //listener->DisableSpatialization();
    //listener->DisableInterpolation();
    //listener->DisableNearFieldEffect();


    // Load HRTFs from SOFA files            
    bool hrtfSofaLoaded1 = LoadSofaFile(SOFA1_FILEPATH);
    bool hrtfSofaLoaded2 = LoadSofaFile(SOFA2_FILEPATH);
    // Set one for the listener. We can change it at runtime    
    if (hrtfSofaLoaded1) {
        listener->SetHRTF(HRTF_list[0]);
    }
    // LOAD NEARFIELD ILD coefficients 
    bool ildSofaLoaded = LoadILD(ILD_NearFieldEffect_44100);
    // Set to the listener
    if (ildSofaLoaded) {
        listener->SetILD(ILD_list[0]);
    }
         
    /////////////////////
    // source1 setup
    /////////////////////    
    brtManager.BeginSetup();
        source1BRT = brtManager.CreateSoundSource<BRTSourceModel::CSourceSimpleModel>("speech");    // Instatiate a BRT Sound Source
        listener->ConnectSoundSource(source1BRT);                                                   // Connect Source to the listener        
    brtManager.EndSetup();
    LoadWav(samplesVectorSource1, SOURCE1_FILEPATH);											    // Loading .wav file            
    Common::CTransform source1 = Common::CTransform();                         
    source1.SetPosition(Spherical2Cartesians(SOURCE1_INITIAL_AZIMUTH, SOURCE1_INITIAL_ELEVATION, SOURCE1_INITIAL_DISTANCE));
    source1BRT->SetSourceTransform(source1);                                           // Set source1 position
            
    /////////////////////
    // source2 setup
    /////////////////////
    brtManager.BeginSetup();
        source2BRT = brtManager.CreateSoundSource<BRTSourceModel::CSourceSimpleModel>("steps");      // Instatiate a BRT Sound Source
        listener->ConnectSoundSource(source2BRT);                                                    // Connect Source to the listener
    brtManager.EndSetup();    
    LoadWav(samplesVectorSource2, SOURCE2_FILEPATH);											   // Loading .wav file    
    source2Azimuth      = SOURCE2_INITIAL_AZIMUTH;
    source2Elevation    = SOURCE2_INITIAL_ELEVATION;
    source2Distance     = SOURCE2_INITIAL_DISTANCE;
    Common::CTransform source2 = Common::CTransform();
    source2.SetPosition(Spherical2Cartesians(source2Azimuth, source2Elevation, source2Distance));
    source2BRT->SetSourceTransform(source2);                                           // Set source2 position
    showSource2PositionCounter = 0;

    // Declaration and initialization of stereo buffer
  	outputBufferStereo.left.resize(iBufferSize);
  	outputBufferStereo.right.resize(iBufferSize);


    AudioSetup();

    // Starting the stream
    audio->startStream();

    // Informing user by the console to press any key to end the execution
    std::cout << "Press ENTER to finish... \n";
    std::cin.ignore();
    getchar();


    // Stopping and closing the stream
    audio->stopStream();
    audio->closeStream();


    return 0;
}

void AudioSetup()
{
    // Audio output configuration, using RtAudio (more info in https://www.music.mcgill.ca/~gary/rtaudio/)
    audio = std::shared_ptr<RtAudio>(new RtAudio());  // Initialization of RtAudio
    // It uses the first API it founds compiled and requires of preprocessor definitions
    // which depends on the OS used and the audio output device (more info in https://www.music.mcgill.ca/~gary/rtaudio/compiling.html)

    // Setting the output parameters
    RtAudio::StreamParameters outputParameters;
    outputParameters.nChannels = 2;									 // Setting output as stereo 

    //outputParameters.deviceId = audio->getDefaultOutputDevice();	 // Choosing default output device
    outputParameters.deviceId = SelectAudioDevice();								// Give user the option to choose the output device	



    // Setting the audio stream options flags.
    RtAudio::StreamOptions options;
    options.flags = RTAUDIO_SCHEDULE_REALTIME;						 // Setting real-time audio output, comment this and uncomment next block to choose the flags of RTAudio.
    /*char flag;
    do{
    cout << "\nFlags :\t0 - CONTINUE\n\t1 - REALTIME\n\t2 - MINIMIZE_LATENCY\n\t3 - HOG_DEVICE\n";
    cin >> flag;
    if(flag == '1'){
    options.flags |= RTAUDIO_SCHEDULE_REALTIME;
    }else if(flag == '2'){
    options.flags |= RTAUDIO_MINIMIZE_LATENCY;
    }else if(flag == '3'){
    options.flags |= RTAUDIO_HOG_DEVICE;
    }
    }while(flag!='0');*/
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
    catch (int e/*RtAudioError& e*/) {
        //std::cout << "\nERROR:\t" << e.getMessage() << '\n' << std::endl;
        std::cout << "\nERROR RtAudio: \t" << '\n' << std::endl;
        exit(0);
    }
    //catch (/*RtError& e*/) {
    //       //e.printMessage();
    //       //std::cout << "\nERROR:\t" << e.getMessage() << '\n' << std::endl;
    //    exit( 0 );
    //}
}

int SelectAudioDevice() {

    std::vector<unsigned int> deviceIds = audio->getDeviceIds();
	int connectedAudioDevices = audio->getDeviceCount();
	std::cout << "     List of available audio outputs" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
	for (int i = 0; i < deviceIds.size(); i++) {
        //auto temp = audio->getDeviceInfo(deviceIds[i]);
        std::cout << "ID: " << i << "-" << audio->getDeviceInfo(deviceIds[i]).name << std::endl;
	}
	int selectAudioDevice;
	//cout << "Please choose which audio output you wish to use: ";
	//cin >> selectAudioDevice; cin.ignore();	
	do {		
		std::cout << "Please choose which audio output you wish to use: ";
		std::cin >> selectAudioDevice;
		std::cin.clear();
		std::cin.ignore(INT_MAX, '\n');
	} while (!(selectAudioDevice > -1 && selectAudioDevice <= connectedAudioDevices));
	
	return deviceIds[selectAudioDevice];
}

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

bool LoadSofaFile(std::string _filePath) {
    std::shared_ptr<BRTServices::CHRTF> hrtf = std::make_shared<BRTServices::CHRTF>();

    int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_filePath);
    if (sampleRateInSOFAFile == -1) {
        std::cout << ("Error loading HRTF Sofa file") << std::endl;
        return false;
    }
    if (globalParameters.GetSampleRate() != sampleRateInSOFAFile)
    {
        std::cout<<"The sample rate in HRTF SOFA file doesn't match the configuration." << std::endl;
        return false;
    }
    bool result = sofaReader.ReadHRTFFromSofa(_filePath, hrtf, HRTFRESAMPLINGSTEP, "NearestPoint");
    if (result) {
        std::cout << ("HRTF Sofa file loaded successfully.") << std::endl;
        HRTF_list.push_back(hrtf);
        return true;
    }
    else {
        std::cout << ("Error loading HRTF") << std::endl;
        return false;
    }
}

bool LoadILD( std::string _ildFilePath) {
    std::shared_ptr<BRTServices::CILD> ild = std::make_shared<BRTServices::CILD>();
    
    
    int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_ildFilePath);
    if (sampleRateInSOFAFile == -1) {
        std::cout << ("Error loading ILD Sofa file") << std::endl;
        return false;
    }
    if (globalParameters.GetSampleRate() != sampleRateInSOFAFile)
    {
        std::cout << "The sample rate in ILD SOFA file" << std::endl;
        return false;
    }
    
    bool result = sofaReader.ReadILDFromSofa(_ildFilePath, ild);
    if (result) {
        std::cout << "ILD Sofa file loaded successfully: " << std::endl;
        ILD_list.push_back(ild);
        return true;
    }
    else {
        std::cout << "Error loading HRTF" << std::endl;
        return false;
    }            
}

///////////////////////
// SOURCE MOVEMENT
///////////////////////
void MoveSource_CircularHorizontalPath() {

    Common::CVector3 newPosition;
    source2Azimuth += SOURCE2_INITIAL_SPEED;
    if (source2Azimuth > 2* M_PI) source2Azimuth = 0;
    newPosition = Spherical2Cartesians(source2Azimuth, source2Elevation, source2Distance);
    
    Common::CTransform sourcePosition = source2BRT->GetCurrentSourceTransform();
    sourcePosition.SetPosition(newPosition);
    source2BRT->SetSourceTransform(sourcePosition);    
}

Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius) {

    float x = radius * cos(azimuth) * cos(elevation);
    float y = radius * sin(azimuth) * cos(elevation);
    float z = radius * sin(elevation);

    Common::CVector3 pos = listener->GetListenerTransform().GetPosition();

    pos.x = pos.x + x;
    pos.y = pos.y + y;
    pos.z = 0.0f;

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