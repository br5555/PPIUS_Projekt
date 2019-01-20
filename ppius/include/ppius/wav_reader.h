#ifndef WAV_READER_
#define WAV_READER_

#include <math.h>
#include <stdlib.h>
#include <string> 
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdint>

#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include <ppius_msg/ppius.h> 


#include <stdint.h>


#include <unistd.h>
#include <typeinfo>

using std::cin;
using std::cout;
using std::endl;
using std::fstream;
using std::string;

typedef struct  WAV_HEADER
{
    /* RIFF Chunk Descriptor */
    uint8_t         RIFF[4];        // RIFF Header Magic header
    uint32_t        ChunkSize;      // RIFF Chunk Size
    uint8_t         WAVE[4];        // WAVE Header
    /* "fmt" sub-chunk */
    uint8_t         fmt[4];         // FMT header
    uint32_t        Subchunk1Size;  // Size of the fmt chunk
    uint16_t        AudioFormat;    // Audio format 1=PCM,6=mulaw,7=alaw,     257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM
    uint16_t        NumOfChan;      // Number of channels 1=Mono 2=Sterio
    uint32_t        SamplesPerSec;  // Sampling Frequency in Hz
    uint32_t        bytesPerSec;    // bytes per second
    uint16_t        blockAlign;     // 2=16-bit mono, 4=16-bit stereo
    uint16_t        bitsPerSample;  // Number of bits per sample
    /* "data" sub-chunk */
    uint8_t         Subchunk2ID[4]; // "data"  string
    uint32_t        Subchunk2Size;  // Sampled data length
} wav_hdr;


class WavReader
{
public:
    WavReader(ros::NodeHandle* nodehandle, const char* filePath);
private:
    ros::NodeHandle nh_;
    ros::Publisher sound_pub_;
    const char* filePath_;

    void initializePublisher();
    
    void readWavFile(const char* filePath);
    int getFileSize(FILE* inFile);
    
};

#endif
