#include <ppius/wav_reader.h>

WavReader::WavReader(ros::NodeHandle* nodehandle, const char* filePath):nh_(*nodehandle)
{ 
    filePath_  = filePath;
    initializePublisher();
    readWavFile(filePath);   
}

void WavReader::initializePublisher()
{
    
    sound_pub_ = nh_.advertise<ppius_msg::ppius>("sound_topic", 1, true); 
    
}


int WavReader::getFileSize(FILE* inFile)
{
    int fileSize = 0;
    fseek(inFile, 0, SEEK_END);

    fileSize = ftell(inFile);

    fseek(inFile, 0, SEEK_SET);
    return fileSize;
}

void WavReader::readWavFile(const char* filePath){

    wav_hdr wavHeader;
    int headerSize = sizeof(wav_hdr), filelength = 0;
    ppius_msg::ppius sound_message;

    FILE* wavFile = fopen(filePath, "r");
    if (wavFile == nullptr)
    {
        fprintf(stderr, "Unable to open wave file: %s\n", filePath);
        return ;
    }

    //Read the header
    size_t bytesRead = fread(&wavHeader, 1, headerSize, wavFile);
    
     
    cout << "Header Read " << bytesRead << " bytes." << endl;
    if (bytesRead > 0)
    {
        //Read the data
        uint16_t bytesPerSample = wavHeader.bitsPerSample / 8;      //Number     of bytes per sample
        uint64_t numSamples = wavHeader.ChunkSize / bytesPerSample; //How many samples are in the wav file?
        static const uint16_t BUFFER_SIZE = 4096;
        int8_t* buffer = new int8_t[BUFFER_SIZE];
        while ((bytesRead = fread(buffer, sizeof buffer[0], BUFFER_SIZE / (sizeof buffer[0]), wavFile)) > 0)
        {
            
            sound_message.dbl_vec.resize(bytesRead);
            for(int i = 0; i< bytesRead; i++){
                sound_message.dbl_vec[i] = buffer[i];

            }
            sound_pub_.publish(sound_message);
        }
        delete [] buffer;
        buffer = nullptr;
        filelength = getFileSize(wavFile);

        cout << "File is                    :" << filelength << " bytes." << endl;
        cout << "RIFF header                :" << wavHeader.RIFF[0] << wavHeader.RIFF[1] << wavHeader.RIFF[2] << wavHeader.RIFF[3] << endl;
        cout << "WAVE header                :" << wavHeader.WAVE[0] << wavHeader.WAVE[1] << wavHeader.WAVE[2] << wavHeader.WAVE[3] << endl;
        cout << "FMT                        :" << wavHeader.fmt[0] << wavHeader.fmt[1] << wavHeader.fmt[2] << wavHeader.fmt[3] << endl;
        cout << "Data size                  :" << wavHeader.ChunkSize << endl;

        // Display the sampling Rate from the header
        cout << "Sampling Rate              :" << wavHeader.SamplesPerSec << endl;
        cout << "Number of bits used        :" << wavHeader.bitsPerSample << endl;
        cout << "Number of channels         :" << wavHeader.NumOfChan << endl;
        cout << "Number of bytes per second :" << wavHeader.bytesPerSec << endl;
        cout << "Data length                :" << wavHeader.Subchunk2Size << endl;
        cout << "Audio Format               :" << wavHeader.AudioFormat << endl;
        // Audio format 1=PCM,6=mulaw,7=alaw, 257=IBM Mu-Law, 258=IBM A-Law, 259=ADPCM

        cout << "Block align                :" << wavHeader.blockAlign << endl;
        cout << "Data string                :" << wavHeader.Subchunk2ID[0] << wavHeader.Subchunk2ID[1] << wavHeader.Subchunk2ID[2] << wavHeader.Subchunk2ID[3] << endl;
    }
    fclose(wavFile);

}

int main(int argc, char** argv) 
{

    ros::init(argc, argv, "WavReaderNode"); 

    ros::NodeHandle nh; 
    
    
    const char* filePath;
    
    {
        filePath = argv[1];
        cout << "Input wave file name: " << filePath << endl;
    }
    

    WavReader wavReader(&nh, filePath); 


    ros::spin();
    return 0;
} 


