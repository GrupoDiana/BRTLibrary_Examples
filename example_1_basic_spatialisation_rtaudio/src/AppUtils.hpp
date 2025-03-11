#include <string>
#include <BRTLibrary.h>

#define HRTFRESAMPLINGSTEP 15

static class AppUtils {
public:
    static bool LoadSofaFile(const std::string & _filePath, std::shared_ptr<BRTServices::CHRTF> hrtf) {
                
        BRTReaders::CSOFAReader sofaReader;
        Common::CGlobalParameters globalParameters;      

        int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_filePath);
        if (sampleRateInSOFAFile == -1) {
            std::cout << ("Error loading HRTF Sofa file") << std::endl;
            return false;
        }
        if (globalParameters.GetSampleRate() != sampleRateInSOFAFile)
        {
            std::cout << "The sample rate in HRTF SOFA file doesn't match the configuration." << std::endl;
            return false;
        }
		std::cout << std::endl << "Loadind SOFA File....." << std::endl;
        bool result = sofaReader.ReadHRTFFromSofa(_filePath, hrtf, HRTFRESAMPLINGSTEP, BRTServices::TEXTRAPOLATION_METHOD::nearest_point);
        if (result) {
            std::cout << ("HRTF Sofa file loaded successfully.") << std::endl;
            //HRTF_list.push_back(hrtf);
            return true;
        }
        else {
            std::cout << ("Error loading HRTF") << std::endl;
            return false;
        }
    }


    static bool LoadNearFieldSOSFilter(std::string _ildFilePath, std::shared_ptr<BRTServices::CSOSFilters> _sosFilter) {
        
        BRTReaders::CSOFAReader sofaReader;
        Common::CGlobalParameters globalParameters;

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
        
        std::cout << std::endl << "Loadind SOS File....." << std::endl;
        bool result = sofaReader.ReadSOSFiltersFromSofa(_ildFilePath, _sosFilter);
        if (result) {
            std::cout << "ILD Sofa file loaded successfully: " << std::endl;
            //nearFieldFilters_list.push_back(sosFilterToAdd);
            return true;
        }
        else {
            std::cout << "Error loading HRTF" << std::endl;
            return false;
        }
    }
private:    

};