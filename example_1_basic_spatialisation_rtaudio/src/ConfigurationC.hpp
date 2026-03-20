
#include <BRTLibrary.h>
#include "AppUtils.hpp"

#ifndef _CONFIGURATION_C_HPP_
#define _CONFIGURATION_C_HPP_

#define LISTENER_HRTF_MODEL_ID "listenerHRTF"
#define LISTENER_BRIR_MODEL_ID "listenerAmbisonicBRIR"

#define SOFA1_FILEPATH "../../resources/hrtf.sofa"
#define SOFA2_FILEPATH "../../resources/3DTI_BRIR_Trapezoid_44100Hz_3D.sofa"

#define ILD_NearFieldEffect_44100 "../../resources/NearFieldCompensation_ILD_44100.sofa"
#define ILD_NearFieldEffect_48000 "../../resources/NearFieldCompensation_ILD_48000.sofa"
#define ILD_NearFieldEffect_96000 "../../resources/NearFieldCompensation_ILD_96000.sofa"


class CConfigurationC
{
public:
    CConfigurationC() {};    


    void Setup(BRTBase::CBRTManager* brtManager, const std::string& listenerID) {
        		
        brtManager->BeginSetup();
        
        std::shared_ptr<BRTBase::CListener> listener = brtManager->GetListener(listenerID);


        std::shared_ptr<BRTListenerModel::CListenerDirectHRTFConvolutionModel>listenerHRTFModel = brtManager->CreateListenerModel<BRTListenerModel::CListenerDirectHRTFConvolutionModel>(LISTENER_HRTF_MODEL_ID);
		if (listenerHRTFModel == nullptr) {
			std::cout << "Error creating listener model" << std::endl;
		}        
        bool control = listener->ConnectListenerModel(LISTENER_HRTF_MODEL_ID);
        if (!control) {
            std::cout << "Error connecting listener model" << std::endl;
        }

        std::shared_ptr<BRTListenerModel::CListenerAmbisonicReverberantVirtualLoudspeakersModel>listenerAmbisonicBRIRModel = brtManager->CreateListenerModel<BRTListenerModel::CListenerAmbisonicReverberantVirtualLoudspeakersModel>(LISTENER_BRIR_MODEL_ID);
		if (listenerAmbisonicBRIRModel == nullptr) {
			std::cout << "Error creating listener model" << std::endl;
		}
		control = listener->ConnectListenerModel(LISTENER_BRIR_MODEL_ID);
		if (!control) {
			std::cout << "Error connecting listener model" << std::endl;
		}

        brtManager->EndSetup();		
    }

    void LoadResources(BRTBase::CBRTManager* brtManager, const std::string& listenerID) {
        std::shared_ptr<BRTBase::CListener> listener = brtManager->GetListener(listenerID);
        
        /// Load HRTF
        std::shared_ptr<BRTServices::CSphericalInterpolatedFIRTable> hrtf1 = std::make_shared<BRTServices::CSphericalInterpolatedFIRTable>();
        bool hrtfSofaLoaded1 = AppUtils::LoadHRTFSofaFile(SOFA1_FILEPATH, hrtf1);        
        // Set one for the listener. We can change it at runtime    
        if (hrtfSofaLoaded1) {
            listener->SetHRTF(hrtf1);
        }

        /// LOAD NEARFIELD ILD coefficients 
        std::shared_ptr<BRTServices::CSphericalSOSTable> sosFilter = std::make_shared<BRTServices::CSphericalSOSTable>();
        bool nearFieldFilterLoaded = AppUtils::LoadNearFieldSOSFilter(ILD_NearFieldEffect_44100, sosFilter);
        // Set to the listener
        if (nearFieldFilterLoaded) {            
            listener->SetNearFieldCompensationFilters(sosFilter);
            // Another way to set the near field filters
            //brtManager.GetListenerModel<BRTListenerModel::CListenerModelBase>(LISTENER_MODEL_ID)->SetNearFieldCompensationFilters(sosFilter);
        }

		/// Load BRIR
		std::shared_ptr<BRTServices::CSphericalFIRTable> brir = std::make_shared<BRTServices::CSphericalFIRTable>();
		bool brirSofaLoaded = AppUtils::LoadBRIRSofaFile(SOFA2_FILEPATH, brir,0,0,0,0);
        if (brirSofaLoaded) {
            listener->SetHRBRIR(brir);
        }
    }

    void ConnectSoundSource(BRTBase::CBRTManager* brtManager, const std::string& _soundSourceID) {
        brtManager->BeginSetup();
        
        std::shared_ptr<BRTListenerModel::CListenerModelBase> listenerHRTFModel = brtManager->GetListenerModel<BRTListenerModel::CListenerModelBase>(LISTENER_HRTF_MODEL_ID);
        if (listenerHRTFModel != nullptr) {              
            bool control = listenerHRTFModel->ConnectSoundSource(_soundSourceID);
			if (!control) {
				std::cout << "Error connecting sound source" << std::endl;
			}                                    
        }

		std::shared_ptr<BRTListenerModel::CListenerModelBase> listenerBRIRModel = brtManager->GetListenerModel<BRTListenerModel::CListenerModelBase>(LISTENER_BRIR_MODEL_ID);
        if (listenerBRIRModel != nullptr) {        
            bool control = listenerBRIRModel->ConnectSoundSource(_soundSourceID);
            if (!control) {
                std::cout << "Error connecting sound source" << std::endl;
            }
        }
        brtManager->EndSetup();
    }

private:

};

#endif