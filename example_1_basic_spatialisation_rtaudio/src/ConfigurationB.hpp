
#include <BRTLibrary.h>
#include "AppUtils.hpp"

#ifndef _CONFIGURATION_B_HPP_
#define _CONFIGURATION_B_HPP_

#define LISTENER_HRTF_MODEL_ID "listenerHRTF1"
#define ENVIRONMENT_MODEL_ID "FreeField"

#define SOFA1_FILEPATH "../../resources/hrtf.sofa"

#define ILD_NearFieldEffect_44100 "../../resources/NearFieldCompensation_ILD_44100.sofa"
#define ILD_NearFieldEffect_48000 "../../resources/NearFieldCompensation_ILD_48000.sofa"
#define ILD_NearFieldEffect_96000 "../../resources/NearFieldCompensation_ILD_96000.sofa"


class CConfigurationB
{
public:
    CConfigurationB() {};    


    void Setup(BRTBase::CBRTManager* brtManager, const std::string& listenerID) {
        		
        brtManager->BeginSetup();
        
        std::shared_ptr<BRTListenerModel::CListenerHRTFModel>listenerModel = brtManager->CreateListenerModel<BRTListenerModel::CListenerHRTFModel>(LISTENER_HRTF_MODEL_ID);
		if (listenerModel == nullptr) {
			std::cout << "Error creating listener model" << std::endl;            
		}
        
        std::shared_ptr<BRTEnvironmentModel::CFreeFieldEnvironmentModel> environmentModel = brtManager->CreateEnvironment<BRTEnvironmentModel::CFreeFieldEnvironmentModel>(ENVIRONMENT_MODEL_ID);
		if (environmentModel == nullptr) {
			std::cout << "Error creating environment model" << std::endl;            
		}

        std::shared_ptr<BRTBase::CListener> listener = brtManager->GetListener(listenerID);
        bool control = listener->ConnectListenerModel(LISTENER_HRTF_MODEL_ID);
        if (!control) {
            std::cout << "Error connecting listener model" << std::endl;
        }

        control = listenerModel->ConnectEnvironmentModel(ENVIRONMENT_MODEL_ID);        
        if (!control) {
            std::cout << "Error connecting enviroment model" << std::endl;
        }

        brtManager->EndSetup();		
    }

    void LoadResources(BRTBase::CBRTManager* brtManager, const std::string& listenerID) {
        std::shared_ptr<BRTBase::CListener> listener = brtManager->GetListener(listenerID);
        
        /// Load HRTF
        std::shared_ptr<BRTServices::CHRTF> hrtf1 = std::make_shared<BRTServices::CHRTF>();
        bool hrtfSofaLoaded1 = AppUtils::LoadHRTFSofaFile(SOFA1_FILEPATH, hrtf1);        
        // Set one for the listener. We can change it at runtime    
        if (hrtfSofaLoaded1) {
            listener->SetHRTF(hrtf1);
        }

        /// LOAD NEARFIELD ILD coefficients 
        std::shared_ptr<BRTServices::CSOSFilters> sosFilter = std::make_shared<BRTServices::CSOSFilters>();                
        bool nearFieldFilterLoaded = AppUtils::LoadNearFieldSOSFilter(ILD_NearFieldEffect_44100, sosFilter);
        // Set to the listener
        if (nearFieldFilterLoaded) {            
            listener->SetNearFieldCompensationFilters(sosFilter);
            // Another way to set the near field filters
            //brtManager.GetListenerModel<BRTListenerModel::CListenerModelBase>(LISTENER_MODEL_ID)->SetNearFieldCompensationFilters(sosFilter);
        }

    }

    void ConnectSoundSource(BRTBase::CBRTManager* brtManager, const std::string& _soundSourceID) {               
        std::shared_ptr<BRTEnvironmentModel::CEnviromentModelBase> environmentModel = brtManager->GetEnvironmentModel<BRTEnvironmentModel::CEnviromentModelBase>(ENVIRONMENT_MODEL_ID);
        if (environmentModel != nullptr) {
            brtManager->BeginSetup();
            bool control = environmentModel->ConnectSoundSource(_soundSourceID);
            brtManager->EndSetup();
        }
    }

    void ConfigureFreeFieldEnviromentModel(BRTBase::CBRTManager* brtManager, bool _enableDistanceAttenuation, bool _enablePropagationDelay) {
		std::shared_ptr<BRTEnvironmentModel::CEnviromentModelBase> environmentModel = brtManager->GetEnvironmentModel<BRTEnvironmentModel::CEnviromentModelBase>(ENVIRONMENT_MODEL_ID);
        if (environmentModel != nullptr) {
            if (_enableDistanceAttenuation) {
                environmentModel->EnableDistanceAttenuation();
            }
            else {
                environmentModel->DisableDistanceAttenuation();
            }
            if (_enablePropagationDelay) {
                environmentModel->EnablePropagationDelay();
            }
            else
            {
                environmentModel->DisablePropagationDelay();
            }
        }
    }    
private:

};

#endif