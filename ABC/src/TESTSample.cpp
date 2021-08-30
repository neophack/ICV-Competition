#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "util/UtilDriver.h"
#include "utilStartSimOneNode.h"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "../HDMap/include/SampleGetLaneST.h"

#include <memory>
#include <limits>
#include <iostream>

#include "tool.h"

//Main function
//
int main()
{

	bool inAEBState = false;
	bool isSimOneInitialized = false;
	StartSimOne::WaitSimOneIsOk(true);
	SimOneSM::SetDriverName(0, "TEST");

	int timeout = 20;
	while (true) {
		if (SimOneSM::LoadHDMap(timeout)) {
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loaded");
			break;
		}
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loading...");
	}

	hello();

    SSD::SimPoint3DVector targetPath;

    while (true) {
		int frame = SimOneAPI::Wait();

		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
			break;
		}

		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		if (!SimOneAPI::GetSimOneGps(pGps.get())) {
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch GPS failed");
		}
		
		if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && pGps->timestamp > 0) {
			if (!isSimOneInitialized) {
				SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initialized!");
				isSimOneInitialized = true;
			}
            std::unique_ptr<SimOne_Data_Control> controler = std::make_unique<SimOne_Data_Control>();
            //controler->throttleMode = EThrottleMode_Accel;
            controler->throttle = 0.f;
            controler->brake = 0.f;
            controler->handbrake = false;
            controler->isManualGear = false;
            controler->gear = static_cast<EGearMode>(1);
            controler->steering = 0.f;
			double v = UtilMath::calculateSpeed(pGps->velX,pGps->velY,pGps->velZ);
            if(v < 10){
                controler->throttle = 1.5;
                controler->brake = 0;
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug,"v < 10,set throttle to 1.5");
            } else{
                controler->throttle = 0;
                controler->brake = 1;
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug,"v >= 10,set brake to 1");
            }

            if(!leftCenterLine(pGps.get(),targetPath)) {
                rightCenterLine(pGps.get(), targetPath);
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug,"no left lane,get right lane");
            }else{
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug,"get left lane");
            }

            controler->steering = UtilDriver::calculateSteering(targetPath,pGps.get());

            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug,"set steering:%lf",controler->steering);

            if(!SimOneSM::SetDrive(0,controler.get()))
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError,"cannot set controler");
		}
		else {
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
