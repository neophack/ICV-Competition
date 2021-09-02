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
#include "car.h"

//Main function
//
int main()
{

	bool inAEBState = false;
	bool isSimOneInitialized = false;
	StartSimOne::WaitSimOneIsOk(true);
	SimOneSM::SetDriverName(0, "TEST");
	long long lastTimeStamp = 0;

	double goalX = -95.1875,goalY = -128.25;
    tool::Car car(2.9187,1.85,3.9187,0.88);
    tool::Path pth;
    for(int i=0;i < 100;i++){
        pth.x.push_back(-104.0+i);
        pth.y.push_back(-134.56);
    }


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
            //-----------------------------------------------------------------------------------------------------------
            double steering,acc;
			double speed;
			int ind;
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "here1");
            speed = UtilMath::calculateSpeed(pGps->velX,pGps->velY,pGps->velZ);
            car.setState(pGps->posX,pGps->posY,pGps->oriZ,speed);
            ind = car.calcIndex(pth,0.5,2);
            steering = car.calcSteering(pth.x[ind],pth.y[ind]);
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "here2");
            if(speed <= 2)
                acc = 1;
            else
                acc = -1;
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "here3");
            setDriver(acc,steering,1);
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "x:%lf y:%lf  steering:%lf",pGps->posX,pGps->posY,steering);
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "target:x:%lf y:%lf",pth.x[ind],pth.y[ind],steering);
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "here4");
            //-----------------------------------------------------------------------------------------------------------
		}
		else {
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
