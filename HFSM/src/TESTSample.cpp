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
#include "hybridAStar.h"
#include "PID.h"
#include "obstacle.h"
#include "path.h"
#include "HDmap.h"

void printStaticObs(SimOne_Data_Gps *gps,const tool::Obstacles &obs){
    int size;
    tool::Obstacle **p = obs.getObsRange(gps->posX,gps->posY,999.9,size);
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "---------printStaticObs---------------");
    for(int i=0;i<size;i++){
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "%d:(%lf,%lf) width:%lf length:%lf yaw:%lf", i, p[i]->x, p[i]->y, p[i]->width,
                                   p[i]->length, p[i]->yaw);
    }
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "---------end of printStaticObs---------------");
}

void printDynamicObs(const path::DynamicObstacles &dyObs){
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "---------printDynamicObs---------------");

    for(int i=0;i<dyObs.dyObs.size();i++){
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "%d(%lf,%lf):(%lf,%lf) width:%lf length:%lf yaw:%lf", i,dyObs.dyObs[i]->vx,dyObs.dyObs[i]->vy,
                                   dyObs.dyObs[i]->obs.x, dyObs.dyObs[i]->obs.y, dyObs.dyObs[i]->obs.width,
                                   dyObs.dyObs[i]->obs.length, dyObs.dyObs[i]->obs.yaw);
    }

    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "---------end of printStaticObs---------------");

}

//Main function
//
int main()
{

	bool inAEBState = false;
	bool isSimOneInitialized = false;
	StartSimOne::WaitSimOneIsOk(true);
	SimOneSM::SetDriverName(0, "TEST");
	//-------------------------------------------------------------
    long long lastTimeStamp = 0;
    //-------------------------------------------------------------
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
            if(pGps->timestamp - lastTimeStamp >= 2000){
//                auto *dyObs = new path::DynamicObstacles;
//                auto *obs = new tool::Obstacles;
//                interface::getAllObstacles(*obs,*dyObs);
//
//                printStaticObs(pGps.get(),*obs);
//                printDynamicObs(*dyObs);
//
//                obs->destroy();
//                delete dyObs;
//                delete obs;
                interface::lane l;
                double s,t;
                interface::getLaneInfoFromGps(pGps.get(),l,s,t);
                SimOneAPI::bridgeLogOutput(ELogLevelDebug, "(%d,%d,%d)>>>s:%lf t:%lf", l.roadID, l.sectionInd, l.laneID,
                                           s, t);
            }

            //-----------------------------------------------------------------------------------------------------------
		}
		else {
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
