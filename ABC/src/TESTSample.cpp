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
    tool::Car car(2.9187,1.85,3.9187,1,0.1);
    car.setMaxSteering(M_PI/3);
    tool::Config cfg;
    tool::Obstacles obs;
    bool pathReady = false;bool stop = false;bool toGo = false;
    cfg.gridRes = 0.3;
    cfg.yawRes = M_PI/12;
    cfg.maxSteering = M_PI/4;
    cfg.NSteer = 48;
    cfg.car = car;
    cfg.obs = obs;
    cfg.obsCost = 15;
    cfg.steerCost = 0.0;
    cfg.switchBackCost = 0.5;

    tool::Path path;
    double startX=-100.404797,startY=-125.4392,startYaw=0,endX=-84.47935,endY=-65.4569,endYaw=0;
    tool::pid contr(1,0.000001,0.001);
    contr = 1.0; //慢一点好
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
			float acc;
			double steering;
            double speed = UtilMath::calculateSpeed(pGps->velX,pGps->velY,pGps->velZ);
            startX = pGps->posX;startY = pGps->posY;startYaw = tool::pi_2_pi(pGps->oriZ);
            if(lastTimeStamp == 0)
                lastTimeStamp = pGps->timestamp;
            if(!pathReady || pGps->timestamp-lastTimeStamp >= 1000) {
                tool::hybridAStar(startX,startY,startYaw,endX,endY,endYaw,cfg,path);
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError,
                                           "start from(%lf,%lf,%lf) path ready!!!!!!!!!!!!----------",startX,startY,startYaw);
                for (int i = 0; i < path.size(); i++) {
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "x:%lf y:%lf yaw:%lf d:%d", path.x[i],
                                               path.y[i],
                                               path.yaw[i], path.d[i]);
                }
                pathReady = true;
                car.setPath(&path);
                car.setK(0.05,0.1);
                lastTimeStamp = pGps->timestamp;
            }
            steering = car.getSteering();
            car.setState(pGps->posX,pGps->posY,pGps->oriZ,speed);

			if(speed <= 1.0)
			    acc = 1;
			else
			    acc = -1;
			if(std::sqrt(std::pow(endY-pGps->posY,2)+std::pow(endX-pGps->posX,2)) <= 0.3)
			    stop = true;
			if(stop) {
			    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning,"in stop state!!!!");
                acc = -20.0;
            }
			if(pGps->timestamp - lastTimeStamp <= 10000 && !toGo) {
                acc = -20.0;
                toGo = true;
            }
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning,"car direction:%d",car.direction);
            setDriver(acc,steering,car.direction);

            //-----------------------------------------------------------------------------------------------------------
		}
		else {
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
		}

		SimOneAPI::NextFrame(frame);
	}
	return 0;
}
