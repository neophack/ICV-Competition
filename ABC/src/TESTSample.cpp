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

    tool::hybridAStar(0.0,0.0,0.0,10.0,10.0,M_PI_2,cfg,path);

    for(int i=0;i<path.size();i++){
        cout<<path.x[i]<<' '<<path.y[i]<<' '<<path.yaw[i]<<'\n';
    }

	return 0;
}
