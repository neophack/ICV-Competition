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
#include <cstdlib>

#include "tool.h"
#include "car.h"
#include "hybridAStar.h"
#include "PID.h"

//Main function
//
int main(int argc, char **argv)
{


	//-------------------------------------------------------------
	long long lastTimeStamp = 0;
    tool::Car car(2.9187,1.85,3.9187,1,0.1);
    car.setMaxSteering(M_PI/3);
    tool::Config cfg;
    tool::Obstacles obs;
    
    tool::Obstacle ob;
    ob.type = OBSTACLE;
    for(int i=7;i<argc;i+=4){
        ob.x = atoi(argv[i]);
        ob.y = atoi(argv[i+1]);
        ob.length = atoi(argv[i+2]);
        ob.width = atoi(argv[i+3]);
        ob.yaw = 0;
        obs.insertObs(ob);
    }

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
    double startX=atoi(argv[1]),startY=atoi(argv[2]),startYaw=atoi(argv[3]),endX=atoi(argv[4]),endY=atoi(argv[5]),endYaw=atoi(argv[6]);
    tool::pid contr(1,0.000001,0.001);
    contr = 1.0; //慢一点好
    //-------------------------------------------------------------
    tool::hybridAStar(startX,startY,startYaw,endX,endY,endYaw,cfg,path);

    for(int i=0;i<path.size();i++){
        cout<<path.x[i]<<' '<<path.y[i]<<' '<<path.yaw[i]<<'\n';
    }
	return 0;
}
