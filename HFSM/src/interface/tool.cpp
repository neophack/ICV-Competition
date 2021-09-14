//
// Created by 谢卫凯 on 2021/8/26.
//

#include "tool.h"

#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "car.h"

#include <cmath>

bool setDriver(float acc,float steering,int direction){
    double throttle,brake;
    std::unique_ptr<SimOne_Data_Control> controler = std::make_unique<SimOne_Data_Control>();
    tool::a2contr(acc,throttle,brake);

    controler->gear = direction == 1?EGearMode_Drive:EGearMode_Reverse;
    controler->throttle = throttle;
    controler->brake = brake;
    controler->steering = -(360.0 * steering / (2 * M_PI))/60.0;
    controler->handbrake = false;
    controler->isManualGear = false;
    return SimOneSM::SetDrive(0,controler.get());
}

bool getSuccessorLane(SSD::SimString laneId,SSD::SimPoint3DVector &targetPath){
    bool ret;
    HDMapStandalone:: MLaneLink linkInfo;
    HDMapStandalone::MLaneInfo laneInfo;
    SimOneSM::GetLaneLink(laneId,linkInfo);
    if(linkInfo.successorLaneNameList.size() <= 0)
        return false;
    ret = SimOneSM::GetLaneSample(linkInfo.successorLaneNameList[0],laneInfo);
    if(!ret)
        return false;
    targetPath = laneInfo.centerLine;

    return true;
}

void appendPath(SSD::SimPoint3DVector &targetPath,SSD::SimPoint3DVector &path){
    for(int i=0;i<path.size();i++)
        targetPath.push_back(path[i]);
}

void logPath(SSD::SimPoint3DVector &Path){
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation,"start to print path----------------------------------");
    for(int i=0;i<Path.size();i++){
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation,"(%lf,%lf,%lf)",Path[i].x,Path[i].y,Path[i].z);
    }
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation,"end to print path----------------------------------");
}

bool getPath(SSD::SimString laneId,SSD::SimPoint3DVector &path){
    bool ret;
    HDMapStandalone::MLaneInfo laneInfo;
    SSD::SimPoint3DVector successor;

    ret = SimOneSM::GetLaneSample(laneId,laneInfo);
    if (!ret)
        return false;
    path = laneInfo.centerLine;
    ret = getSuccessorLane(laneId,successor);
    if(!ret)
        return true;//不接下一段车道
    appendPath(path,successor);

    return true;
}

void hello(){
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "hello hello hello hello hello hello--------");
}

int leftCenterLine(SimOne_Data_Gps *gps,SSD::SimPoint3DVector &targetPath){
    SSD::SimString laneId;
    double s, t, s_toCenterLine, t_toCenterLine;
    bool check;
    HDMapStandalone::MLaneLink laneLink;
    HDMapStandalone::MLaneInfo laneInfo;

    SSD::SimPoint3D pos(gps->posX,gps->posY,gps->posZ);
    check = SimOneSM::GetNearMostLane(pos,laneId,s,t,s_toCenterLine,t_toCenterLine);
    if(!check){
        //SimOneAPI::bridgeLogOutput(2,"cannot get most near lane");
        return false;
    }

    SimOneSM::GetLaneLink(laneId,laneLink);
    laneId = laneLink.leftNeighborLaneName;
    //check = SimOneSM::GetLaneSample(laneId,laneInfo);
    check = getPath(laneId,targetPath);

    if(!check)
        return false;

    //targetPath = laneInfo.centerLine;

    return true;
}

int rightCenterLine(SimOne_Data_Gps *gps,SSD::SimPoint3DVector &targetPath){
    SSD::SimString laneId;
    double s, t, s_toCenterLine, t_toCenterLine;
    bool check;
    HDMapStandalone::MLaneLink laneLink;
    HDMapStandalone::MLaneInfo laneInfo;

    SSD::SimPoint3D pos(gps->posX,gps->posY,gps->posZ);
    check = SimOneSM::GetNearMostLane(pos,laneId,s,t,s_toCenterLine,t_toCenterLine);
    if(!check){
        //SimOneAPI::bridgeLogOutput(2,"cannot get most near lane");
        return false;
    }

    SimOneSM::GetLaneLink(laneId,laneLink);
    laneId = laneLink.rightNeighborLaneName;
    //check = SimOneSM::GetLaneSample(laneId,laneInfo);
    check = getPath(laneId,targetPath);
    if(!check)
        return false;

    //targetPath = laneInfo.centerLine;

    return true;
}

int centerLine(SimOne_Data_Gps *gps,SSD::SimPoint3DVector &targetPath){
    SSD::SimString laneId;
    double s, t, s_toCenterLine, t_toCenterLine;
    bool check;
    HDMapStandalone::MLaneLink laneLink;
    HDMapStandalone::MLaneInfo laneInfo;

    SSD::SimPoint3D pos(gps->posX,gps->posY,gps->posZ);
    check = SimOneSM::GetNearMostLane(pos,laneId,s,t,s_toCenterLine,t_toCenterLine);
    if(!check){
        //SimOneAPI::bridgeLogOutput(2,"cannot get most near lane");
        return false;
    }

    check = SimOneSM::GetLaneSample(laneId,laneInfo);
    if(!check)
        return false;

    targetPath = laneInfo.centerLine;

    return true;
}
