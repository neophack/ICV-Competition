//
// Created by 谢卫凯 on 2021/8/26.
//

#include "tool.h"

#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"


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
    check = SimOneSM::GetLaneSample(laneId,laneInfo);
    if(!check)
        return false;

    targetPath = laneInfo.centerLine;

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
    check = SimOneSM::GetLaneSample(laneId,laneInfo);
    if(!check)
        return false;

    targetPath = laneInfo.centerLine;

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
