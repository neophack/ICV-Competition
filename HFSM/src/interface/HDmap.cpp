//
// Created by 谢卫凯 on 2021/9/23.
//

#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"


#include "HDmap.h"

#include <stdio.h>

namespace interface{

    void SimString2Lane(SSD::SimString &s,lane &lane){
        sscanf(s.GetString(),"%d_%d_%d",&lane.roadID,&lane.sectionInd,&lane.laneID);
    }

    void Lane2SimString(const lane &lane, SSD::SimString &s) {
        char tmp[100];
        sprintf(tmp,"%d_%d_%d",lane.roadID,lane.sectionInd,lane.laneID);
        s.SetString(tmp);
    }

    bool getLaneST(const lane &lane, double x, double y, double &s, double &t) {
        SSD::SimString id;
        SSD::SimPoint3D pos(x,y,0);
        Lane2SimString(lane,id);

        return SimOneSM::GetLaneST(id,pos,s,t);
    }

    bool getRoadSt(const lane &lane, double x, double y, double &s, double &t) {
        SSD::SimString id;
        SSD::SimPoint3D pos(x,y,0);
        double z;
        Lane2SimString(lane,id);

        return SimOneSM::GetRoadST(id,pos,s,t,z);
    }

    bool ST2XYZ(const lane &lane, const double &s, const double &t, double &x, double &y, double &z) {
        SSD::SimPoint3D pos,dir;
        SSD::SimString id;
        Lane2SimString(lane,id);

        if(!SimOneSM::GetInertialFromLaneST(id,s,t,pos,dir))
            return false;

        x = pos.x;
        y = pos.y;
        z = pos.z;

        return true;
    }

    bool getLaneInfoFromGps(SimOne_Data_Gps *gps, lane &l, double &s, double t) {
        SSD::SimPoint3D pos(gps->posX,gps->posY,gps->posZ);
        SSD::SimString id;
        double ignoredS,ignoredT;

        if(!SimOneSM::GetNearMostLane(pos,id,s,t,ignoredS,ignoredT))
            return false;
        SimString2Lane(id,l);

        return true;
    }

}
