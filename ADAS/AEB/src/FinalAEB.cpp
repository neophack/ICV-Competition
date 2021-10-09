#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "utilStartSimOneNode.h"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "../HDMap/include/SampleGetLaneST.h"
#include "../HDMap/include/SampleGetLaneType.h"

#include <memory>
#include <limits>
#include <iostream>
#include <sstream>

using namespace HDMapStandalone;

int g_g_count = 0, g_total = 10;
const int endOfRoute = -123;
SimOne_Data_Gps Gps = SimOne_Data_Gps();
std::unique_ptr<SimOne_Data_WayPoints> pWaypoints = std::make_unique<SimOne_Data_WayPoints>();

SSD::SimPoint3D g_storedKeepPointPos(0.0f, 0.0f, 0.0f);

int isNearLanes(string S_mainVehicleLaneId, string S_mInfoLaneId) {
    stringstream ss_mainVehicleLaneId, ss_mInfoLaneId;

    for (int i = 0; i < S_mainVehicleLaneId.length(); i++) {
        if(S_mainVehicleLaneId[i] == '_')
            S_mainVehicleLaneId[i] = ' ';
    }
    for (int i = 0; i < S_mInfoLaneId.length(); i++) {
        if (S_mInfoLaneId[i] == '_')
            S_mInfoLaneId[i] = ' ';
    }
    ss_mainVehicleLaneId << S_mainVehicleLaneId;
    ss_mInfoLaneId << S_mInfoLaneId;

    int I_mainVehicleLaneId, I_mInfoLaneId;
    for (int i = 0; i < 3; i++) {
        ss_mainVehicleLaneId >> I_mainVehicleLaneId;
        ss_mInfoLaneId >> I_mInfoLaneId;
    }

    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicleLaneId = %d, mInfoLaneId = %d", I_mainVehicleLaneId, I_mInfoLaneId);
    if ((I_mainVehicleLaneId - I_mInfoLaneId) == 1)
        return 1;
    else
        return 0;
}

void UpdateRoute(SSD::SimPoint3D mainVehiclePos, SSD::SimPoint3D potentialObstaclePos, SSD::SimPoint3DVector &route) {
    SSD::SimPoint3DVector inputPoints;
    SSD::SimVector<int> indexOfValidPoints;
    HDMapStandalone::MLaneInfo mInfo;
    SSD::SimStringVector nearlanes;
    double distance = 10.0f;

    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehiclePos(%f, %f, %f)", mainVehiclePos.x, mainVehiclePos.y, mainVehiclePos.z);

    if(!SimOneSM::GetNearLanes(potentialObstaclePos, distance, nearlanes)) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Failed to get lanes near obstacle!");
        return;
    }
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "nearlanes.size = %d", nearlanes.size());
    for(int i = 0; i < nearlanes.size(); i++) {
        if(!SimOneSM::GetLaneSample(nearlanes[i], mInfo)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "GetLaneSample Failed!");
        }
        //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "No.[%d] mInfo.size = %d", i, mInfo.centerLine.size());
        SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
        SSD::SimString targetLaneId = SampleGetNearMostLane(mInfo.centerLine.front());

        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicleLaneId = %s, targetLaneId = %s", mainVehicleLaneId.GetString(), targetLaneId.GetString());

        HDMapStandalone::MLaneType targetLaneType;
        SampleGetLaneType(targetLaneId);
        if(!SimOneSM::GetLaneType(targetLaneId, targetLaneType)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch lane type failed!");
        }
        switch(targetLaneType) {
            //case HDMapStandalone::MLaneType::shoulder:
            //    break;
            case HDMapStandalone::MLaneType::driving:
                if(isNearLanes(mainVehicleLaneId.GetString(), targetLaneId.GetString())) {
                    //两条Lane相邻
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "MLaneType = MLaneType::driving, centerLine.y = %f", mInfo.centerLine.front().y);
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicleLaneId = %s, targetLaneId = %s", mainVehicleLaneId.GetString(), targetLaneId.GetString());
                    i = std::numeric_limits<int>::max();
                }
                break;
            default:
                break;
        }
        //if(targetLaneType == HDMapStandalone::MLaneType::driving && isNearLanes(mainVehicleLaneId.GetString(), targetLaneId.GetString())) {
        //    //两条Lane相邻
        //    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "MLaneType = MLaneType::driving, centerLine.y = %f", mInfo.centerLine.front().y);
        //    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicleLaneId = %s, targetLaneId = %s", mainVehicleLaneId.GetString(), targetLaneId.GetString());
        //    break;
        //}
    }

    double driveTime = 1.0f;

    SimOne_Data_Gps Gps = SimOne_Data_Gps();
    if(!SimOneAPI::GetSimOneGps(&Gps)) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fecth Gps Failed in UpdateRoute!");
    }

    SSD::SimPoint3D targetPointPos(mainVehiclePos.x + Gps.velX * driveTime, mInfo.centerLine.front().y, mInfo.centerLine.front().z);
    SSD::SimPoint3D keepPointPos(targetPointPos.x + Gps.velX * driveTime, targetPointPos.y, targetPointPos.z);
    g_storedKeepPointPos.x = keepPointPos.x, g_storedKeepPointPos.y = keepPointPos.y, g_storedKeepPointPos.z = keepPointPos.z;
    //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "storedKeepPointPos(%f, %f, %f)", g_storedKeepPointPos.x, g_storedKeepPointPos.y, g_storedKeepPointPos.z);

    SimOneAPI::bridgeLogOutput( ELogLevel_Type::ELogLevelDebug, "targetPointPos(%f, %f, %f), mainVehiclePos(%f, %f, %f), Gps.velX = %f", targetPointPos.x, targetPointPos.y, targetPointPos.z, mainVehiclePos.x, mainVehiclePos.y, mainVehiclePos.z, Gps.velX);

    inputPoints.push_back(mainVehiclePos);
    inputPoints.push_back(targetPointPos);
    inputPoints.push_back(keepPointPos);
    SimOneSM::GenerateRoute(inputPoints, indexOfValidPoints, route);
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "route.size = %d", route.size());
    for(int i = 0; i < route.size(); i++) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "routept[%d](%f, %f, %f)", i, route[i].x, route[i].y, route[i].z);
    }
}

void getTargetPath(const SSD::SimPoint3D pos, SSD::SimPoint3DVector &route) {
    if (SimOneSM::GetWayPoints(pWaypoints.get()))
    {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "wayPointsSize:%d", pWaypoints->wayPointsSize);

        //std::cout << "wayPointsSize:" << pWaypoints->wayPointsSize << std::endl;
        if ((pWaypoints->wayPointsSize) >= 2) {
            //SSD::SimPoint3DVector route;
            if (SimOneSM::GetPredefinedRoute(route)) {
                //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "route path size:%d, route[0] = (%f, %f)", route.size(), route.front().x, route.front().y);
                for(int i = 0; i < route.size(); i++) {
                    auto &item = route[i];
                    //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "pt(%f, %f, %f), mainVehiclePos(%f, %f, %f)", item.x, item.y, item.z, pos.x, pos.y, pos.z);
                }
            }
        }
        else {
            //SSD::SimPoint3D pos(Gps.posX, Gps.posY, Gps.posZ);
            SSD::SimString laneId = SampleGetNearMostLane(pos);
            HDMapStandalone::MLaneInfo info;
            if (SimOneSM::GetLaneSample(laneId, info)) {
                route = info.centerLine;
            }
        }
    }
    else {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicle not set path !!!");
    }
}

double calculateSteering(const SSD::SimPoint3DVector& targetPath, SimOne_Data_Gps *pGps, int& lastTargetPathIndex) {
    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Gps(%f, %f, %f)",
                             pGps->posX, pGps->posY, pGps->posZ);

    std::vector<float> pts;
    for (int i = 0; i < targetPath.size(); ++i)
    {
        pts.push_back(pow((pGps->posX - (float)targetPath[i].x), 2) + pow((pGps->posY - (float)targetPath[i].y), 2));
    }
    int index = min_element(pts.begin(), pts.end()) - pts.begin();
    //if(!g_g_count) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lastTargetPathIndex:%d", lastTargetPathIndex);
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "first index:%d", index);
    //}

    while (1) {
        if (index >= lastTargetPathIndex) {
            lastTargetPathIndex = index;
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lastTargetPathIndex = index = %d", lastTargetPathIndex);
            break;
        }
        else {
            if (pts.size() > 0) {
                pts.erase(pts.begin() + index);
                index = min_element(pts.begin(), pts.end()) - pts.begin();
            }
        }
    }
    if(lastTargetPathIndex >= targetPath.size() - 1) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "EndOfRoute");
        return endOfRoute;
    }

    int forwardIndex = 0;
    float minProgDist = 1.f;
    float progTime = 0.8f;
    float mainVehicleSpeed = sqrtf(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
    float progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

    for (; index < targetPath.size(); ++index)
    {
        forwardIndex = index;
        float distance = sqrtf(pow((float)targetPath[index].x - pGps->posX, 2) + pow((float)targetPath[index].y - pGps->posY, 2));
        if (distance >= progDist)
        {
            break;
        }
    }

    const double errAng = 0.1f;
    double psi = static_cast<double>(pGps->oriZ);
    double alfa = atan2(targetPath[forwardIndex].y - pGps->posY, targetPath[forwardIndex].x - pGps->posX) - psi;
    double ld = static_cast<double>(sqrt(pow(targetPath[forwardIndex].y - pGps->posY, 2) + pow(targetPath[forwardIndex].x - pGps->posX, 2)));
    double steering = 0;
    if(alfa > errAng)
        steering = -1;
    else if(alfa < -errAng)
        steering = 1;
    else {
        steering = 0;
    }
    //double steering = static_cast<double>(-atan2(2. * (1.3 + 1.55) * sin(alfa), ld) * 36. / (7. * M_PI));

    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "psi = %f, steering = %f", psi, steering);

    return steering;
}

//Main function
//
int main() {

    int g_count = 0, totalg_count = 5;
    bool isGeneratedRoute = false;
    double targetSpeed = 8.333f;
    bool inAEBState = false;
    StartSimOne::WaitSimOneIsOk(true);
    SimOneSM::SetDriverName(0, "Route");

    //Wait for the Sim-One case to run
    while (true) {
        int frame = SimOneAPI::Wait();
        SimOneAPI::GetSimOneGps(&Gps);
        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && (Gps.timestamp > 0)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "SimOne Initialized");
            SimOneAPI::NextFrame(frame);
            break;
        }

        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "SimOne Initializing...");

    }

    int timeout = 20;
    while (true) {
        if (SimOneSM::LoadHDMap(timeout)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loaded");
            break;
        }
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loading...");
    }

    //Get Predefined Route
    SSD::SimPoint3DVector route;
    SSD::SimPoint3D pos(Gps.posX, Gps.posY, Gps.posZ);
    getTargetPath(pos, route);

    while (true) {
        int frame = SimOneAPI::Wait();

        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
            break;
        }

        std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
        if (!SimOneAPI::GetSimOneGps(pGps.get())) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch GPS failed");
        }

        std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
        if (!SimOneAPI::GetSimOneGroundTruth(pObstacle.get())) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch obstacle failed");
        }

        g_count++;
        if(g_count >= g_total)
            g_count = 0;


        SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
        double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

        double safeDistance = 10.0f;
        double minDistance = std::numeric_limits<double>::max();
        int potentialObstacleIndex = pObstacle->obstacleSize;
        SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
        SSD::SimString potentialObstacleLaneId = "";
        for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
            SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
            SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
            if(obstacleLaneId == mainVehicleLaneId) {
                double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);
                if (obstacleDistance < minDistance) {
                    minDistance = obstacleDistance;
                    potentialObstacleIndex = (int)i;
                    potentialObstacleLaneId = obstacleLaneId;
                }
            }
        }

        auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
        double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);
        //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "obstacleSpeedX = %f, SpeedY = %f, speedZ = %f", potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);

        SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);

        SimOne_Data_Gps Gps = SimOne_Data_Gps();
        if(!SimOneAPI::GetSimOneGps(&Gps)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch Gps Failed! -L:297");
        }

        double sObstacle = 0;
        double tObstacle = 0;

        double sMainVehicle = 0;
        double tMainVehicle = 0;

        bool isObstacleFront = false;
        double steering = 0.0f;
        if(isGeneratedRoute) {
            double distance = UtilMath::planarDistance(mainVehiclePos, g_storedKeepPointPos);
            double errDistance = 0.5f;
            if(distance < errDistance) {
                isGeneratedRoute = false;
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Tracking generated route finished.");
            }
        }
        if (minDistance <= safeDistance && !isGeneratedRoute && !potentialObstacleLaneId.Empty()) {

            SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
            SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

            if(!g_count)
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "sMainVehicle = %f, sObstacle = %f", sMainVehicle, sObstacle);

            isObstacleFront = !(sMainVehicle >= sObstacle);
            if(isObstacleFront) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "---Barrier Avoiding State---");
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "Generating barrier avoiding route...");
                route.clear();
                UpdateRoute(mainVehiclePos, potentialObstaclePos, route);
                isGeneratedRoute = true;
            }
        }

        if(!isGeneratedRoute) {
            SSD::SimPoint3D targetPointPos;
            SSD::SimPoint3D dir;
            if(!SimOneSM::GetLaneMiddlePoint(mainVehiclePos, mainVehicleLaneId, targetPointPos, dir)) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch lane middle point failed!");
            }
            else {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "---Normal State---");
                double timeUnit = 0.5f;
                targetPointPos.x = targetPointPos.x + Gps.velX * timeUnit;
                //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "get lane middle point(%f, %f, %f)", targetPointPos.x, targetPointPos.y, targetPointPos.z);
                SSD::SimPoint3DVector inputpts;
                SSD::SimVector<int> indexOfValidPoints;
                route.clear();
                inputpts.push_back(mainVehiclePos);
                inputpts.push_back(targetPointPos);
                SimOneSM::GenerateRoute(inputpts, indexOfValidPoints, route);
            }
        }

        steering = 0;
        int lastTargetPathIndex = -1;
        steering = calculateSteering(route, &Gps, lastTargetPathIndex);
        if(steering == endOfRoute) {
            isGeneratedRoute = false;

        }

        std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

        pControl->timestamp = pGps->timestamp;
        pControl->handbrake = 0;
        pControl->isManualGear = 0;
        pControl->gear = EGearMode_Drive;
        pControl->throttleMode = EThrottleMode_Percent;
        if(mainVehicleSpeed > targetSpeed) {
            pControl->throttle = 0;
            pControl->brake = 1;
        }
        else {
            pControl->throttle = 1;
            pControl->brake = 0;
        }
        pControl->steeringMode = ESteeringMode_Percent;
        pControl->steering = steering;

        //if (isObstacleFront) {
        //    //EGear Mode

        //    double defaultDistance = 4.2f + 1.1f;

        //    double timeToCollision = std::abs((minDistance - defaultDistance) / (- mainVehicleSpeed));
        //    if(!g_count)
        //        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "TimeToCollision: %f", timeToCollision);
        //    double defaultTimeToCollision = 0.8f;

        //    if (timeToCollision < defaultTimeToCollision && timeToCollision > 0) {
        //        inAEBState = true;
        //    }
        //    if (inAEBState) {
        //        if(!g_count)
        //            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "--- In AEB State ---");
        //        pControl->gear = EGearMode_Drive;
        //        pControl->throttleMode = EThrottleMode_Accel;
        //        pControl->isManualGear = 0;
        //        double accel = std::pow((mainVehicleSpeed ), 2) / (2 * (minDistance - defaultDistance));
        //        pControl->throttle = -accel;
        //        if(!g_count) {
        //            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Acceleration: %f,calculatedAccel: %f, distance: %f", pGps->accelX, accel, std::abs(minDistance));
        //        }
        //    }
        //}
        //else {
        //    inAEBState = false;
        //}

        //if(!g_count)
        //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "obstacleSpeed = %f, mainVehicleSpeed = %f", obstacleSpeed, mainVehicleSpeed);
        SimOneSM::SetDrive(0, pControl.get());

        SimOneAPI::NextFrame(frame);
    }
    return 0;
}
