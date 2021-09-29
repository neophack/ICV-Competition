#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "public/common/MLaneInfo.h"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "public/common/MLaneId.h"
#include "util/UtilDriver.h"
#include "utilStartSimOneNode.h"
#include "../HDMap/include/SampleGetLaneST.h"

using namespace HDMapStandalone;

int g_count = 0, g_total = 10;
SimOne_Data_Gps Gps = SimOne_Data_Gps();
std::unique_ptr<SimOne_Data_WayPoints> pWaypoints = std::make_unique<SimOne_Data_WayPoints>();
SimOne_Data_CaseInfo pCaseInfoTest = SimOne_Data_CaseInfo();

void getTargetPath(const SSD::SimPoint3D pos, SSD::SimPoint3DVector &route)
{
    if (SimOneSM::GetWayPoints(pWaypoints.get()))
    {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "wayPointsSize:%d", pWaypoints->wayPointsSize);

        //std::cout << "wayPointsSize:" << pWaypoints->wayPointsSize << std::endl;
        if ((pWaypoints->wayPointsSize) >= 2) {
            //SSD::SimPoint3DVector route;
            if (SimOneSM::GetPredefinedRoute(route)) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "route path size:%d, route[0] = (%f, %f)", route.size(), route.front().x, route.front().y);
                for(int i = 0; i < route.size(); i++) {
                    auto &item = route[i];
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "pt(%f, %f, %f), mainVehiclePos(%f, %f, %f)", item.x, item.y, item.z, pos.x, pos.y, pos.z);
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

double calculateSteering(const SSD::SimPoint3DVector& targetPath, SimOne_Data_Gps *pGps, int& lastTargetPathIndex)
{
    std::vector<float> pts;
    for (int i = 0; i < targetPath.size(); ++i)
    {
        pts.push_back(pow((pGps->posX - (float)targetPath[i].x), 2) + pow((pGps->posY - (float)targetPath[i].y), 2));
    }
    int index = min_element(pts.begin(), pts.end()) - pts.begin();
    if(!g_count) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lastTargetPathIndex:%d", lastTargetPathIndex);
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "first index:%d", index);
    }

    while (1) {
        if (index >= lastTargetPathIndex) {
            lastTargetPathIndex = index;
            break;
        }
        else {
            if (pts.size() > 0) {
                pts.erase(pts.begin() + index);
                index = min_element(pts.begin(), pts.end()) - pts.begin();
            }
        }
    }

    int forwardIndex = 0;
    float minProgDist = 3.f;
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

    double psi = static_cast<double>(pGps->oriZ);
    double alfa = atan2(targetPath[forwardIndex].y - pGps->posY, targetPath[forwardIndex].x - pGps->posX) - psi;
    double ld = static_cast<double>(sqrt(pow(targetPath[forwardIndex].y - pGps->posY, 2) + pow(targetPath[forwardIndex].x - pGps->posX, 2)));
    double steering = static_cast<double>(-atan2(2. * (1.3 + 1.55) * sin(alfa), ld) * 36. / (7. * M_PI));

    return steering;
}

//Main function
//
int main()
{
    StartSimOne::WaitSimOneIsOk(true);
    SimOneSM::SetDriverName(0, "MicuksPreDeRo");

    //Wait for the Sim-One case to run
    while (1) {
        int frame = SimOneAPI::Wait();
        SimOneAPI::GetSimOneGps(&Gps);
        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && (Gps.timestamp > 0)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "SimOne Initialized");
            SimOneAPI::NextFrame(frame);
            break;
        }

        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "SimOne Initializing...");

    }

    //load Map Data
    int timeout = 20;
    bool slowDown = false;
    int obstacleFlag = 0;
    if (!SimOneSM::LoadHDMap(timeout)) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Failed to load map!");
        return 0;
    }

    SSD::SimPoint3DVector route;
    SSD::SimPoint3D pos(Gps.posX, Gps.posY, Gps.posZ);
    getTargetPath(pos, route);

    if (SimOneSM::GetWayPoints(pWaypoints.get()))
    {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "wayPointsSize:%d\n", pWaypoints->wayPointsSize);
        if ((pWaypoints->wayPointsSize) >= 2) {
            SSD::SimPoint3DVector route;
            if (SimOneSM::GetPredefinedRoute(route)) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "route path size:%d\n", route.size());
            }
        }
        else {
            SSD::SimPoint3D pos(Gps.posX, Gps.posY, Gps.posZ);
            SSD::SimString laneId = SampleGetNearMostLane(pos);
            HDMapStandalone::MLaneInfo info;
            if (SimOneSM::GetLaneSample(laneId, info)) {
                SSD::SimPoint3DVector route = info.centerLine;
            }

        }
    }
    else {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicle not set path !!!");
    }


    while (1)
    {
        //exit
        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) {
            break;
        }
        int frame = SimOneAPI::Wait();

        g_count++;
        if(g_count >= g_total)
            g_count = 0;

        if (!SimOneAPI::GetSimOneGps(&Gps)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Fetch GPS failed!!!");
        }
        //pWaypoints

        double mainVehicleSpeed = std::sqrt(std::pow(Gps.velX, 2) + std::pow(Gps.velY, 2) + std::pow(Gps.velZ, 2));
        static int lastTargetPathIndex = -1;
        double steering = calculateSteering(route, &Gps, lastTargetPathIndex);
        double targetSpeed = 8.33333f;
        if(!g_count)
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "steering: %f", steering);

        std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();
        pControl->timestamp = Gps.timestamp;
        pControl->gear = EGearMode_Drive;
        pControl->steeringMode = ESteeringMode_Percent;
        pControl->throttleMode = EThrottleMode_Percent;
        pControl->steering = steering;
        pControl->handbrake = false;
        pControl->isManualGear = false;
        if(mainVehicleSpeed < targetSpeed) {
            pControl->throttle = 1;
            pControl->brake = 0;
        }
        else {
            pControl->throttle = 0;
            pControl->brake = 1;
        }
        SimOneSM::SetDrive(0, pControl.get());

        SimOneAPI::NextFrame(frame);
    }
}
