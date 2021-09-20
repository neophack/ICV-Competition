
#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilDriver.h"
#include "utilStartSimOneNode.h"
#include "util/UtilMath.h"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "../HDMap/include/SampleGetLaneST.h"

#include <iostream>
#include <memory>

//Main function
//
int main()
{
    int count = 0, total = 10;
    bool inAEBState = false;
    int timeout = 20;
    bool isSimOneInitialized = false;
    StartSimOne::WaitSimOneIsOk(true);
    SimOneSM::SetDriverName(0, "LKA");
    while (true) {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "LKA Started...");
        if (SimOneSM::LoadHDMap(timeout)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loaded");
            break;
        }
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loading...");
    }

    SSD::SimPoint3DVector inputPoints;
    std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
    if (SimOneSM::GetWayPoints(pWayPoints.get()))
    {
        for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i) {
            SSD::SimPoint3D inputWayPoints(pWayPoints->wayPoints[i].posX, pWayPoints->wayPoints[i].posY, 0);
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Point[%d]: (%f, %f)", pWayPoints->wayPoints[i].x, pWayPoints->wayPoints[i].y);
            inputPoints.push_back(inputWayPoints);
        }
    }
    else {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Get mainVehicle wayPoints failed");
        return -1;
    }

    SSD::SimPoint3DVector targetPath;
    if (pWayPoints->wayPointsSize >= 2) {
        SSD::SimVector<int> indexOfValidPoints;
        if (!SimOneSM::GenerateRoute(inputPoints, indexOfValidPoints, targetPath)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Generate mainVehicle route failed");
            return -1;
        }
    }
    else if (pWayPoints->wayPointsSize == 1) {
        SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
        HDMapStandalone::MLaneInfo laneInfoInit;
        if (!SimOneSM::GetLaneSample(laneIdInit, laneInfoInit)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Generate mainVehicle initial route failed");
            return -1;
        }
        else {
            targetPath = laneInfoInit.centerLine;
        }
    }
    else {
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelError, "Number of wayPoints is zero");
        return -1;
    }

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

        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && pGps->timestamp > 0 && pObstacle->timestamp > 0) {
            if (!isSimOneInitialized) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initialized!");
                isSimOneInitialized = true;
            }

            SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
            double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

            double minDistance = std::numeric_limits<double>::max();
            int potentialObstacleIndex = pObstacle->obstacleSize;
            SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
            SSD::SimString potentialObstacleLaneId = "";
            for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
                SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
                SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
                if (mainVehicleLaneId == obstacleLaneId) {
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


            SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
            double sObstalce = 0.;
            double tObstacle = 0.;

            double sMainVehicle = 0.;
            double tMainVehicle = 0.;

            bool isObstalceBehind = false;
            if (!potentialObstacleLaneId.Empty()) {

                SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstalce, tObstacle);
                SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

                isObstalceBehind = !(sMainVehicle >= sObstalce);
            }

            std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

            // Control mainVehicle with SimOneDriver
            //SimOneSM::GetDriverControl(0, pControl.get());

            // Control mainVehicle without SimOneDriver
            pControl->throttle = 0.12f;
            pControl->brake = 0.f;
            pControl->steering = 0.f;
            pControl->handbrake = false;
            pControl->isManualGear = false;
            pControl->gear = static_cast<EGearMode>(1);

            if (isObstalceBehind) {
                double defaultDistance = 10.;
                double timeToCollision = std::abs((minDistance - defaultDistance)) / (obstacleSpeed - mainVehicleSpeed);
                double defautlTimeToCollision = 3.4;
                if (-timeToCollision < defautlTimeToCollision && timeToCollision < 0) {
                    inAEBState = true;
                    pControl->brake = (float)(mainVehicleSpeed * 3.6 * 0.65 + 0.20);
                }

                if (inAEBState) {
                    pControl->throttle = 0.f;
                }
            }
            double steering = UtilDriver::calculateSteering(targetPath, pGps.get());
            pControl->steering = (float)steering;
            SimOneSM::SetDrive(0, pControl.get());
        }
        else {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
        }

        SimOneAPI::NextFrame(frame);
    }
    return 0;
}
