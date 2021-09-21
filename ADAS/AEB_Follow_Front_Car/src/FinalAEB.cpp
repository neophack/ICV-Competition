#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "utilStartSimOneNode.h"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "../HDMap/include/SampleGetLaneST.h"

#include <memory>
#include <limits>
#include <iostream>

//Main function
//
int main()
{

    int count = 0, totalcount = 5;
    double targetSpeed = 8.333f;
    bool inAEBState = false;
    bool isSimOneInitialized = false;
    StartSimOne::WaitSimOneIsOk(true);
    SimOneSM::SetDriverName(0, "AEB");

    int timeout = 20;
    while (true) {
        if (SimOneSM::LoadHDMap(timeout)) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loaded");
            break;
        }
        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "HDMap Information Loading...");
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

        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && pObstacle->timestamp > 0 && pGps->timestamp > 0) {
            if (!isSimOneInitialized) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initialized!");
                isSimOneInitialized = true;
            }
            count++;
            if(count >= totalcount)
                count = 0;

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
                    //double obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);
                    double obstacleDistance = std::abs(mainVehiclePos.x - obstaclePos.x);

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
            double sObstacle = 0;
            double tObstacle = 0;

            double sMainVehicle = 0;
            double tMainVehicle = 0;

            bool isObstacleFront = false;
            if (!potentialObstacleLaneId.Empty()) {

                SampleGetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
                SampleGetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);

                if(!count)
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "sMainVehicle = %f, sObstacle = %f", sMainVehicle, sObstacle);

                isObstacleFront = !(sMainVehicle >= sObstacle);
            }

            std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

            // Control mainVehicle with SimOneDriver
            //SimOneSM::GetDriverControl(0, pControl.get());

            // Control mainVehicle without SimOneDriver
            pControl->throttleMode = EThrottleMode_Percent;
            if(mainVehicleSpeed > targetSpeed) {
                pControl->throttle = 0;
                pControl->brake = 1;
            }
            else {
                pControl->throttle = 1;
                pControl->brake = 0;
            }
            pControl->steering = 0;
            pControl->handbrake = 0;
            pControl->isManualGear = 0;
            pControl->gear = EGearMode_Drive;

            if (isObstacleFront) {
                //EGear Mode

                double defaultDistance = 5.2f + 1.1f;

                double timeToCollision = std::abs((minDistance - defaultDistance) / (obstacleSpeed - mainVehicleSpeed));
                if(!count)
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "TimeToCollision: %f", timeToCollision);
                double defaultTimeToCollision = 1.4f;

                if (timeToCollision < defaultTimeToCollision && timeToCollision > 0) {
                    inAEBState = true;
                }
                if (inAEBState) {
                    if(!count)
                        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "--- In AEB State ---");
                    pControl->gear = EGearMode_Drive;
                    pControl->throttleMode = EThrottleMode_Accel;
                    pControl->isManualGear = 0;
                    double accel = std::pow((mainVehicleSpeed - obstacleSpeed), 2) / (2 * (minDistance - defaultDistance));
                    pControl->throttle = -accel;
                    if(!count) {
                        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Acceleration: %f,calculatedAccel: %f, distance: %f", pGps->accelX, accel, std::abs(minDistance));
                    }
                }
                if(inAEBState && timeToCollision > defaultDistance && (obstacleSpeed - mainVehicleSpeed) > 1.0f) {
                    inAEBState = false;
                }
            }	
            if(!count)
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "obstacleSpeed = %f, mainVehicleSpeed = %f", obstacleSpeed, mainVehicleSpeed);
            SimOneSM::SetDrive(0, pControl.get());
        }
        else {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
        }

        SimOneAPI::NextFrame(frame);
    }
    return 0;
}
