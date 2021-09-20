#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "utilStartSimOneNode.h"
#include "../HDMap/include/SampleGetNearMostLane.h"
#include "../HDMap/include/SampleGetLaneST.h"
#include "myfunctions.h"

#include <memory> 
#include <limits>
#include <iostream>

//Main function
//
int main()
{
    int __my_count = 0, _total_cnt = 10;
    bool inAEBState = false;
    bool inStopLineState = false;
    bool isSimOneInitialized = false;
    StartSimOne::WaitSimOneIsOk(true);
    SimOneSM::SetDriverName(0, "Link_Avoidance");

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

        SSD::SimVector<HDMapStandalone::MSignal> SignalLightList;
        SimOneSM::GetTrafficLightList(SignalLightList);
        if(SignalLightList.size() < 1) {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Fetch SM::TrafficLight Failed.");
        }

        if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running && pObstacle->timestamp > 0 && pGps->timestamp > 0) {
            if (!isSimOneInitialized) {
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initialized!");
                isSimOneInitialized = true;
            }

            __my_count++;
            if(__my_count >= _total_cnt) __my_count = 0;

            SSD::SimPoint3D mainVehiclePos(pGps->posX, pGps->posY, pGps->posZ);
            double mainVehicleSpeed = UtilMath::calculateSpeed(pGps->velX, pGps->velY, pGps->velZ);

            double minDistance = std::numeric_limits<double>::max();
            double minStopLineDistance = std::numeric_limits<double>::max();
            int potentialObstacleIndex = pObstacle->obstacleSize;
            int minStopLineIndex;
            SSD::SimString mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);

            double StopLineDistance;

            double sMainVehicle;
            double tMainVehicle;
            
            double sStopLine;
            double tStopLine;
            SSD::SimString StopLineLaneId;

            bool isSignalLightRed = false;
            bool isStopLineFront = false;
            
            SimOneSM::GetLaneST(mainVehicleLaneId, mainVehiclePos, sMainVehicle, tMainVehicle);
            SSD::SimVector<HDMapStandalone::MObject> StopLineList;
            for(int i = 0; i < SignalLightList.size(); i++) {
                SimOneSM::GetStoplineList(SignalLightList[i], mainVehicleLaneId, StopLineList);
                if (StopLineList.size() >= 1) {
                    for(int j = 0; j < StopLineList.size(); j++) {
                        auto& thisStopLine = StopLineList[j];
                        double tmppos[3];
                        tmppos[0] = (thisStopLine.boundaryKnots[0].x + thisStopLine.boundaryKnots[1].x)/2;
                        tmppos[1] = (thisStopLine.boundaryKnots[0].y + thisStopLine.boundaryKnots[1].y)/2;
                        tmppos[2] = (thisStopLine.boundaryKnots[0].z + thisStopLine.boundaryKnots[1].z)/2;
                        SSD::SimPoint3D StopLinePos(tmppos[0], tmppos[1], tmppos[2]);
                        StopLineDistance = UtilMath::planarDistance(mainVehiclePos, StopLinePos);
                        if(StopLineDistance < minStopLineDistance) {
                            isSignalLightRed = false;
                            minStopLineDistance = StopLineDistance;
                            SSD::SimPoint3D minStopLinePos(tmppos[0], tmppos[1], tmppos[2]);
                            SSD::SimPoint3D minStopLinePTPos(thisStopLine.pt.x, thisStopLine.pt.y, thisStopLine.pt.z);
                            StopLineLaneId = SampleGetNearMostLane(minStopLinePTPos);
                            SimOneSM::GetLaneST(StopLineLaneId, minStopLinePTPos, sStopLine, tStopLine);
                            isStopLineFront = !(sMainVehicle >= sStopLine);
                            if(!__my_count)
                                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "sMainVehicle = %f, sStopLine = %f, isStopLineFront = %d", sMainVehicle, sStopLine, isStopLineFront);
                            if(!__my_count) {
                                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "minStopLine(%f, %f, %f): %f", minStopLinePos.x, minStopLinePos.y, minStopLinePos.z, minStopLineDistance);
                                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "mainVehicle(%f, %f, %f): %f", mainVehiclePos.x, mainVehiclePos.y, mainVehiclePos.z);
                            }
                        }
                    }
                }
            }

            SSD::SimString potentialObstacleLaneId = "";
            double safeDistance = 10.0f;
            double goAwayDistance = 30.0f;
            double obstacleDistance;
            for (size_t i = 0; i < pObstacle->obstacleSize; ++i) {
                SSD::SimPoint3D obstaclePos(pObstacle->obstacle[i].posX, pObstacle->obstacle[i].posY, pObstacle->obstacle[i].posZ);
                SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
                obstacleDistance = UtilMath::planarDistance(mainVehiclePos, obstaclePos);
                if (pObstacle->obstacle[i].type == 6 && obstacleDistance <= goAwayDistance) {
                    if(!__my_count)
                        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "toCarDistance = %f", obstacleDistance);
                    if (obstacleDistance < minDistance) {
                        minDistance = obstacleDistance; potentialObstacleIndex = (int)i;
                        potentialObstacleLaneId = obstacleLaneId;
                    }
                }
            }

            auto& potentialObstacle = pObstacle->obstacle[potentialObstacleIndex];
            double obstacleSpeed = UtilMath::calculateSpeed(potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);

            if(!__my_count)
                SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "obstacleSpeedX = %f, SpeedY = %f, SpeedZ = %f", potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ);

            SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ);
            double sObstacle = 0;
            double tObstacle = 0;

            bool isObstacleFront = false;
            bool isObstacleGoAway = true;
            if (!potentialObstacleLaneId.Empty()) {
                SimOneSM::GetLaneST(potentialObstacleLaneId, potentialObstaclePos, sObstacle, tObstacle);
                //if(!__my_count) {
                //    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "ObstacleLaneId = %s", potentialObstacleLaneId.GetString());
                //    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "sMainVehicle = %f, tMainVehicle = %f", sMainVehicle, tMainVehicle);
                //    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "sObstacle = %f, tObstacle = %f", sObstacle, tObstacle);
                //}
                isObstacleFront = !(sMainVehicle >= sObstacle);
                isObstacleGoAway = (obstacleDistance >= goAwayDistance);
            }
            if(isObstacleGoAway) {
                if(!__my_count)
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Obstacle has gone");
                inAEBState = false;
                inStopLineState = false;
            }

            std::unique_ptr<SimOne_Data_Control> pControl = std::make_unique<SimOne_Data_Control>();

            // Control mainVehicle with SimOneDriver
            //SimOneSM::GetDriverControl(0, pControl.get());

            // Control mainVehicle without SimOneDriver
            pControl->steering = 0;
            pControl->handbrake = 0;

            if(isStopLineFront && !inAEBState) {
                if((isSignalLightRed) || (!isObstacleGoAway )) {
                    //EGear Mode
                    double defaultDistance = 1.1f + 5.0f;
                    double timeToCollision = std::abs((minStopLineDistance - defaultDistance) / (mainVehicleSpeed));
                    if(!__my_count)
                        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "TimeToCollision: %f", timeToCollision);
                    double defaultTimeToCollision = 2.4f;

                    if (timeToCollision < defaultTimeToCollision && timeToCollision > 0) {
                        inStopLineState = true;
                    }
                    else {
                        inStopLineState = false;
                    }
                    if (inStopLineState) {
                        if(!__my_count)
                            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "--- To Stop at StopLine ---");
                        pControl->gear = EGearMode_Drive;
                        pControl->throttleMode = EThrottleMode_Accel;
                        pControl->isManualGear = 0;
                        double accel = std::pow((mainVehicleSpeed), 2) / (2 * (minStopLineDistance - defaultDistance));
                        pControl->throttle = accel;
                        if(!__my_count)
                            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Acceleration: %f,calculatedAccel: %f, distance: %f", pGps->accelX, accel, std::abs(minStopLineDistance));
                    }
                    if(inStopLineState && (isObstacleGoAway)) {
                        inStopLineState = false;
                    }
                }
            }

            if (isObstacleFront ) {
                //EGear Mode
                double defaultDistance = 4.2f + 1.1f;

                double timeToCollision = std::abs((minDistance - defaultDistance) / (obstacleSpeed - mainVehicleSpeed));
                if(!__my_count)
                    SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "TimeToCollision: %f", timeToCollision);
                double defaultTimeToCollision = 1.4f;

                if (timeToCollision < defaultTimeToCollision && timeToCollision > 0) {
                    inAEBState = true;
                }
                if (inAEBState) {
                    if(!__my_count)
                        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "--- In AEB State ---");
                    pControl->gear = EGearMode_Drive;
                    pControl->throttleMode = EThrottleMode_Accel;
                    pControl->isManualGear = 0;
                    double accel = std::pow((mainVehicleSpeed - obstacleSpeed), 2) / (2 * (minDistance - defaultDistance));
                    pControl->throttle = -accel;
                    if(!__my_count)
                        SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "Acceleration: %f,calculatedAccel: %f, distance: %f", pGps->accelX, accel, std::abs(minDistance));
                }
                if(inAEBState && timeToCollision > defaultDistance) {
                    inAEBState = false;
                }
            }	
            else if(inAEBState) {
                inAEBState = false;
            }

            if(!inAEBState && !inStopLineState) {
                //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "--- In Normal State ---");
                double targetSpeed = 11.11f;
                pControl->gear = EGearMode_Drive;
                pControl->throttleMode = EThrottleMode_Percent;
                pControl->brakeMode = EBrakeMode_Percent;
                pControl->isManualGear = 0;
                double diffSpeed = targetSpeed - mainVehicleSpeed;
                if(std::abs(diffSpeed) >= 2.78f) {
                    pControl->throttle = 1;
                    pControl->brake = 0;
                }
                else {
                    pControl->throttle = 0;
                    pControl->brake = 1;
                }
                //SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "speed = %f", mainVehicleSpeed);
            }

            SimOneSM::SetDrive(0, pControl.get());
        }
        else {
            SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelInformation, "SimOne Initializing...");
        }

        SimOneAPI::NextFrame(frame);
    }
    return 0;
}
