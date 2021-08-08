import pySimOneIO
import math
from SimOneIOStruct import *

case_info = SimOne_Data_CaseInfo()
SimOne_Data_Gps_Test = SimOne_Data_Gps()
SimOne_Data_MainVehicle_Info_Test = SimOne_Data_MainVehicle_Info()
SimOne_Data_MainVehicle_Status_Test = SimOne_Data_MainVehicle_Status()
control = SimOne_Data_Control()
wayPoints = SimOne_Data_WayPoints()

SimOne_Data_Obstacle_Test = SimOne_Data_Obstacle()
M_PI = 3.14159265358979323846


def SampleGetNearMostLane(pos):

    # SoBridgeLogOutput(0, "SampleGetNearMostLane:")

    info = pySimOneIO.getNearMostLane(pos)
    if info.exists == False:

        SoBridgeLogOutput(0, "Not exists!")

        return

    # SoBridgeLogOutput(0, "lane id:%s" % info.laneId.GetString())

    return info.laneId


def apiAllStart(isJoinTimeLoop):
    SoAPIStartSimOneNode(0, 0)
    SoAPISimOneNodeReady()
    if SoAPIGetCaseInfo(case_info):
        SoBridgeLogOutput(0, "GetCaseInfo caseName: %s" % case_info.caseName)
        SoBridgeLogOutput(0, "GetCaseInfo caseId: %s" % case_info.caseId)
        SoBridgeLogOutput(0, "GetCaseInfo taskId: %s" % case_info.taskId)
        SoBridgeLogOutput(0, "GetCaseInfo sessionId: %s" % case_info.sessionId)

    SoBridgeLogOutput(0, "SoAPIGetCaseRunStatus: %s" % SoAPIGetCaseRunStatus())

    if SoAPIGetMainVehicleList(SimOne_Data_MainVehicle_Info_Test):

        print("MainVehicle size: %s" % SimOne_Data_MainVehicle_Info_Test.size)

    while 1:
        SoAPISubMainVehicle_result = SoAPISubMainVehicle(0, isJoinTimeLoop)

        SoBridgeLogOutput(
            0, "SoAPISubMainVehicle_result:%s" % SoAPISubMainVehicle_result
        )
        if SoAPISubMainVehicle_result:
            break

    if SoAPIGetMainVehicleStatus(SimOne_Data_MainVehicle_Status_Test):

        SoBridgeLogOutput(
            0, "mainVehicleId:%s" % SimOne_Data_MainVehicle_Status_Test.mainVehicleId
        )
        SoBridgeLogOutput(
            0,
            "mainVehicleStatus:%s"
            % SimOne_Data_MainVehicle_Status_Test.mainVehicleStatus,
        )

    ret = pySimOneIO.loadHDMap(100)

    SoBridgeLogOutput(0, "Load xodr success:%s" % ret)


def calculateSpeed(velX, velY, velZ):
    return math.sqrt(pow(velX, 2) + pow(velY, 2) + pow(velZ, 2))


def planarDistance(pt1, pt2):
    return math.sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2))


def SampleGetLaneST(laneId, pos):

    # SoBridgeLogOutput(0, "SampleGetLaneST:")

    stInfo = pySimOneIO.getLaneST(laneId, pos)
    if stInfo.exists == False:

        SoBridgeLogOutput(2, "Not exists!")

        return

    # SoBridgeLogOutput(0, "[%s,%s] relative to this lane:" % (stInfo.s, stInfo.t))

    return stInfo.s, stInfo.t


def DEBUG():
    SoBridgeLogOutput(
        0,
        "Gps data: pos=(%.1f, %.1f, %.1f), vel=(%.1f, %.1f, %.1f)"
        % (
            SimOne_Data_Gps_Test.posX,
            SimOne_Data_Gps_Test.posY,
            SimOne_Data_Gps_Test.posZ,
            SimOne_Data_Gps_Test.velX,
            SimOne_Data_Gps_Test.velY,
            SimOne_Data_Gps_Test.velZ,
        ),
    )
    SoBridgeLogOutput(0, "ObstacleSize: %d" % SimOne_Data_Obstacle_Test.obstacleSize)
    for i in range(SimOne_Data_Obstacle_Test.obstacleSize):
        SoBridgeLogOutput(
            0,
            "Obstacle %d: type %d, pos=(%.1f, %.1f, %.1f), vel=(%.1f, %.1f, %.1f)"
            % (
                SimOne_Data_Obstacle_Test.obstacle[i].id,
                SimOne_Data_Obstacle_Test.obstacle[i].type.value,
                SimOne_Data_Obstacle_Test.obstacle[i].posX,
                SimOne_Data_Obstacle_Test.obstacle[i].posY,
                SimOne_Data_Obstacle_Test.obstacle[i].posZ,
                SimOne_Data_Obstacle_Test.obstacle[i].velX,
                SimOne_Data_Obstacle_Test.obstacle[i].velY,
                SimOne_Data_Obstacle_Test.obstacle[i].velZ,
            ),
        )


if __name__ == '__main__':

    inAEBState = False
    isSimOneInitialized = False
    apiAllStart(True)
    SoSetDriverName(0, "yrAEB")

    while 1:
        if SoAPIGetCaseRunStatus() == 1:
            SoBridgeLogOutput(0, "case stop")
            break

        frame = SoAPIWait()

        if not SoAPIGetSimOneGps(SimOne_Data_Gps_Test):
            SoBridgeLogOutput(2, "Fetch GPS failed")
        if not SoAPIGetSimOneGroundTruth(SimOne_Data_Obstacle_Test):
            SoBridgeLogOutput(2, "Fetch GroundTruth failed")

        DEBUG()

        mainVehiclePos = pySimOneIO.pySimPoint3D(
            SimOne_Data_Gps_Test.posX,
            SimOne_Data_Gps_Test.posY,
            SimOne_Data_Gps_Test.posZ,
        )
        mainVehicleSpeed = calculateSpeed(
            SimOne_Data_Gps_Test.velX,
            SimOne_Data_Gps_Test.velY,
            SimOne_Data_Gps_Test.velZ,
        )
        minDistance = 10000000
        potentialObstacleIndex = SimOne_Data_Obstacle_Test.obstacleSize
        mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos)
        potentialObstacleLaneId = ""

        for i in range(0, SimOne_Data_Obstacle_Test.obstacleSize):
            obstaclePos = pySimOneIO.pySimPoint3D(
                SimOne_Data_Obstacle_Test.obstacle[i].posX,
                SimOne_Data_Obstacle_Test.obstacle[i].posY,
                SimOne_Data_Obstacle_Test.obstacle[i].posZ,
            )
            obstacleLaneId = SampleGetNearMostLane(obstaclePos)
            if mainVehicleLaneId.GetString() == obstacleLaneId.GetString():
                obstacleDistance = planarDistance(mainVehiclePos, obstaclePos)
                SoBridgeLogOutput(0, "obstacleDistance: %.1f" % obstacleDistance)
                if obstacleDistance < minDistance:
                    minDistance = obstacleDistance
                    potentialObstacleIndex = i
                    potentialObstacleLaneId = obstacleLaneId
        SoBridgeLogOutput(0, "potentialObstacleIndex: %s" % potentialObstacleIndex)
        potentialObstacle = SimOne_Data_Obstacle_Test.obstacle[potentialObstacleIndex]
        obstacleSpeed = calculateSpeed(
            potentialObstacle.velX, potentialObstacle.velY, potentialObstacle.velZ
        )

        potentialObstaclePos = pySimOneIO.pySimPoint3D(
            potentialObstacle.posX, potentialObstacle.posY, potentialObstacle.posZ
        )

        SoBridgeLogOutput(
            0,
            "potentialObstacle: pos=(%.1f, %.1f, %.1f), vel=%.1f"
            % (
                potentialObstacle.posX,
                potentialObstacle.posY,
                potentialObstacle.posZ,
                obstacleSpeed,
            ),
        )

        sObstalce = 0
        tObstacle = 0
        sMainVehicle = 0
        tMainVehicle = 0
        isObstalceBehind = False

        if potentialObstacleLaneId:
            sObstalce, tObstacle = SampleGetLaneST(
                potentialObstacleLaneId, potentialObstaclePos
            )
            sMainVehicle, tMainVehicle = SampleGetLaneST(
                potentialObstacleLaneId, mainVehiclePos
            )
            isObstalceBehind = False if sMainVehicle >= sObstalce else True

        SoBridgeLogOutput(
            0,
            "stObstacle: (%.1f, %.1f), stMainVehicle: (%.1f, %.1f)"
            % (sObstalce, tObstacle, sMainVehicle, tMainVehicle),
        )

        flag=True
        if not SoGetDriverControl(0, control):
            flag=False
            SoBridgeLogOutput(2, "GetDriverControl Failed")

        if isObstalceBehind:
            defaultDistance = 6
            defautlTimeToCollision = 3.4
            timeToCollision = (minDistance - defaultDistance) / (
                mainVehicleSpeed - obstacleSpeed
            )

            SoBridgeLogOutput(0, "timeToCollision: %.1f" % timeToCollision)

            if timeToCollision < defautlTimeToCollision and timeToCollision > 0:
                inAEBState = True
                control.brake = mainVehicleSpeed * 3.6 * 0.65 + 0.20

            if inAEBState:
                control.throttle = 0

        if flag:
            if not SoApiSetDrive(0, control):
                SoBridgeLogOutput(2, "SetDrive Failed")
            else:
                SoBridgeLogOutput(0,"SetDrive Successfully")

        SoBridgeLogOutput(0, "-" * 20 + "END OF FRAME" + "-" * 20)
        SoAPINextFrame(frame)
