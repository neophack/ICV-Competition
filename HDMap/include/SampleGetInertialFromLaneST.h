#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetInertialFromLaneST(const SimString& laneId, const double& s, const double& t)
{
	SSD::SimPoint3D inertial;
	SSD::SimPoint3D dir;
	bool result = SimOneSM::GetInertialFromLaneST(laneId, s, t, inertial, dir);
	if (!result)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Not exists!");
		return;
	}
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[inertial.x: %f, inertial.y: %f, inertial.z: %f]", inertial.x, inertial.y, inertial.z);
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[dir.x: %f, dir.y: %f, dir.z: %f]", inertial.x, inertial.y, inertial.z);
}
