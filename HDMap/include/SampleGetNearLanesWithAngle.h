#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimPoint3D;
using SSD::SimString;
using SSD::SimStringVector;

void SampleGetNearLanesWithAngle(const SimPoint3D& pos,
	const double& distance,
	const double& headingAngle,
	const double& shiftAngle)
{
	SimStringVector nearLanes;
	if (!SimOneSM::GetNearLanesWithAngle(pos, distance, headingAngle, shiftAngle, nearLanes))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "No lanes(lane) are(is) found.");
		return;
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "nearLanes size: %d", nearLanes.size());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lane id list:");

	for (unsigned int i = 0; i < nearLanes.size(); i++)
	{
		const SimString& laneId = nearLanes[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "%s,", laneId.GetString());
	}
}

