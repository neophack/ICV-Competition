#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimPoint3D;
using SSD::SimString;
using SSD::SimStringVector;

void SampleGetNearLanes(const SimPoint3D& pos, const double& distance)
{
	SimStringVector nearLanes;
	if (!SimOneSM::GetNearLanes(pos, distance, nearLanes))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "No lanes(lane) are(is) found");
		return;
	}
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lane id list:");
	for (unsigned int i = 0; i < nearLanes.size(); i++)
	{
		const SimString& laneId = nearLanes[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "%s,", laneId.GetString());
	}
}

