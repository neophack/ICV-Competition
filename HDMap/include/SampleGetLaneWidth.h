#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetLaneWidth(const SimString& laneId, const SimPoint3D& pos)
{
	double width;
	if (!SimOneSM::GetLaneWidth(laneId, pos, width))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane does not eixst in the map.");
		return;
	}
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lane width at this location: (%f, %f, %f)", pos.x, pos.y, pos.z);
}