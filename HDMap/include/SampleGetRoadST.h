#pragma once

#include "SimOneNetAPI.h"
#include <iostream>


using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetRoadST(const SimString& laneId, const SimPoint3D& pos)
{
	double s, t, z;
	if (!SimOneSM::GetRoadST(laneId, pos, s, t, z))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane does not eixst in the map");
		return;
	}
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "relative to this lane's owner road, this location: (%f, %f, %f), s-t coordidate position: [s: %f,t: %f]", pos.x, pos.y, pos.z, s, t);
}