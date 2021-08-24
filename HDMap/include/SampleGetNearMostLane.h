#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimPoint3D;
using SSD::SimString;

SimString SampleGetNearMostLane(const SimPoint3D& pos)
{
	SimString laneId;
	double s, t, s_toCenterLine, t_toCenterLine;
	if (!SimOneSM::GetNearMostLane(pos, laneId, s, t, s_toCenterLine, t_toCenterLine))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane is not found.");
		return laneId;
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lane id: %s", laneId.GetString());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[s: %f, t: %f]", s,t);
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[s_toCenterLine: %f, t_toCenterLine: %f]", s_toCenterLine, t_toCenterLine);
	return laneId;
}

