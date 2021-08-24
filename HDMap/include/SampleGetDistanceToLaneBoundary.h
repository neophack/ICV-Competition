#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimPoint3D;
using SSD::SimString;

void SampleGetDistanceToLaneBoundary(const SimPoint3D& pos)
{
	SimString laneId;
	double distToLeft, distToRight, distToLeft2D, distToRight2D;
	if (!SimOneSM::GetDistanceToLaneBoundary(pos, laneId, distToLeft, distToRight, distToLeft2D, distToRight2D))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane is not found.");
		return;
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lane id: %s", laneId.GetString());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[distToLeft, distToRight]: %f, %f", distToLeft, distToRight);
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[distToLeft2D, distToRight2D]: %f, %f", distToLeft2D, distToRight2D);
}