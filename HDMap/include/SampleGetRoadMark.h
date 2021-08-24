#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetRoadMark(const SimString& laneId, const SimPoint3D& pos)
{
	HDMapStandalone::MRoadMark left;
	HDMapStandalone::MRoadMark right;
	bool result = SimOneSM::GetRoadMark(pos, laneId, left, right);
	if (result)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left.sOffset: %f", left.sOffset);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left.width: %f", left.width);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left.type: %d", (int)left.type);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left.color: %d", (int)left.color);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "right.sOffset: %f", right.sOffset);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "right.width: %f", right.width);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left.type: %d", (int)left.type);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left.color: %d", (int)left.color);	
	}
	else
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Not exists!");
		return;
	}

}
