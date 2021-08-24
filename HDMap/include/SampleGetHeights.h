#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetHeights(const SSD::SimPoint3D& inputPt, double radius)
{
	SSD::SimVector<double> heights;
	SSD::SimVector<long> roadIds;
	SSD::SimVector<bool> insideRoadStates;
	bool result = SimOneSM::GetHeights(inputPt, radius, heights, roadIds, insideRoadStates);
	if (!result)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "not exist");
		return;
	}
	for (auto&item : heights)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "height: %f", item);
	}
	for (auto&item : roadIds)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "roadId: %ld", item);
	}
	for (auto&item : insideRoadStates)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "insideRoadState: %d", item);
	}
}
