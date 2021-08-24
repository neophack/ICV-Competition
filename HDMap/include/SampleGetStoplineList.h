#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetStoplineList(const SimString& laneId, SSD::SimVector<HDMapStandalone::MSignal> TrafficLightList)
{
	if (TrafficLightList.size() < 1)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "No traffic light exists!");
		return;
	}
	SSD::SimVector<HDMapStandalone::MObject> stoplineList;
	for (auto&item : TrafficLightList)
	{
		SimOneSM::GetStoplineList(item, laneId, stoplineList);
		for (size_t i = 0; i < stoplineList.size(); i++)
		{
			auto& objectItem = stoplineList[i];
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.id: %ld", objectItem.id);
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.type: %s", objectItem.type.GetString());
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[pt.x: %f, pt.y: %f, pt.y: %f]", objectItem.pt.x, objectItem.pt.y, objectItem.pt.z);
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.boundaryKnots: %d", objectItem.boundaryKnots.size());
		}
	}
}