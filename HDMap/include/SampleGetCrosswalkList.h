#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetCrosswalkList(const SimPoint3D& pos, const SimString& laneId, SSD::SimVector<HDMapStandalone::MSignal> TrafficLightList)
{
	if (TrafficLightList.size() < 1) 
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "No traffic light exists!");
		return;
	}
	SSD::SimVector<HDMapStandalone::MObject> crosswalkList;
	for (auto&item : TrafficLightList)
	{
		SimOneSM::GetCrosswalkList(item, laneId, crosswalkList);
		for (size_t i = 0; i < crosswalkList.size(); i++)
		{
			auto& objectItem = crosswalkList[i];
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.id: %ld", objectItem.id);
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.type: %s", objectItem.type.GetString());
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.pt.x: %f, objectItem.pt.y: %f, objectItem.pt.z: %f", objectItem.pt.x, objectItem.pt.y, objectItem.pt.z);
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "objectItem.boundaryKnots: %d", objectItem.boundaryKnots.size());
		}
	}
}