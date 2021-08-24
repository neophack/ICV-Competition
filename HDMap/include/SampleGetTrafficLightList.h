#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;

SSD::SimVector<HDMapStandalone::MSignal> SampleGetTrafficLightList()
{
	SSD::SimVector<HDMapStandalone::MSignal> TrafficLightList;
	SimOneSM::GetTrafficLightList(TrafficLightList);
	if (TrafficLightList.size() < 1)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "No traffic light exists!");
		return TrafficLightList;
	}
	
	for (auto& item : TrafficLightList)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "sign id: %ld", item.id);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "type: %s", item.type.GetString());
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "subType: %s", item.subType.GetString());
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "item.pt.x: %f, item.pt.y: %f, item.pt.z: %f", item.pt.x, item.pt.y, item.pt.z);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "heading.pt.x: %f, heading.pt.y: %f, heading.pt.z: %f,", item.heading.x, item.heading.y, item.heading.z);
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "value: %s", item.value.GetString());
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "unit: %s", item.unit.GetString());
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "isDynamic: %d", item.isDynamic);
	}
	return TrafficLightList;
}