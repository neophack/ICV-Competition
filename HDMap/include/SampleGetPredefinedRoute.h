#pragma once
#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;
using SSD::SimPoint3D;

void SampleGetPredefinedRoute()
{
	SSD::SimPoint3DVector routeInfo;
	bool resule = SimOneSM::GetPredefinedRoute(routeInfo);
	if (!resule)
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Not exists!");
	} 
	for (size_t i = 0; i < routeInfo.size(); i++)
	{
		auto &item = routeInfo[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "[item.x: %f, item.y: %f, item.y: %f]", item.x, item.y, item.z);
	}
}