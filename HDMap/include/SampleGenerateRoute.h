#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

void SampleGenerateRoute()
{
	SSD::SimPoint3DVector route;
	SSD::SimPoint3DVector inputPoints;
	inputPoints.push_back(SSD::SimPoint3D(-135.477, 52.329, 0));
	inputPoints.push_back(SSD::SimPoint3D(-48.290, -18.109, 0));
	SSD::SimVector<int> indexOfValidPoints;
	if (!SimOneSM::GenerateRoute(inputPoints, indexOfValidPoints, route))
	{
		
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: route is not generated in the map.");
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "indexOfValidPoints:");
	
		for (auto& index : indexOfValidPoints)
		{
			SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, " %d,", index);
		}
		return;
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "route size: %d", route.size());

	for (unsigned int i = 0; i < route.size(); i++)
	{
		auto& knot = route[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "( %f, %f, %f),", knot.x, knot.y, knot.z);
	}
}