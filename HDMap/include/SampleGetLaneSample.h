#pragma once

#include "SimOneNetAPI.h"
#include "public/common/MLaneInfo.h"
#include <iostream>

using namespace HDMapStandalone;
using SSD::SimPoint3D;
using SSD::SimString;

void SampleGetLaneSample(const SimString& laneId)
{
	MLaneInfo info;
	if (!SimOneSM::GetLaneSample(laneId, info))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane does not eixst in the map.");
		return;
	}
		
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left boundary knot size: %d", info.leftBoundary.size());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left boundary knot list:");

	for (unsigned int i = 0; i < info.leftBoundary.size(); i++)
	{
		const SimPoint3D& knot = info.leftBoundary[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "( %f, %f, %f),", knot.x, knot.y, knot.z);
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "right boundary knot size: %d", info.rightBoundary.size());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "right boundary knot list:");

	for (unsigned int i = 0; i < info.rightBoundary.size(); i++)
	{
		const SimPoint3D& knot = info.rightBoundary[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "( %f, %f, %f),", knot.x, knot.y, knot.z);
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "center line knot size: %d", info.centerLine.size());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "center line knot list:");

	for (unsigned int i = 0; i < info.centerLine.size(); i++)
	{
		const SimPoint3D& knot = info.centerLine[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "( %f, %f, %f),", knot.x, knot.y, knot.z);
	}
}