#pragma once

#include "SimOneNetAPI.h"
#include "public/common/MLaneLink.h"
#include <iostream>

using SSD::SimString;

void SampleGetLaneLink(const SimString& laneId)
{
	MLaneLink link;
	if (!SimOneSM::GetLaneLink(laneId, link))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane does not eixst in the map");
		return;
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "predecessor lane Id list:");
	for (unsigned int i = 0; i < link.predecessorLaneNameList.size(); i++)
	{
		const SimString& laneId = link.predecessorLaneNameList[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, " %s,", laneId.GetString());
	}
		
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "successor lane Id list:");
	for (unsigned int i = 0; i < link.successorLaneNameList.size(); i++)
	{
		const SimString& laneId = link.successorLaneNameList[i];
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, " %s,", laneId.GetString());
	}

	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "left neighbor Id: %s", link.leftNeighborLaneName.GetString());
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "right neighbor Id: %s", link.rightNeighborLaneName.GetString());
}