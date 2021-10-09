#pragma once

#include "SimOneNetAPI.h"
#include <iostream>

using SSD::SimString;

void SampleGetLaneType(const SimString& laneId)
{
	HDMapStandalone::MLaneType type;
	if (!SimOneSM::GetLaneType(laneId, type))
	{
		SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelWarning, "Error: lane does not eixst in the map");
		return;
	}
	std::string typeStr;
	switch (type)
	{
		case HDMapStandalone::MLaneType::none:
		{
			typeStr = "HDMapStandalone::MLaneType::none";
		}
		break;
		case HDMapStandalone::MLaneType::driving:
		{
			typeStr = "HDMapStandalone::MLaneType::driving";
		}
		break;
		case HDMapStandalone::MLaneType::stop:
		{
			typeStr = "HDMapStandalone::MLaneType::stop";
		}
		break;
		case HDMapStandalone::MLaneType::shoulder:
		{
			typeStr = "HDMapStandalone::MLaneType::shoulder";
		}
		break;
		case HDMapStandalone::MLaneType::biking:
		{
			typeStr = "HDMapStandalone::MLaneType::biking";
		}
		break;
		case HDMapStandalone::MLaneType::sidewalk:
		{
			typeStr = "HDMapStandalone::MLaneType::sidewalk";
		}
		break;
		case HDMapStandalone::MLaneType::border:
		{
			typeStr = "HDMapStandalone::MLaneType::border";
		}
		break;
		case HDMapStandalone::MLaneType::restricted:
		{
			typeStr = "HDMapStandalone::MLaneType::restricted";
		}
		break;
		case HDMapStandalone::MLaneType::parking:
		{
			typeStr = "HDMapStandalone::MLaneType::parking";
		}
		break;
		case HDMapStandalone::MLaneType::bidirectional:
		{
			typeStr = "HDMapStandalone::MLaneType::bidirectional";
		}
		break;
		case HDMapStandalone::MLaneType::median:
		{
			typeStr = "HDMapStandalone::MLaneType::median";
		}
		break;
		case HDMapStandalone::MLaneType::special1:
		{
			typeStr = "HDMapStandalone::MLaneType::special1";
		}
		break;
		case HDMapStandalone::MLaneType::special2:
		{
			typeStr = "HDMapStandalone::MLaneType::special2";
		}
		break;
		case HDMapStandalone::MLaneType::special3:
		{
			typeStr = "HDMapStandalone::MLaneType::special3";
		}
		break;
		case HDMapStandalone::MLaneType::roadWorks:
		{
			typeStr = "HDMapStandalone::MLaneType::roadWorks";
		}
		break;
		case HDMapStandalone::MLaneType::tram:
		{
			typeStr = "HDMapStandalone::MLaneType::tram";
		}
		break;
		case HDMapStandalone::MLaneType::rail:
		{
			typeStr = "HDMapStandalone::MLaneType::rail";
		}
		break;
		case HDMapStandalone::MLaneType::entry:
		{
			typeStr = "HDMapStandalone::MLaneType::entry";
		}
		break;
		case HDMapStandalone::MLaneType::exit:
		{
			typeStr = "HDMapStandalone::MLaneType::exit";
		}
		break;
		case HDMapStandalone::MLaneType::offRamp:
		{
			typeStr = "HDMapStandalone::MLaneType::offRamp";
		}
		break;
		case HDMapStandalone::MLaneType::onRamp:
		{
			typeStr = "HDMapStandalone::MLaneType::onRamp";
		}
		break;
		case HDMapStandalone::MLaneType::mwyEntry:
		{
			typeStr = "HDMapStandalone::MLaneType::mwyEntry";
		}
		break;
		case HDMapStandalone::MLaneType::mwyExit:
		{
			typeStr = "HDMapStandalone::MLaneType::mwyExit";
		}
		break;
		default:
			typeStr = "HDMapStandalone::MLaneType::none";
	}
	SimOneAPI::bridgeLogOutput(ELogLevel_Type::ELogLevelDebug, "lane type: %s", typeStr.c_str());
}
