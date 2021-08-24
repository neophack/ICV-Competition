// ==========================================================================
// Copyright (C) 2018 - 2021 Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// , and/or its licensors.  All rights reserved.
//
// The coded instructions, statements, computer programs, and/or related 
// material (collectively the "Data") in these files contain unpublished
// information proprietary to Beijing 51WORLD Digital Twin Technology Co., Ltd. 
// ("51WORLD") and/or its licensors,  which is protected by the People's 
// Republic of China and/or other countries copyright law and by 
// international treaties.
//
// The Data may not be disclosed or distributed to third parties or be
// copied or duplicated, in whole or in part, without the prior written
// consent of 51WORLD.
//
// The copyright notices in the Software and this entire statement,
// including the above license grant, this restriction and the following
// disclaimer, must be included in all copies of the Software, in whole
// or in part, and all derivative works of the Software, unless such copies
// or derivative works are solely in the form of machine-executable object
// code generated by a source language processor.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
// 51WORLD DOES NOT MAKE AND HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED
// WARRANTIES INCLUDING, BUT NOT LIMITED TO, THE WARRANTIES OF
// NON-INFRINGEMENT, MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE,
// OR ARISING FROM A COURSE OF DEALING, USAGE, OR TRADE PRACTICE. IN NO
// EVENT WILL 51WORLD AND/OR ITS LICENSORS BE LIABLE FOR ANY LOST
// REVENUES, DATA, OR PROFITS, OR SPECIAL, DIRECT, INDIRECT, OR
// CONSEQUENTIAL DAMAGES, EVEN IF 51WORLD AND/OR ITS LICENSORS HAS
// BEEN ADVISED OF THE POSSIBILITY OR PROBABILITY OF SUCH DAMAGES.
// ==========================================================================
#pragma once

#include "public/libexport.h"
#ifdef STATIC_SSD
#include "SSD/public/SimVector.h"
#include "SSD/public/SimString.h"
#else
#include "SSD/SimVector.h"
#include "SSD/SimString.h"
#endif

namespace HDMapStandalone {
	struct LIBEXPORT MLaneLink
	{
		//@brief lane's predecessors' laneName list, formatted as roadId_sectionIndex_laneId.
		SSD::SimStringVector predecessorLaneNameList;
		//@brief lane's successors' laneName list, formatted as roadId_sectionIndex_laneId.
		SSD::SimStringVector successorLaneNameList;
		//@brief lane's left neighbor's laneName, formatted as roadId_sectionIndex_laneId.
		SSD::SimString leftNeighborLaneName;
		//@brief lane's right neighbor's laneName, formatted as roadId_sectionIndex_laneId.
		SSD::SimString rightNeighborLaneName;
	};
}