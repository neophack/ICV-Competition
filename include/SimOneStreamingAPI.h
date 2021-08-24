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


#if defined(WIN32) || defined(_WIN32)
#define SIMONE_NET_API __declspec(dllexport)
#elif defined(__linux__) || defined(__linux)
#define SIMONE_NET_API __attribute__((visibility("default")))
#endif
#include <string>
#include "SimOneIOStruct.h"

#ifdef __cplusplus
extern "C"
{
#endif
	namespace SimOneSM {
		
		/*!
		\li function:
		*	GetStreamingImage
		\li brief:
		*	Get Streaming Image Data
		@param
		*	ip: UDP Streaming Image ip
		@param
		*   port: UDP Streaming Image port
		@param
		*	pImage: Image data(output)
		@return
		*	Success or not
		*/
		SIMONE_NET_API bool GetStreamingImage(const char *ip, unsigned short port, SimOne_Data_Image *pImage);
		/*!
		\li function:
		*	SetStreamingImageCB
		\li brief:
		*	Streaming Image Data update callback
		@param
		*	ip: UDP Streaming Image ip
		@param
		*   port: UDP Streaming Image port
		@param
		*	pImage: Image data(output)
		@return
		*	Success or not
		*/
		SIMONE_NET_API bool SetStreamingImageCB(const  char* ip, unsigned short port, void(*cb)(SimOne_Data_Image *pImage));
		

		/*!
		\li function:
		*	GetStreamingPointCloud
		\li brief:
		*	Get Streaming PointCloud Data
		@param
		*	ip: UDP Streaming PointCloud ip
		@param
		*   port: UDP Streaming PointCloud port
		@param
		*   infoPort: UDP Streaming device info port
		@param
		*	pPointCloud: PointCloud data(output)
		@return
		*	Success or not
		*/
		SIMONE_NET_API bool GetStreamingPointCloud(const  char* ip, unsigned short port, unsigned short infoPort, SimOne_Data_Point_Cloud *pPointCloud);
		/*!
		\li function:
		*	SetStreamingPointCloudUpdateCB
		\li brief:
		*	Streaming PointCloud Data update callback
		@param
		*	ip: UDP Streaming PointCloud ip
		@param
		*   port: UDP Streaming PointCloud port
		@param
		*   infoPort: UDP Streaming device info port
		@param
		*	pPointCloud: PointCloud data(output)
		@return
		*	Success or not
		*/
		SIMONE_NET_API bool SetStreamingPointCloudUpdateCB(const  char* ip, unsigned short port, unsigned short infoPort, void(*cb)(SimOne_Data_Point_Cloud *pPointCloud));

	}
#ifdef __cplusplus
}
#endif