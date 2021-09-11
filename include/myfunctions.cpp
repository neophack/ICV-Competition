#pragma once

#ifdef BUILD_SIMONE_IO
#if defined(WIN32) || defined(_WIN32)
#define MY_FUNCTIONS __declspec(dllexport)
#elif defined(__linux__) || defined(__linux)
#define MY_FUNCTIONS __attribute__((visibility("default")))
#endif
#else
#define MY_FUNCTIONS 
#endif

#include <string>
#include "SimOneIOStruct.h"
#ifndef WITHOUT_HDMAP
#include "public/common/MEnum.h"
#include "public/common/MLaneLink.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MRoadMark.h"
#include "public/common/MSignal.h"
#include "public/common/MObject.h"
#include "public/common/MParkingSpace.h"
#include "SSD/SimPoint3D.h"
#endif
#include "SimOneNetAPI.h"

namespace myfunctions {
    int __my_count = 0;
    bool LessMessage(int& cnt, ELogLevel_Type level, const char *format, ...) {
        if(__my_count++ >= cnt) {
            __my_count = 0;
            return SimOneAPI::bridgeLogOutput(level, format);
        }
        return 0;
    }
}
