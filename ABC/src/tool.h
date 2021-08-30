//
// Created by 谢卫凯 on 2021/8/26.
//

#ifndef SIMONEIOAPISAMPLE_HELLO_H
#define SIMONEIOAPISAMPLE_HELLO_H

#include "SimOneNetAPI.h"
#include "SSD/SimPoint3D.h"
#include "util/UtilMath.h"
#include "util/UtilDriver.h"

void hello();
int leftCenterLine(SimOne_Data_Gps *gps,SSD::SimPoint3DVector &targetPath);
int rightCenterLine(SimOne_Data_Gps *gps,SSD::SimPoint3DVector &targetPath);
int centerLine(SimOne_Data_Gps *gps,SSD::SimPoint3DVector &targetPath);

#endif //SIMONEIOAPISAMPLE_HELLO_H
