//
// Created by 谢卫凯 on 2021/9/21.
//

#ifndef ICV_COMPETITION_OBSTACLE_H
#define ICV_COMPETITION_OBSTACLE_H

#include "car.h"
#include "predict.h"
#include "path.h"

namespace interface{
    //获取障碍物，转换成已有代码的类型
    //以连个引用的方式返回
    //注意传入的连个引用参数默认得是空的，要不会出问题
    //若调用API失败，则返回false
    bool getAllObstacles(tool::Obstacles &obs,path::DynamicObstacles &dyObs);

}

#endif //ICV_COMPETITION_OBSTACLE_H
