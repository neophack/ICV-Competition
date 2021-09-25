//
// Created by 谢卫凯 on 2021/9/11.
//

#ifndef CPP_PREDICT_H
#define CPP_PREDICT_H

#include "car.h"

#include <math.h>
#include <cmath>

namespace tool{
    class DynamicObs{//c++ 的继承还没看，就不用继承这种离谱的东西了
    public:
        double vx,vy; //两个分速度
        Obstacle obs; //障碍物

        DynamicObs(){vx = vy = 0;obs.type = POINT;}
        DynamicObs(double vx,double vy,const Obstacle &ob){
            this->vx = vx;
            this->vy = vy;
            obs = ob;
        }
        DynamicObs(const DynamicObs &d);

        DynamicObs & operator=(const DynamicObs &d);

        void predict(Path &res,double dt,double T);
    };
    void predictObsPath(const Obstacle &obs,double vx,double vy,double dt,double T,Path &res);
}

#endif //CPP_PREDICT_H
