//
// Created by 谢卫凯 on 2021/9/11.
//
#include "predict.h"

namespace tool{
    void predictObsPath(const Obstacle &obs,double vx,double vy,double dt,double T,Path &res){
        int size = (int )ceil(T/dt);
        double curT = 0.0,x = obs.x,y=obs.y;
        res.clear();
        res.x.reserve(size+1);
        res.y.reserve(size+1);
        res.T.reserve(size+1);
        for(int i=0;i<=size;i++){
            res.T.push_back(curT);
            res.x.push_back(x);
            res.y.push_back(y);
            curT += dt;
            x += vx * dt;
            y += vy * dt;
        }
    }

    void DynamicObs::predict(Path &res, double dt, double T) {
        predictObsPath(obs,vx,vy,dt,T,res);
    }

    DynamicObs::DynamicObs(const DynamicObs &d) {
        this->vx = d.vy;
        this->vy = d.vy;
        this->obs = d.obs;
    }

    DynamicObs &DynamicObs::operator=(const DynamicObs &d) {
        this->vx = d.vx;
        this->vy = d.vy;
        this->obs = d.obs;
        return *this;
    }
}
