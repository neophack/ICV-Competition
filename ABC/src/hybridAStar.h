//
// Created by 谢卫凯 on 2021/9/2.
//

#ifndef CPP_HYBRIDASTAR_H
#define CPP_HYBRIDASTAR_H

#include "car.h"

#include <cmath>
#include <iostream>
#include <ostream>
#include <math.h>
#include <algorithm>

namespace tool {
    typedef struct ind {
        int x, y, yaw;
        ind(){x=0;y=0;yaw=0;}
        ind(int x,int y,int yaw){this->x=x;this->y=y;this->yaw=yaw;}
        bool operator==(ind&b){return this->x==b.x && this->y==b.y && this->yaw == b.yaw;}
    } Ind;

    typedef struct conf {
        double gridRes; //grid resolution x-y网格最小格边长
        double yawRes;  //yaw resolution
        double maxSteering; //max steering angle
        int NSteer; //将 [-maxSteering,maxSteering]分成NSteer个角度，执行往后走的操作

        double steerCost; //转角惩罚为steerCost * steering
        double switchBackCost; //换方向的惩罚为 switchBackCost

        Obstacles obs; //各个障碍物  （POINT 为尽量不碰的点 SOLID_POINT为绝不能碰的点 OBSTACLE为不能碰的有形状障碍物）
        double obsCost; //碰到POINT 类型的障碍物的惩罚

        tool::Car car; //车辆模型
    } Config;

    typedef struct node {
        double x, y, yaw; //该节点状态
        double parentKappa; //到该节点的父节点转弯到该节点的曲线曲率
        double cost; //g 值
        Ind parent;
        int dir; //以什么方向开到这个点
    } Node;

    //起点状态、终点状态、配置   注意这里返回的path是不带v和T的，也就是pth.v和pth.T还是空的
    bool hybridAStar(double x,double y,double yaw,double endx,double endy,double endYaw,const Config &cfg,Path &pth);

    void test();
}
#endif //CPP_HYBRIDASTAR_H
