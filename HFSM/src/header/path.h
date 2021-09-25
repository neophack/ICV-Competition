//
// Created by 谢卫凯 on 2021/9/14.
//

#ifndef CPP_PATH_H
#define CPP_PATH_H

#define MAX_LATERAL_ACC 6.0

#include "car.h"
#include "predict.h"

#include <vector>

namespace path {
    //动态障碍物类
    class DynamicObstacles{
    public:
        std::vector<tool::DynamicObs *> dyObs;

        //插入一个动态障碍物
        void insertDynamicOb(const tool::DynamicObs &ob);

        //默认构造函数
        DynamicObstacles(){dyObs.reserve(128);}

        //复制构造函数
        DynamicObstacles(const DynamicObstacles &d);

        //重载赋值运算符
        DynamicObstacles & operator=(const DynamicObstacles &d);

        ~DynamicObstacles();
    };

    //这个类，为计算车到车道中心线这个功能提供一个平台到我这个代码的接口，我的代码就不用管平台的事
    //在calcCost里会用到这个类
    class Deviation{
        void *data;
        double (*deviation)(void *data,double x,double y); //这个函数指针是计算坐标(x,y)到车道中心的距离，data自行设计
    public:
        Deviation(void *data,double (*deviation)(void *data,double x,double y)):data(data),deviation(deviation){};
        //~Deviation(){delete data;}

        //封装一下，只需要传x,y参数就好
        double operator()(double x,double y) const {return deviation(data,x,y);}
    };

    //为下面那个calcCost的权重参数
    typedef struct weight{
        double k1,k2,k3,k4;//四个权重分别为偏离中心线、距离终点距离、速度、障碍物代价
    }Weight;

    //根据车辆配置（车辆坐标 车体参数）  前轮转角 速度
    void generatePath(const tool::Car &car, double delta, double v, double T, tool::Path &pth);

    // 检测一辆车的路径是否与某个静态障碍物发生碰撞
    bool checkStaticCollision(const tool::Car &car, const tool::Path &carPth, const tool::Obstacles &obs);

    // 检测一辆车的路径是否与某个动态障碍物发生碰撞
    bool checkDynamicCollision(const tool::Car &car,const tool::Path &carPth,const DynamicObstacles &obs);

    //算一条轨迹的cost值 pth:轨迹 deviation:能当成函数返回某点到中心线距离 (goalX,goalY)是目标点 obstacles是静态障碍物，wght是权重结构体
    double
    calcCost(const tool::Path &pth,const Deviation &deviation, double goalX,double goalY,
             const tool::Obstacles &obstacles,const Weight &wght);

    //根据速度范围和前轮转角范围获取最佳轨迹
    //pth:返回的轨迹 car:车辆参数 maxSteering:最大方向盘转角 maxVel:最大速度
    //deviation:同上 (goalX,goalY)同上 后面两个也同上
    //注意，若该函数找不到合适的轨迹，则会将pth整个清空(若发生这种情况那就该停车了)
    void getGreatestPath(tool::Path &pth,const tool::Car &car,double maxSteering,double maxVel,
                         const Deviation &deviation,
                         double goalX,double goalY,const tool::Obstacles &obstacles,const DynamicObstacles &dyObs,const Weight &wght);
}
#endif //CPP_PATH_H
