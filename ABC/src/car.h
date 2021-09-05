//
// Created by 谢卫凯 on 2021/8/27.
//

#ifndef CPP_CAR_H
#define CPP_CAR_H

#include <cmath>
#include <iostream>
#include <ostream>
#include <math.h>
#include <algorithm>
#include <vector>

#include "kdtree.h"

#define SOLID_POINT 0
#define POINT 2
#define OBSTACLE 1

//WB = 3.  # rear to front wheel
//W = 2.  # width of car
//        LF = 3.3  # distance from rear to vehicle front end
//        LB = 1.0  # distance from rear to vehicle back end

namespace tool{
    double pi_2_pi(double angel); // 把角度限制在 [0,2pi]
    void a2contr(double a,double &throttle,double &brake); // 把加速度转为油门刹车，这里瞎写一通就好了...

    class Path{
    public:
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw;//航偏角
        std::vector<double> T;//时间点，秒为单位
        std::vector<double> v;//应该要达到的速度
        std::vector<double> kappa;//该点曲率
        std::vector<int> d;//方向，1为前进，-1为后退
        Path();
        int size(){return x.size();}
        friend std::ostream &operator<<(std::ostream &out,Path path);
    };

    typedef struct obs{
        int type;//为0则看成一个点 为1则看成一个长方体，形状方向在下面定义 0和1都不能碰撞 若该值为2则是尽量不碰（如车道线的点可以设成2）
        double x,y;//坐标
        double width,length;//宽 长
        double yaw;//方向 弧度 该方向是从坐标引一条垂直于width的向量的方向
    }Obstacle;

    class Obstacles{
        //暂时千万不要拿这个类对象来互相赋值或者初始化
        struct kdtree *kd;//维护一颗kd-tree来保存障碍物

    public:
        Obstacles();
        ~Obstacles();

        //因为拷贝构造和赋值什么的比较麻烦，手动写一个这个代替吧
        void destroy();

        //插入一个障碍物
        void insertObs(const Obstacle &o);

        //给定一个位置找最近的障碍物
        Obstacle *getNearestObs(double x,double y) const;

        //给定一个位置，找一定距离内的障碍物，障碍物个数通过size参数返回,返回结果用完后要自行释放内存
        Obstacle **getObsRange(double x,double y,double dis,int & size) const;
    };

    class Car{
        double wb,w,lf,lb;//前后轮距离  车宽  后轮到车最前方距离  后轮到车尾距离
        double maxSteering;
    public:
        double x,y,psi,v; // 坐标 航向角 速度
        double dt;// 采样时间间隔

        //设置车辆各个参数 轴距 车宽 后轮到车头 后轮到车尾 采样时间间隔
        Car(double wb = 3,double w=2,double lf = 3.3,double lb = 1,double dt = 0.1);

        //设置车辆状态
        void setState(double x,double y,double psi,double v);

        //设置最大转角
        void setMaxSteering(double max){maxSteering = max;}

        //由控制量（加速度，前轮转角）更新车辆状态
        void move(double a,double delta);

        // 检测车辆是否与某个点发生碰撞
        bool checkPointCol(double x,double y) const;

        // 给定线段两端点，测试车辆是否与这条线段碰撞
        bool checkLineCol(double x1,double y1,double x2,double y2) const;

        //检查车辆是否与障碍物发生碰撞
        bool checkObstacleCol(Obstacle &o) const;

        // purePursuit计算steering
        double calcSteering(double ,double ) const;

        //返回一条路径中用PurePersuit 要找的下一个路径点下标
        // path：路径点 k:预瞄距离=k*v dis:最小预瞄距离
        int calcIndex(Path path,int &lastIndex,double k=0.1,double dis = 2.0);

        //返回前轮转角对应的曲率
        double getKappa(double delta) const{
            return std::abs(tan(delta)/wb);
        }
    };

}

#endif //CPP_CAR_H
