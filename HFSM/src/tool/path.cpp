//
// Created by 谢卫凯 on 2021/9/14.
//

#include "path.h"
#include "predict.h"

#include <cmath>
#include <math.h>

#define OBSTACLE_RANGE 15.0 //这个用在getObstacleRange函数的参数
#define DELTA_V  1.5  //速度采样间隔
#define DELTA_STEERING  (3.14159265358979323846264338327950288/24)    //前轮转角采样间隔
#define GENERATE_TIME 5.0 //生成路径的时间(s)
#define MAX_TIME 2.0  //超过这个时间就不管偏离中心线的距离了

namespace path{
    double calcLateralAcc(const tool::Car &car,double delta){
        return car.getKappa(delta) * car.v * car.v;
    }

    void pathReserve(tool::Path &pth,int n){
        pth.x.reserve(n);
        pth.y.reserve(n);
        pth.yaw.reserve(n);
        pth.d.reserve(n);
        pth.kappa.reserve(n);
        pth.T.reserve(n);
    }
    void pathPush(tool::Path &pth,double x,double y,double yaw,int d,double kappa,double T,double v){
        pth.x.push_back(x);
        pth.y.push_back(y);
        pth.yaw.push_back(yaw);
        pth.d.push_back(d);
        pth.kappa.push_back(kappa);
        pth.T.push_back(T);
        pth.v.push_back(v);
    }

    void generatePath(const tool::Car &car, double delta, double v,double T,tool::Path &pth) {
        tool::Car tmpCar = car;
        int size = (int)ceil(T/car.dt);
        pth.clear();
        pathReserve(pth,size+1);
        tmpCar.v = v;
        double kappa = tmpCar.getKappa(delta);
        if(calcLateralAcc(tmpCar,delta) >= MAX_LATERAL_ACC)
            tmpCar.v = sqrt(MAX_LATERAL_ACC/kappa);

        double curT = 0.0;
        for(int i=0;i <= size;i++){
            pathPush(pth,tmpCar.x,tmpCar.y,tmpCar.psi,1,kappa,curT,tmpCar.v);
            curT += car.dt;
            tmpCar.move(0,delta);
        }
    }

    //调试时用的函数
    void printOb(const tool::Obstacle &ob){
        printf("ob>>type:%d pos:(%f,%lf) yaw:%lf len:%lf width:%lf\n",ob.type,ob.x,ob.y,ob.yaw,ob.length,ob.width);
    }

    bool checkStaticCollision(const tool::Car &car,const tool::Path &carPth,const tool::Obstacles &obs){
        int size = carPth.size();
        tool::Car tmpCar = car;

        tool::Obstacle **o;
        int obsSize;
        for(int i=0;i < size ;i++){
            tmpCar.setState(carPth.x[i],carPth.y[i],carPth.yaw[i],1);
            o = obs.getObsRange(tmpCar.x,tmpCar.y,OBSTACLE_RANGE,obsSize);
            //printf("car:(%lf,%lf) obsSize:%d------------\n",tmpCar.x,tmpCar.y,obsSize);
            for (int j = 0; j < obsSize; j++) {
                //printOb(*o[j]);
                if(o[j]->type != POINT && tmpCar.checkObstacleCol(*o[j])){
                    //printf("here should return true");
                    delete [] o;
                    return true; //会发生碰撞
                }
            }
            //printf("----------------------------------\n");
            delete [] o;
        }

        return false;
    }

    bool checkDynamicCollision(const tool::Car &car, const tool::Path &carPth, const DynamicObstacles &obs) {
        int size = obs.dyObs.size();
        tool::Path predictPath;
        tool::DynamicObs dyObs;
        tool::Car tmpCar = car;

        for (int i = 0; i < size; i++) {
            int carPathSize = carPth.size();
            tool::Obstacle tmpOb;
            dyObs = *obs.dyObs[i];
            tmpOb = dyObs.obs;
            dyObs.predict(predictPath, car.dt, carPathSize * car.dt);

            for(int j=0;j < carPathSize;j++){
                tmpCar.setState(carPth.x[j],carPth.y[j],carPth.yaw[j],0);
                tmpOb.x = predictPath.x[j];
                tmpOb.y = predictPath.y[j];

//                printf("car(%lf,%lf) T:%lf obs(%lf,%lf) T:%lf\n", carPth.x[j], carPth.y[j], carPth.T[j],
//                       predictPath.x[j], predictPath.y[j], predictPath.T[j]);

                if(tmpOb.type != POINT && tmpCar.checkObstacleCol(tmpOb)){
                    return true;
                }
            }
        }
        return false;
    }

    void getGreatestPath(tool::Path &pth, const tool::Car &car, double maxSteering, double maxVel, const Deviation &deviation,
                         double goalX, double goalY, const tool::Obstacles &obstacles, const DynamicObstacles &dyObs,
                         const Weight &wght) {
        //printf("get into getGreatestPath -------------------------\n");
        tool::Path greatestPath;
        tool::Path curPath;
        double minCost = 999999.999;
        double curCost = minCost;
        std::vector<double> steerings;
        steerings.reserve(48);
        for(double steering = maxSteering;steering >= -maxSteering;steering -= DELTA_STEERING)
            steerings.push_back(steering);
        steerings.push_back(0.0);
        steerings.push_back(-maxSteering);
        int size = steerings.size();

        for(double vel=maxVel;vel >= 1.0;vel -= DELTA_V){
            for(int steerInd=0;steerInd < size;steerInd++) {
                double steering = steerings[steerInd];
                generatePath(car,steering,vel,GENERATE_TIME,curPath);
                if(checkStaticCollision(car,curPath,obstacles) || checkDynamicCollision(car,curPath,dyObs))//发生碰撞，舍弃该轨迹
                    continue;
                //printf("call calcCost steering:%lf vel:%lf>>\n",steering,vel);
                curCost = calcCost(curPath,deviation,goalX,goalY,obstacles,wght);
                //printf("get out of calcCost----------------\n\n");
                if(curCost < minCost) {
                    //printf("and this is minCost^^^^^^^^^^^^\n\n");
                    minCost = curCost;
                    greatestPath <<= curPath;
                }
            }
        }
        pth <<= greatestPath; //若上面的循环一条轨迹都没找到，则greatestPath还是空的状态，则pth此时也为空
    }

    double calcCost(const tool::Path &pth, const Deviation &deviation, double goalX,double goalY,
                    const tool::Obstacles &obstacles,const Weight &wght) {
        double cost = 0.0;
        double devia = 0.0;
        double dist = 0.0;
        double vCost = 0.0;
        double ObsCost = 0.0;

        int size = pth.size();
        double T,x,y;
        tool::Obstacle **tmpObs;
        int obSize;
        for (int i = 0; i < size; i++) {
            T = pth.T[i];
            x = pth.x[i];
            y = pth.y[i];

            //printf("cur:(%lf,%lf) goal:(%lf,%lf) T:%lf\n",x,y,goalX,goalY,T);
            //printf("devia:%lf dist:%lf....\n",deviation(x,y),sqrt(pow(x-goalX,2)+pow(y-goalY,2)));

            if(T <= 0.01)
                T = 0.01;
            if(T >= MAX_TIME)
                T = 9999999.9;
            devia += deviation(x,y) / T;
            dist += sqrt(pow(x-goalX,2)+pow(y-goalY,2));
            tmpObs = obstacles.getObsRange(x,y,15.0,obSize);
            for(int j=0;j<obSize;j++)
                ObsCost -= sqrt(pow(x-tmpObs[j]->x,2)+pow(y-tmpObs[j]->y,2));
            delete [] tmpObs;
        }
        vCost = -pow(pth.v[0],2);
        //printf("deviate:%lf dist:%lf vel:%lf obs:%lf\n",wght.k1 * devia,wght.k2 * dist,wght.k3 * vCost,wght.k4 * ObsCost);
        return wght.k1 * devia + wght.k2 * dist + wght.k3 * vCost + wght.k4 * ObsCost;
    }

    DynamicObstacles::DynamicObstacles(const DynamicObstacles &d) {
        int size = d.dyObs.size();
        tool::DynamicObs *tmp;

        dyObs.reserve(size);
        for (int i = 0; i < size; i++) {
            tmp = new tool::DynamicObs;
            *tmp = *d.dyObs[i];
            dyObs.push_back(tmp);
        }
    }

    DynamicObstacles &DynamicObstacles::operator=(const DynamicObstacles &d) {
        int size = dyObs.size();
        for(int i=0;i<size;i++)
            delete dyObs[i]; //释放空间

        dyObs.clear(); //清除原来的

        //下面是复制
        size = d.dyObs.size();
        tool::DynamicObs *tmp;

        dyObs.reserve(size);
        for (int i = 0; i < size; i++) {
            tmp = new tool::DynamicObs;
            *tmp = *d.dyObs[i];
            dyObs.push_back(tmp);
        }
        return *this;
    }

    DynamicObstacles::~DynamicObstacles() {
        int size = dyObs.size();
        for(int i=0;i<size;i++)
            delete dyObs[i]; //释放空间

        dyObs.clear();
    }

    void DynamicObstacles::insertDynamicOb(const tool::DynamicObs &ob) {
        tool::DynamicObs *tmp;
        tmp = new tool::DynamicObs;
        *tmp = ob;
        dyObs.push_back(tmp);
    }

}
