//
// Created by 谢卫凯 on 2021/9/2.
//
#include "car.h"
#include "hybridAStar.h"

#include <cmath>
#include <vector>
#include <map>
#include <queue>
#include <ostream>
#include <iostream>
#include <algorithm>
#include <math.h>

#define ALPHA 1.5
#define DISTANCE 5.0
#define SCALOR 25.0
//上面这俩宏和启发函数有关

#define DEBUG 0
#define AFTERNEXT 0
#define GETNEXTNODE 0
#define MAXCOST 99999.9 //无法接受的代价

typedef struct mem{
    double cost;
    tool::Ind ind;
    mem(double cost,tool::Ind i){this->cost=cost;this->ind=i;}
}Mem;

struct cmp{
    bool operator()(Mem *a,Mem *b){
        return a->cost >= b->cost;
    }
};

void getInd(const tool::Config &cfg,double x,double y,double yaw,tool::Ind &ind){
    ind.x = x >= 0 ? (int )floor(x/cfg.gridRes)+1:(int)ceil(x/cfg.gridRes)-1;
    ind.y = y >= 0 ? (int)floor(y/cfg.gridRes)+1:(int )ceil(y/cfg.gridRes)-1;
    ind.yaw = yaw >= 0 ? (int)floor(yaw/cfg.yawRes)+1:(int )ceil(yaw/cfg.yawRes)-1;
}

double hCost(const tool::Node &start,const tool::Node &end){ //这里随便写一个启发函数
    double d = abs(end.yaw-start.yaw);
    double cost = pow(end.y-start.y,2) + pow(end.x-start.x,2);
    if(cost <= DISTANCE*DISTANCE) {
        if(d > M_PI)
            d = 2*M_PI - d;
        cost += ALPHA * d*d;
    }
    return SCALOR*sqrt(cost); //放大到目标点的重要性
}

namespace tool{
    bool getFinalPath(std::map<Ind,Node> closeList,Path &path,Ind ind){
#if DEBUG
        printf("get into getFinalPath\n");
#endif
        double lastKappa = 0.0;
        Ind toEnd;
        Node curNode = closeList[ind];
        do {
#if 0
            printf("here is in getFinalPath----------------%lu\n",closeList.size());
            printf("ind:x:%d y:%d yaw:%d\n",ind.x,ind.y,ind.yaw);
            printf("x:%lf y:%lf yaw:%lf\n",curNode.x,curNode.y,curNode.yaw);
            printf("*****************************\n");
#endif
            path.x.push_back(curNode.x);
            path.y.push_back(curNode.y);
            path.yaw.push_back(curNode.yaw);
            path.kappa.push_back(lastKappa);
            path.d.push_back(curNode.dir);
            lastKappa = curNode.parentKappa;
            ind = curNode.parent;
            curNode = closeList[ind];
        } while (!(curNode.parent == toEnd));
        std::reverse(path.x.begin(),path.x.end());
        std::reverse(path.y.begin(),path.y.end());
        std::reverse(path.yaw.begin(),path.yaw.end());
        std::reverse(path.kappa.begin(),path.kappa.end());
        std::reverse(path.d.begin(),path.d.end());
        return true;
    }

    bool operator<(const tool::Ind &a,const tool::Ind &b){
        if(a.x != b.x) return a.x < b.x;
        if(a.y != b.y) return a.y < b.y;
        if(a.yaw != b.yaw) return a.yaw < b.yaw;
        return false;
    }
    // 返回和障碍物碰撞的cost
    double calcObsCost(const Car &car,const Config &cfg){
        Obstacle **p;
        int size;
        double cost = 0.0;
        p = cfg.obs.getObsRange(car.x,car.y,8,size);//8米之内应该够了吧
        for(int i=0;i<size;i++){
            if(car.checkObstacleCol(*p[i])){
                if(p[i]->type == POINT )
                    cost += cfg.obsCost;
                else {
                    delete [] p;
                    return 999999999.9; //不该碰的东西碰了就直接返回一个大数
                }
            }
        }
        delete [] p;
        return cost;
    }

    //判断是否在同一个格
    bool inSameGrid(const Ind &a,const Ind &b){
        //return a == b;
        return a.x == b.x && a.y == b.y; //试一下不管角度有什么效果
    }

    //由配置，当前车辆状态，上一个状态的前进方向，前轮转角为输入  通过参数返回上一个节点到新节点增加的cost 新节点 及新节点的ind
    void getNextNode(const Config &cfg,Car car,int lastDir,double steering,double &deltaCost,Node &node,Ind &ind){
        Ind tmp,curInd;
        getInd(cfg,car.x,car.y,car.psi,curInd);
#if GETNEXTNODE
        printf("--------------------in getNextNode------------------------------\n");
        printf("steering:%lf d:%lf\n",steering,car.v);
        printf("curInd:x:%d y:%d yaw:%d\n",curInd.x,curInd.y,curInd.yaw);
#endif
        car.move(0,steering);
        getInd(cfg,car.x,car.y,car.psi,tmp);
        deltaCost = abs(car.v)*car.dt;
        while(inSameGrid(tmp,curInd)){//走到下一个格子为止
            car.move(0,steering);
            getInd(cfg,car.x,car.y,car.psi,tmp);
            deltaCost += abs(car.v)*car.dt;
        }
        node.x = car.x;node.y = car.y;node.yaw = car.psi;node.dir = car.v;
        deltaCost += calcObsCost(car ,cfg);
        deltaCost += abs(cfg.steerCost * steering);
        if(lastDir != car.v)//换方向了
            deltaCost += cfg.switchBackCost;
        ind = tmp;
#if GETNEXTNODE
        printf("if switchback:%d\n",lastDir != car.v);
        printf("%lf %lf %lf %lf\n",abs(car.v)*car.dt,calcObsCost(car ,cfg),abs(cfg.steerCost * steering),cfg.switchBackCost);
        printf("ind:x:%d y:%d yaw:%d \n",ind.x,ind.y,ind.yaw);
        printf("nextNode: x:%lf y:%lf yaw:%lf deltaCost:%lf\n",node.x,node.y,node.yaw,deltaCost);
        printf("--------------------------\n\n\n");
#endif
    }

    bool hybridAStar(double x,double y,double yaw,double endx,double endy,double endYaw,const Config &cfg,Path &pth){
        Node start,currentNode,end;
        Ind endInd;
        Ind ind;
        std::map<Ind,Node> openList;
        std::map<Ind,Node> closeList;
        std::priority_queue<Mem*,std::vector<Mem*>,cmp> hp;
        Mem *temp;
        std::vector<double > steerings;
        steerings.reserve(cfg.NSteer+5); //预留一些
        double change = 2*cfg.maxSteering / cfg.NSteer;
        for(double steer=-cfg.maxSteering;steer <= cfg.maxSteering;steer += change)
            steerings.push_back(steer);
        steerings.push_back(cfg.maxSteering);
        steerings.push_back(0.0);
#if DEBUG
        for(int i=0;i<steerings.size();i++)
            printf("%lf ",steerings[i]);
        printf("  end of steerings\n");
#endif

        start.x = x;start.y = y;start.yaw = yaw;
        start.cost = 0;
        start.parentKappa = 0;
        start.parent = ind;
        start.dir = 1;
        end.x = endx; end.y = endy; end.yaw = endYaw;
        getInd(cfg,endx,endy,endYaw,endInd);
        getInd(cfg,x,y,yaw,ind);

        openList[ind]=start;
        hp.push(new Mem(start.cost+hCost(start,end),ind));

        while(!hp.empty()){
            temp = hp.top();
            hp.pop();
            if(openList.find(temp->ind) == openList.end()){//不在openList里
                delete temp;
                continue;
            }
            ind = temp->ind;
            currentNode = openList[ind];
            openList.erase(ind);
            if(temp->cost >= MAXCOST){//最小的代价都很大，那就算是找不到路了吧
                delete temp;
                while(!hp.empty()){//释放空间
                    temp = hp.top();
                    hp.pop();
                    delete temp;
                }
                return false;
            }
            delete temp;
            closeList[ind] = currentNode;
#if DEBUG
            printf("here in hybridA*-----------\n");
            printf("endInd:x:%d y:%d yaw:%d   ind:x:%d y:%d yaw:%d\n",endInd.x,endInd.y,endInd.yaw,ind.x,ind.y,ind.yaw);
            printf("currentNode:x:%lf y:%lf yaw:%lf endNode:x:%lf y:%lf yaw:%lf\n",currentNode.x,currentNode.y,currentNode.yaw,end.x,end.y,end.yaw);
            printf("cost:%lf hCost:%lf totalCost:%lf\n",currentNode.cost,hCost(currentNode,end),currentNode.cost+hCost(currentNode,end));
            printf("******************************\n");

#endif

            if(ind == endInd) {
                while(!hp.empty()){//释放空间
                    temp = hp.top();
                    hp.pop();
                    delete temp;
                }
                return getFinalPath(closeList, pth, endInd);
            }

            for(int d=-1;d <= 1;d+=2){
                for(int i=0;i<steerings.size();i++){
                    double steer = steerings[i];
                    Ind nextInd;
                    double deltaCost;
                    Node nextNode;
                    Car car = cfg.car;
                    car.setState(currentNode.x,currentNode.y,currentNode.yaw,d);
                    getNextNode(cfg,car,currentNode.dir,steer,deltaCost,nextNode,nextInd);
                    if(closeList.find(nextInd) != closeList.end())//新走到的节点已经在closeList里了
                        continue;
                    if(deltaCost >= 999999999.0) //忽略该节点
                        continue;
                    nextNode.cost = currentNode.cost + deltaCost;
                    nextNode.parent = ind;
                    nextNode.parentKappa = car.getKappa(steer);
#if AFTERNEXT
                    printf("after getNextNode:nextInd:x:%d y:%d yaw:%d\n",nextInd.x,nextInd.y,nextInd.yaw);
                    printf("in openList:\n");
                    for(auto it=openList.begin();it != openList.end();it++)
                        printf("key:x:%d y:%d yaw:%d\n",it->first.x,it->first.y,it->first.yaw);
                    printf("end\n");
                    printf("find:%d\n",openList.find(nextInd) == openList.end());
#endif
                    if(openList.find(nextInd) == openList.end() || openList.find(nextInd)->second.cost > nextNode.cost){//加入openList
#if 0
                        printf("not in:%d lessCost:%d\n", openList.find(nextInd) == openList.end(),
                               openList.find(nextInd)->second.cost > nextNode.cost);
                        //printf("here something was pushed. cost:%lf nextNodeCost:%lf hCost:%lf x:%d y:%d yaw:%d\n",nextNode.cost+hCost(nextNode,end),nextNode.cost,hCost(nextNode,end),nextInd.x,nextInd.y,nextInd.yaw);
#endif
                        openList[nextInd] = nextNode;
                        hp.push(new Mem(nextNode.cost+hCost(nextNode,end),nextInd));
                    }
                }
            }

        }
        return false;//没找到
    }

    void test(){
        tool::Ind ind;
        tool::Car car(2.9187,1.85,3.9187,1,0.1);
        tool::Config cfg;
        tool::Obstacles obs;
        cfg.gridRes = 0.5;
        cfg.yawRes = M_PI/24;
        cfg.maxSteering = M_PI/3;
        cfg.NSteer = 12;
        cfg.car = car;
        cfg.obs = obs;
        cfg.obsCost = 15;
        cfg.steerCost = 0.0;
        cfg.switchBackCost = 70.0;

        getInd(cfg,10.440798,10.079969,-0.033124,ind);
        printf("test:x:%d y:%d yaw:%d\n",ind.x,ind.y,ind.yaw);

    }
}


