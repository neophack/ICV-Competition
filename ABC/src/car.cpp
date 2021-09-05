//
// Created by 谢卫凯 on 2021/8/27.
//

#include "car.h"
#include "kdtree.h"

#include <cmath>
#include <iostream>
#include <ostream>
#include <math.h>
#include <algorithm>
#include <vector>

#define MAX_ACC 15 //最大加速度

namespace tool{
    double distance(double x1,double y1,double x2,double y2){
        return std::sqrt(pow(x2-x1,2)+pow(y2-y1,2));
    }

    double pi_2_pi(double angel){ // 把角度限制在 [0,2pi]
        if(angel < 0)
            while(angel < 0)
                angel += 2*M_PI;

        if(angel > 2*M_PI)
            while(angel > 2*M_PI)
                angel -= 2*M_PI;
        return angel;
    }

    void a2contr(double a,double &throttle,double &brake){ // 把加速度转为油门刹车，这里瞎写一通就好了...
        throttle = 0;
        brake = 0;
        if(a > MAX_ACC)
            a = MAX_ACC;
        if(a < -MAX_ACC)
            a = -MAX_ACC;
        if(a >= 0)
            throttle = a / MAX_ACC;
        if(a < 0)
            brake = a / MAX_ACC;
    }

    Car::Car(double wb, double w, double lf, double lb,double dt) {
        this->wb = wb;
        this->w = w;
        this->lf = lf;
        this->lb = lb;
        this->dt = dt;
        this->maxSteering = M_PI / 6;
    }

    void Car::setState(double x, double y, double psi, double v) {
        this->x = x;
        this->y = y;
        this->psi = pi_2_pi(psi);
        this->v = v;
    }

    void Car::move(double a, double delta) {
        x += v*dt*cos(psi);
        y += v*dt*sin(psi);
        psi += v*tan(delta)*dt / wb;
        psi = pi_2_pi(psi);
        v += a*dt;
    }

    bool Car::checkPointCol(double x, double y) const{
        double carX = this->x + (lf-lb)/2 * std::cos(psi);
        double carY = this->y + (lf-lb)/2 * std::sin(psi);// 车中心点坐标(carX,carY)

        double vx = x - carX;
        double vy = y - carY;//由车中心指向该点的向量(vx,vy)

        double c = std::cos(psi);
        double s = std::sin(psi);

        x = vy*s + vx*c;
        y = vy*c - vx*s; // （vx,vy）逆时针旋转psi后的向量，和车一起旋转

        if(x<=(lf+lb)/2 && x >= -(lf+lb)/2 && y <= w/2 && y >= -w/2)
            return true;//碰撞

        return false;
    }

    bool Car::checkLineCol(double x1, double y1, double x2, double y2) const {
        double x,y; // 生成两点之间的点(x,y)

        for(double a=0;a <= 1;a=a+0.01){
            x = a*x1 + (1-a)*x2;
            y = a*y1 + (1-a)*y2;
            if(checkPointCol(x,y))
                return true;
        }
        return false;
    }

    bool Car::checkObstacleCol(Obstacle &o) const {
        double v1x,v1y,v2x,v2y;
        if(o.type != OBSTACLE)
            return this->checkPointCol(o.x,o.y);

        v1x = o.length / 2 * std::cos(o.yaw);
        v1y = o.length / 2 * std::sin(o.yaw);//(v1x,v1y)是中心为起点垂直于width的向量

        v2x = o.width / 2 * std::cos(o.yaw + M_PI/2);
        v2y = o.width / 2 * std::sin(o.yaw + M_PI/2);//(v2x,v2y)是中心为起点垂直于length的向量

        //依次检查四条边是否碰撞
        if(this->checkLineCol(o.x+v1x+v2x,o.y+v1y+v2y,o.x+v1x-v2x,o.y+v1y-v2y))
            return true;
        if(this->checkLineCol(o.x+v1x+v2x,o.y+v1y+v2y,o.x-v1x+v2x,o.y-v1y+v2y))
            return true;
        if(this->checkLineCol(o.x-v1x+v2x,o.y-v1y+v2y,o.x-v1x-v2x,o.y-v1y-v2y))
            return true;
        if(this->checkLineCol(o.x-v1x-v2x,o.y-v1y-v2y,o.x+v1x-v2x,o.y+v1y-v2y))
            return true;

        return false;
    }

    double Car::calcSteering(double goalx, double goaly) const {
        double Ld = std::sqrt(std::pow(x-goalx,2)+std::pow(y-goaly,2));
        double alpha = std::atan2(goaly-y,goalx-x) - psi;
        if(alpha == 0)
            return 0; //直走就能到

        double R = Ld/2/std::sin(alpha);
        double steering = std::atan(wb/R);

        if(steering > maxSteering)
            return maxSteering;
        if(steering < -maxSteering)
            return -maxSteering;
        return steering;
    }

    int Car::calcIndex(Path path,int &lastIndex,double k, double dis) {
        int minInd = -1;
        double minDis = 99999999;
        std::vector<double> dist;

        dist.reserve(path.size());
        // 计算每一个路径点到车坐标的距离
        for(int i=lastIndex;i < path.size();i++){
            dist[i] = distance(path.x[i],path.y[i],x,y);
            if(dist[i] < minDis){
                minDis = dist[i];
                minInd = i;
            }
        }

        //这里开始minDis设置为最短预瞄距离
        minDis = k*v > dis? k*v:dis;
        for(int i=minInd;i < path.size();i++)
            if(dist[i] >= minDis) {
                lastIndex = i;
                return i;
            }

        return path.size() - 1;
    }

    Path::Path() {
        x.reserve(100);
        y.reserve(100);
        yaw.reserve(100);
        T.reserve(100);
        v.reserve(100);
        kappa.reserve(100);
    }

    std::ostream &operator<<(std::ostream &out, Path path) {
        for (int i = 0; i < path.size(); i++)
            printf("x:%lf y:%lf yaw:%lf d:%d\n", path.x[i], path.y[i], path.yaw[i], path.d[i]);
        return out;
    }

    void destr(void *data){
        Obstacle *o = (Obstacle *)data;
        delete o;
    }

    void Obstacles::insertObs(const Obstacle &o) {
        Obstacle *obs = new Obstacle ;
        double *pos = new double[2];

        *obs = o;
        pos[0] = o.x;pos[1]=o.y;
        kd_insert(kd,pos,obs);
    }

    Obstacles::~Obstacles() {
        //暂时没有写拷贝构造函数 也没有写赋值重载的函数，这个析构函数暂时先不用了，写一个显示调用的destroy吧
//        kd_clear(kd);
//        kd_free(kd);
    }

    void Obstacles::destroy() {
        kd_clear(kd);
        kd_free(kd);
    }

    Obstacles::Obstacles() {
        kd = kd_create(2);
        kd_data_destructor(kd, destr);
    }

    Obstacle *Obstacles::getNearestObs(double x, double y) const{
        Obstacle *ret;
        kdres *res;
        double pos[2];

        pos[0] = x; pos[1] = y;
        res = kd_nearest(kd,pos);
        if(res->size <= 0)
            return NULL;
        ret = (Obstacle*)res->riter->item->data;
        kd_res_free(res);
        return ret;
    }

    Obstacle **Obstacles::getObsRange(double x, double y, double dis, int &size) const {
        Obstacle **ret;
        Obstacle **p;
        kdres *res;
        double pos[2];
        pos[0] = x; pos[1] = y;

        res = kd_nearest_range(kd, pos, dis);
        size = res->size;
        ret = new Obstacle* [size];
        p = ret;
        for(struct res_node *it=res->riter;it != NULL;it=it->next)
            *p++ = (Obstacle *)it->item->data;
        kd_res_free(res);
        return ret;
    }
}
