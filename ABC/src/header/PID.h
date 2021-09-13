//
// Created by 谢卫凯 on 2021/8/27.
//

#ifndef CPP_PID_H
#define CPP_PID_H

namespace tool{
    class pid{
        double kp,ki,kd;
        double integral; // 积分值
        double dt; //离散化时间间隔
        double maxIntegral;//最大积分值
        double ref;     //目标值
        double last; //上一个达到的值
    public:
        pid(double kp=0,double ki=0,double kd=0,double dt=0.1,double max = 100.0,double ref = 0);
        void setMaxIntegral(double max){this->maxIntegral = max;}//设定最大积分值
        void setPid(double kp,double ki,double kd){this->kp=kp;this->ki=ki;this->kd=kd;} //设定三个系数

        double getControl(double current);

        pid & operator=(double); // used to set ref
    };
}

#endif //CPP_PID_H
