//
// Created by 谢卫凯 on 2021/8/27.
//

#include "PID.h"

#include <iostream>

#include <cmath>

namespace tool{
    pid::pid(double kp,double ki,double kd,double dt,double max ,double ref) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->maxIntegral = max;
        this->ref = ref;
        this->last = std::nan("");
        this->integral = 0;
        this->dt = dt;
    }

    double pid::getControl(double current) {
        double err = this->ref - current;
        double d = 0;
        this->integral += err * this->dt;
        if(this->integral > this->maxIntegral)
            this->integral = 0;
        if(!std::isnan(this->last))
            d = (current - last) / dt;

        last = current;
//        std::cout << err << "  d:"<<d << " integral:"<<integral<<std::endl;
//        std::cout <<"control:" << kp*err + ki*integral - kd*d << std::endl;
        return kp*err + ki*integral - kd*d;
    }

    pid & pid::operator=(double ref) {
        this->ref = ref;
        return *this;
    }
}
