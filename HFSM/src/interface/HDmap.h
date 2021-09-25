//
// Created by 谢卫凯 on 2021/9/23.
//

#ifndef ICV_COMPETITION_HDMAP_H
#define ICV_COMPETITION_HDMAP_H

namespace interface {
    //道路的结构体
    struct lane {
        int roadID;
        int laneID;
        int sectionInd;
    };

    //SimString转lane结构
    void SimString2Lane(SSD::SimString &s, lane &lane);

    //lane结构转SimString
    void Lane2SimString(const lane &lane,SSD::SimString &s);

    //获取相对车道的s-t坐标
    bool getLaneST(const lane &lane,double x,double y,double &s,double &t);

    //获取相对道路的s-t坐标
    bool getRoadSt(const lane &lane,double x,double y,double &s,double &t);

    //根据s-t坐标获取x-y-z坐标
    bool ST2XYZ(const lane &lane,const double &s,const double &t,double &x,double &y,double &z);

    //根据gps信息获取道路信息
    bool getLaneInfoFromGps(SimOne_Data_Gps *gps,lane &l,double &s,double t);

}

#endif //ICV_COMPETITION_HDMAP_H
