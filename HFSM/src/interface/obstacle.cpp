//
// Created by 谢卫凯 on 2021/9/21.
//

#include "obstacle.h"

#include <memory>
#include "SimOneNetAPI.h"
#include "util/UtilMath.h"

namespace interface{

    bool getAllObstacles(tool::Obstacles &obs, path::DynamicObstacles &dyObs) {
        std::unique_ptr<SimOne_Data_Obstacle> pObstacle = std::make_unique<SimOne_Data_Obstacle>();
        if (!SimOneAPI::GetSimOneGroundTruth(pObstacle.get()))
            return false;

        tool::Obstacle ob;
        ob.type = OBSTACLE;
        tool::DynamicObs dyOb;

        for(int i=0;i<pObstacle->obstacleSize;i++){
            double v = UtilMath::calculateSpeed(pObstacle->obstacle[i].velX,pObstacle->obstacle[i].velY,0.0);
            ob.x = pObstacle->obstacle[i].posX;
            ob.y = pObstacle->obstacle[i].posY;
            ob.yaw = pObstacle->obstacle[i].oriZ;
            ob.width = pObstacle->obstacle[i].width;
            ob.length = pObstacle->obstacle[i].length;
            if(v <= 0.2){ //当成静态障碍物
                obs.insertObs(ob);
            }else{
                dyOb.obs = ob;
                dyOb.vx = pObstacle->obstacle[i].velX;
                dyOb.vy = pObstacle->obstacle[i].velY;
                dyObs.insertDynamicOb(dyOb);
            }
        }

        return true;
    }
}
