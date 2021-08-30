# 智能网联汽车设计竞赛

## 介绍

该项目用于参加2021年度的智能网联汽车设计竞赛，基于 51Sim-One Cloud 给出的样例文件开发。

## 目录结构

```
.
├── ADAS
│   ├── AEB
│   └── AVP
├── HDMap
│   └── include
├── README.md
├── bin
│   ├── AEB
│   ├── AVP
│   ├── libHDMapModule.so
│   ├── libSSD.so
│   ├── libSimOneIOAPI.so
│   ├── libSimOneStreamingIOAPI.so
│   ├── libavcodec.so.58
│   ├── libavformat.so.58
│   ├── libavutil.so.56
│   ├── libswresample.so.3
│   ├── libswscale.so.5
│   └── libx264.so.148
├── build
│   ├── CMakeLists.txt
│   ├── build_clean.sh
│   ├── build_release.sh
│   └── build_tmp
├── include
│   ├── Config.h
│   ├── DataElement.hpp
│   ├── DataFrame.hpp
│   ├── GetSensorDetectionsJsonResult.h
│   ├── GetSignType.h
│   ├── Message.hpp
│   ├── OSI
│   ├── SSD
│   ├── SimOneIOStruct.h
│   ├── SimOneNetAPI.h
│   ├── SimOneStreamingAPI.h
│   ├── client.h
│   ├── google
│   ├── gtest
│   ├── nlohmann
│   ├── public
│   ├── rapidjson
│   ├── util
│   ├── utilStartSimOneNode.h
│   └── utilTest.h
├── legacy
│   ├── AEB.py
│   └── release.sh
├── lib
│   ├── SimOneIOStruct.py
│   ├── libHDMapModule.so
│   ├── libProtobufModule.a
│   ├── libSSD.so
│   ├── libSimOneIOAPI.so
│   ├── libSimOneStreamingIOAPI.so
│   ├── libavcodec.so.58
│   ├── libavformat.so.58
│   ├── libavutil.so.56
│   ├── libgtest.a
│   ├── libopen_simulation_interface.so
│   ├── libprotobuf.so
│   ├── libpython3.6m.so.1.0
│   ├── libswresample.so.3
│   ├── libswscale.so.5
│   ├── libx264.so.148
│   ├── pySimOneIO.so
│   └── pySimOneV2X.so
└── release.sh
```

## 编译

克隆到本地：

```shell
git clone https://github.com/YMaster7/ICV-Competition.git
cd ICV-Competition
```

Compile & zip：

```shell
sh release.sh
```

清除：

```shell
sh clean.sh
```
