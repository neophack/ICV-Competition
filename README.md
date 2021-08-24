# 智能网联汽车设计竞赛

## 介绍

该项目用于参加2021年度的智能网联汽车设计竞赛，基于 51Sim-One Cloud 给出的样例文件开发。

## 目录结构

```
.
├── ADAS           // ADAS赛题
│   └── AEB
├── HDMap
│   └── include    // HDMap样例
├── build
├── include
├── legacy         // Python代码尝试
└── lib
```

## 编译

克隆到本地：

```shell
git clone https://github.com/YMaster7/ICV-Competition.git
cd ICV-Competition
```

编译：

```shell
cd build
bash build_release.sh
```

清除：

```shell
bash build_clean.sh
```