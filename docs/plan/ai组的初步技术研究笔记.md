# 研究笔记
> [!Tip]
> 这里前面的一部分是一个粗略的架构，目前具体的确定下来的内容需要从从[相机需要的东西](#相机需要的东西)

## 用于参考的GitHub开源项目或文献

* [高斯光泼溅指南](https://zsc.github.io/3dgs_tutorial/html/index.html)

> 这篇指南是2025年出的，对于3DGS的技术整体和其中比较新的技术进行了说明

* [ORB-slam3](https://github.com/UZ-SLAMLab/ORB_SLAM3)和上面那个组成了我们项目的两大核心

* [OPEN3D开源社区](https://github.com/isl-org/Open3D)通过这个可以在点云的那步生成粗略的点云模型，以便后续补拍

* [GenBV](https://github.com/zjwzcx/GenNBV/blob/master/README.md?plain=1)是用于自动规划路径进行补拍的项目，这里接入难度过大，初期暂且搁置

* [on-the-fly](https://github.com/xywjohn/GS_On-The-Fly)近乎实时更新高斯模型的项目，可以借鉴，作为我们实时更新模块的一部分,这块还有很多别的项目可以借鉴，不用只看这篇



## 一开始研究的时候搞的整体路线(已废弃)留着给你们理解用

```
RGB-D 相机
   │
   ├── RGB图像
   └── Depth
        │
        ▼
   相机位姿 Pose
   （SLAM / COLMAP）
        │
        ▼
   稀疏 / 稠密 3D点云
        │
        ▼
   初始化 3D Gaussians
        │
        ▼
   ┌──────────────────────┐
   │     3D Gaussian       │
   │       Splatting        │
   │                        │
   │  3D Gaussian → 2D图像 │
   └──────────┬───────────┘
              │
         渲染图像
              │
              ▼
      与真实RGB图像比较
              │
              ▼
          Loss / 梯度
              │
              ▼
      优化 Gaussian 参数
              │
              └───────循环
                       ↓
                   3DGS模型
```

## 相机需要的东西
首先需要 Camera Pose。

例如：
```
Camera 1       Camera 2       Camera 3


   📷             📷             📷
    \              \              \
     \              \              \
      \              \              \
       ─────────建筑──────────────
```

每个相机都需要知道：
![20260815170628](https://cdn.jsdelivr.net/gh/YCDR-810518/imageBed@main/picGo_vscode/20260815170628.png)

## 传统的3DGS模型流程架构图
```
             多张 RGB 图片
                   │
                   ▼
           Feature Extraction
                   │
                   ▼
             Feature Matching
                   │
                   ▼
                  SfM
                   │
          ┌────────┴────────┐
          ▼                 ▼
     Camera Pose       Sparse Point Cloud
          │                 │
          │                 ▼
          │          Gaussian Initialization
          │                 │
          └────────┬────────┘
                   ▼
             3D Gaussian Scene
                   │
                   ▼
             Differentiable
               Rendering
                   │
                   ▼
            Rendered Image
                   │
             ↙           ↘
       Real Image       Render Image
             ↘           ↙
                  Loss
                   │
                   ▼
             Backpropagation
                   │
                   ▼
       优化 Gaussian 参数
                   │
                   ▼
             Densification
                   │
                   ▼
            最终 3DGS 模型
```

## 我们的3DGS架构流程图
![20260815171749](https://cdn.jsdelivr.net/gh/YCDR-810518/imageBed@main/picGo_vscode/20260815171749.png)

### 我们加入的模块会帮助模型完成什么工作？
![20260815200927](https://cdn.jsdelivr.net/gh/YCDR-810518/imageBed@main/picGo_vscode/20260815200927.png)
![20260815201029](https://cdn.jsdelivr.net/gh/YCDR-810518/imageBed@main/picGo_vscode/20260815201029.png)