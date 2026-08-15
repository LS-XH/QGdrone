# 研究笔记

[高斯光泼溅指南](https://zsc.github.io/3dgs_tutorial/html/index.html)
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