# SplaTAM 技术路线与实现流程

## 一、项目概述

SplaTAM (Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM) 是一种基于3D高斯的稠密RGB-D SLAM系统。它使用3D高斯作为场景表示，通过可微渲染实现相机追踪和地图构建的联合优化。

---

## 二、核心算法原理

### 2.1 3D高斯场景表示

SplaTAM使用一组3D各向同性（或各向异性）高斯来表示3D场景：

```
每个高斯的参数：
├── means3D          [N, 3]    高斯中心3D坐标（世界坐标系）
├── rgb_colors       [N, 3]    RGB颜色（直接值，非球谐系数）
├── unnorm_rotations [N, 4]    未归一化四元数（各向同性时不做旋转变换）
├── logit_opacities  [N, 1]    不透明度的logit值
└── log_scales       [N, 1]    各向同性:1维 / 各向异性:3维 尺度的对数
```

**设计决策**：
- **各向同性 vs 各向异性**：通过配置`gaussian_distribution`选择
- **颜色表示**：使用预计算的RGB颜色（而非球谐系数），简化优化
- **深度初始化**：高斯尺度通过射影几何方法初始化，距离越远的高斯越大

### 2.2 相机位姿参数化

```
cam_unnorm_rots  [1, 4, T]  每帧的未归一化旋转四元数
cam_trans        [1, 3, T]  每帧的平移向量
```

所有位姿都是**相对于第一帧**的相对变换。

---

## 三、系统架构

```
SplaTAM/
├── scripts/                          核心脚本
│   ├── splatam.py                    ★ 主算法入口
│   ├── gaussian_splatting.py         离线3DGS训练
│   ├── post_splatam_opt.py           后处理优化
│   ├── eval_novel_view.py            新视角合成评估
│   ├── export_ply.py                 PLY导出
│   ├── iphone_demo.py                iPhone在线演示
│   └── nerfcapture2dataset.py        数据集转换
│
├── utils/                            工具函数库
│   ├── slam_helpers.py               SLAM核心辅助
│   ├── slam_external.py              3DGS核心算子
│   ├── gs_helpers.py                 Gaussian辅助函数
│   ├── gs_external.py                稠密化/剪枝
│   ├── keyframe_selection.py         关键帧选择
│   └── neighbor_search.py            FAISS近邻搜索
│
├── datasets/                         数据加载层
│   └── gradslam_datasets/
│       ├── basedataset.py            基类
│       ├── replica.py                Replica数据集
│       ├── scannet.py                ScanNet数据集
│       └── tum.py                    TUM-RGBD数据集
│
├── configs/                          配置文件
│   ├── data/                         数据集配置(YAML)
│   └── <dataset>/                    场景配置(Python)
│
├── viz_scripts/                      可视化脚本
├── bash_scripts/                     Shell脚本
└── diff-gaussian-rasterization-w-depth.git/  CUDA渲染器
```

---

## 四、完整处理流程

### 4.1 流程概览

```
┌─────────────────────────────────────────────────────────────┐
│                    输入: RGB-D 视频序列                        │
│              (RGB图像 + 深度图 + 内参矩阵)                     │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              第一帧初始化 (Frame 0)                          │
│  1. 从深度图反投影生成初始高斯点云                              │
│  2. 初始化高斯参数: 位置、颜色、尺度、不透明度                    │
│  3. 设置第一帧位姿为单位变换                                    │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  主循环 (Frame 1 ~ N)                       │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  追踪阶段 (Tracking)                                 │   │
│  │  - 冻结高斯参数，仅优化当前帧相机位姿                    │   │
│  │  - 使用恒速模型初始化位姿                               │   │
│  │  - 迭代优化直到收敛                                    │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│                          ▼                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  关键帧判断                                          │   │
│  │  - 检查追踪质量是否达标                                │   │
│  │  - 达标则添加为关键帧                                  │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│                          ▼                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  建图阶段 (Mapping)                                   │   │
│  │  - 冻结相机位姿，优化所有高斯参数                        │   │
│  │  - 执行高斯稠密化 (Densification)                     │   │
│  │  - 执行高斯剪枝 (Pruning)                             │   │
│  │  - 迭代优化直到收敛                                    │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│                          ▼                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  更新场景表示                                         │   │
│  │  - 移除小高斯 (mean_scale < 0.01 * scene_radius)     │   │
│  │  - 更新最近邻数据结构                                   │   │
│  └─────────────────────────────────────────────────────┘   │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              输出: 3D高斯地图 + 相机轨迹                      │
│  - 可导出为.ply格式                                         │
│  - 支持交互式渲染和新视角合成                                  │
└─────────────────────────────────────────────────────────────┘
```

---

## 五、核心算法详解

### 5.1 第一帧初始化

```python
# 伪代码
def initialize_frame_0(rgb, depth, intrinsics):
    # 1. 从深度图反投影生成3D点云
    points_3d = depth_to_pointcloud(depth, intrinsics)
    
    # 2. 创建高斯参数
    means3D = points_3d                      # 位置
    rgb_colors = rgb[valid_mask]             # 颜色
    
    # 3. 初始化尺度 (与深度成正比)
    log_scales = initialize_scales_from_depth(depth, intrinsics)
    
    # 4. 初始化不透明度
    logit_opacities = torch.full((N, 1), 0.1)  # 初始不透明度较低
    
    # 5. 设置第一帧位姿为单位变换
    cam_unnorm_rots = [1, 0, 0, 0]          # 单位四元数
    cam_trans = [0, 0, 0]                   # 零平移
    
    return gaussians, camera_poses
```

### 5.2 追踪阶段 (Tracking)

**目标**：给定当前帧的RGB-D观测，估计相机位姿。

**核心思想**：固定高斯参数，仅优化当前帧的相机位姿。

```python
def tracking(current_frame, gaussians, prev_poses):
    # 1. 位姿初始化 (恒速模型)
    if len(prev_poses) >= 2:
        # v = pose[t-1] * pose[t-2]^-1
        initial_pose = prev_poses[-1] * v
    else:
        initial_pose = prev_poses[-1]
    
    # 2. 优化循环
    optimizer = Adam([cam_unnorm_rots, cam_trans], lr=...)
    
    for iter in range(max_iters):
        # 渲染当前视图
        rendered_rgb, rendered_depth, silhouette = render(
            gaussians, current_pose
        )
        
        # 计算损失
        loss = (
            w_im * L1(rendered_rgb, gt_rgb) +
            w_depth * L1(rendered_depth, gt_depth)
        )
        
        # 只在有效区域计算损失
        mask = (silhouette > 0.99) & valid_depth
        
        # 反向传播
        loss.backward()
        optimizer.step()
        
        # 记录最佳位姿
        if loss < best_loss:
            best_pose = current_pose.clone()
    
    return best_pose
```

**损失函数**：
```
L_tracking = w_im * ||I_rendered - I_gt||_1 + w_depth * ||D_rendered - D_gt||_1
```

### 5.3 关键帧选择

```python
def is_keyframe(current_frame, last_keyframe, rendered_depth):
    # 检查1: 相对位姿变化
    pose_change = current_pose.inverse() @ last_keyframe.pose
    if pose_change translation > trans_threshold:
        return True
    
    # 检查2: 渲染深度与观测深度的差异
    depth_diff = torch.abs(rendered_depth - observed_depth)
    if depth_diff.mean() > depth_threshold:
        return True
    
    # 检查3: 混合渲染深度与观测深度的重叠
    if overlap_ratio < overlap_threshold:
        return True
    
    return False
```

### 5.4 建图阶段 (Mapping)

**目标**：给定已知位姿的多个关键帧，优化场景表示（高斯参数）。

**核心思想**：固定相机位姿，优化所有高斯参数。

```python
def mapping(keyframes, gaussians):
    # 1. 优化循环
    optimizer = Adam([means3D, rgb_colors, log_scales, ...], lr=...)
    
    for iter in range(max_iters):
        for keyframe in keyframes:
            # 渲染关键帧视图
            rendered_rgb, rendered_depth = render(
                gaussians, keyframe.pose
            )
            
            # 计算损失 (L1 + SSIM)
            loss = (
                0.8 * L1(rendered_rgb, gt_rgb) +
                0.2 * (1 - SSIM(rendered_rgb, gt_rgb)) +
                w_depth * L1(rendered_depth, gt_depth)
            )
            
            loss.backward()
        
        optimizer.step()
        
        # 2. 高斯稠密化 (每N步)
        if step % densify_interval == 0:
            densify(gaussians)
        
        # 3. 高斯剪枝
        prune(gaussians)
    
    return gaussians
```

**损失函数**：
```
L_mapping = 0.8 * ||I_rendered - I_gt||_1 + 0.2 * (1 - SSIM(I_rendered, I_gt))
          + w_depth * ||D_rendered - D_gt||_1
```

### 5.5 高斯稠密化 (Densification)

SplaTAM结合了两种稠密化策略：

#### 策略1: 基于剪影的新高斯添加 (Splat特有)

```python
def add_new_gaussians_splat(gaussians, current_frame):
    # 1. 渲染当前帧的剪影
    silhouette = render_silhouette(gaussians, current_pose)
    
    # 2. 检测非覆盖区域
    uncovered_mask = silhouette < 0.5  # 阈值
    
    # 3. 检测深度不一致区域
    rendered_depth = render_depth(gaussians, current_pose)
    depth_diff = rendered_depth - gt_depth
    inconsistent_mask = (depth_diff > 0) & (depth_diff > 50 * median_diff)
    
    # 4. 在这些区域添加新高斯
    new_points = backproject(uncovered_mask | inconsistent_mask, gt_depth, intrinsics)
    new_gaussians = create_gaussians(new_points, gt_rgb)
    
    gaussians = concat(gaussians, new_gaussians)
    return gaussians
```

#### 策略2: 基于梯度的标准稠密化 (3DGS)

```python
def densify_by_gradient(gaussians):
    # 1. 累积2D投影中心的梯度
    grads = accumulate_gradient(means2D)
    
    # 2. 小高斯 + 高梯度 → 克隆
    small_mask = (gaussians.scales < 0.01 * scene_radius) & (grads > threshold)
    cloned = clone_gaussians(gaussians[small_mask])
    
    # 3. 大高斯 + 高梯度 → 分裂
    large_mask = (gaussians.scales > 0.01 * scene_radius) & (grads > threshold)
    split = split_gaussians(gaussians[large_mask])
    
    # 4. 移除低不透明度高斯
    opacity_mask = sigmoid(gaussians.logit_opacities) > 0.005
    
    gaussians = update(gaussians, cloned, split, opacity_mask)
    return gaussians
```

### 5.6 高斯剪枝 (Pruning)

```python
def prune_gaussians(gaussians):
    # 1. 移除低不透明度高斯
    opacity = sigmoid(gaussians.logit_opacities)
    keep_mask = opacity > 0.005
    
    # 2. 移除过大的高斯
    scale_mask = gaussians.scales < 0.1 * scene_radius
    
    # 3. 应用掩码
    keep_mask = keep_mask & scale_mask
    gaussians = gaussians[keep_mask]
    
    return gaussians
```

### 5.7 可微渲染

渲染使用`diff-gaussian-rasterization-w-depth`（修改版3D Gaussian Splatting渲染器）：

```python
def render(gaussians, camera_pose):
    # 1. 变换到相机坐标系
    transformed_means = transform_to_camera(gaussians.means3D, camera_pose)
    
    # 2. 准备渲染参数
    rendervar = {
        'means3D':        transformed_means,
        'colors_precomp': gaussians.rgb_colors,
        'rotations':      normalize(gaussians.unnorm_rotations),
        'opacities':      sigmoid(gaussians.logit_opacities),
        'scales':         exp(gaussians.log_scales),
        'means2D':        project_to_2d(transformed_means)
    }
    
    # 3. 调用CUDA渲染器
    rendered_rgb, rendered_depth, silhouette = rasterize(rendervar)
    
    return rendered_rgb, rendered_depth, silhouette
```

---

## 六、数据流详解

### 6.1 输入数据格式

```yaml
# 典型数据集目录结构
DATAROOT/
├── scene_name/
│   ├── frames/
│   │   ├── color/           # RGB图像 (PNG/JPG)
│   │   │   ├── 0.jpg
│   │   │   ├── 1.jpg
│   │   │   └── ...
│   │   ├── depth/           # 深度图 (PNG)
│   │   │   ├── 0.png
│   │   │   ├── 1.png
│   │   │   └── ...
│   │   ├── intrinsic/       # 相机内参
│   │   │   └── 0.txt        # 3x3内参矩阵
│   │   └── pose/            # 相机位姿 (可选，用于评估)
│   │       ├── 0.txt
│   │       └── ...
│   └── info.yaml            # 数据集元信息
```

### 6.2 数据处理流程

```
原始数据
    │
    ▼
┌─────────────────────────────────────┐
│ 数据加载与预处理                      │
│ 1. 读取RGB图像                       │
│ 2. 读取深度图 (÷ png_depth_scale)    │
│ 3. 读取内参矩阵 (按图像缩放比例调整)   │
│ 4. 读取位姿 (转为相对于第一帧)         │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ 第一帧高斯初始化                      │
│ 1. 深度图 → 3D点云 (反投影)           │
│ 2. 点云 → 初始高斯 (位置+颜色)        │
│ 3. 深度 → 高斯尺度初始化             │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ 主循环处理                           │
│ for frame in frames[1:]:            │
│     追踪 → 关键帧判断 → 建图          │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│ 输出                                │
│ 1. 高斯参数 (means3D, colors, ...)  │
│ 2. 相机轨迹 (每帧位姿)              │
│ 3. 可选: PLY文件 (用于可视化)        │
└─────────────────────────────────────┘
```

---

## 七、损失函数设计

### 7.1 追踪损失

```
L_tracking = w_im * ||I_rendered - I_gt||_1
           + w_depth * ||D_rendered - D_gt||_1
```

其中：
- `w_im`: RGB损失权重
- `w_depth`: 深度损失权重
- 使用剪影掩码过滤无效区域

### 7.2 建图损失

```
L_mapping = 0.8 * ||I_rendered - I_gt||_1
          + 0.2 * (1 - SSIM(I_rendered, I_gt))
          + w_depth * ||D_rendered - D_gt||_1
```

其中：
- `0.8`: L1损失权重
- `0.2`: SSIM损失权重
- `SSIM`: 结构相似性度量

### 7.3 损失掩码

```python
# 创建有效掩码
def create_mask(silhouette, depth, gt_depth):
    # 剪影掩码: 高斯覆盖概率 > 阈值
    sil_mask = silhouette > 0.99
    
    # 深度掩码: 有效深度范围
    depth_mask = (gt_depth > 0) & (gt_depth < max_depth)
    
    # 组合掩码
    valid_mask = sil_mask & depth_mask
    
    return valid_mask
```

---

## 八、优化策略

### 8.1 学习率设置

```python
# 追踪阶段 (仅优化位姿)
tracking_lr = {
    'cam_unnorm_rots': 0.0004,   # 旋转学习率
    'cam_trans':       0.002     # 平移学习率
}

# 建图阶段 (仅优化高斯)
mapping_lr = {
    'means3D':         0.0001,   # 位置学习率
    'rgb_colors':      0.0025,   # 颜色学习率
    'unnorm_rotations':0.001,    # 旋转学习率
    'logit_opacities': 0.05,     # 不透明度学习率
    'log_scales':      0.001     # 尺度学习率
}
```

### 8.2 优化器

- 使用Adam优化器
- 追踪阶段: 300次迭代
- 建图阶段: 500次迭代

### 8.3 正则化

- **不透明度重置**: 周期性将不透明度重置为0.01
- **尺度裁剪**: 移除过大的高斯 (scale > 0.1 * scene_radius)
- **近邻约束**: 使用FAISS进行近邻搜索

---

## 九、关键配置参数

### 9.1 高斯初始化参数

```python
gaussian_init = {
    'init_scale_factor': 0.5,      # 初始尺度因子
    'init_opacity': 0.1,           # 初始不透明度
    'gaussian_distribution': 'isotropic',  # 各向同性/各向异性
}
```

### 9.2 追踪参数

```python
tracking_config = {
    'num_iters': 300,              # 优化迭代次数
    'w_im': 1.0,                   # RGB损失权重
    'w_depth': 0.5,                # 深度损失权重
    'silhouette_threshold': 0.99,  # 剪影阈值
}
```

### 9.3 建图参数

```python
mapping_config = {
    'num_iters': 500,              # 优化迭代次数
    'densify_interval': 100,       # 稠密化间隔
    'prune_interval': 100,         # 剪枝间隔
    'reset_opacity_interval': 3000,# 不透明度重置间隔
}
```

### 9.4 稠密化参数

```python
densify_config = {
    'densify_threshold': 0.0002,   # 梯度阈值
    'scene_radius_thresh': 0.01,   # 场景半径阈值
    'prune_opacity_thresh': 0.005, # 不透明度剪枝阈值
    'prune_scale_thresh': 0.1,     # 尺度剪枝阈值
}
```

---

## 十、后处理与可视化

### 10.1 后处理优化

```bash
python scripts/post_splatam_opt.py configs/<dataset>/post_splatam_opt.py
```

后处理阶段在SplaTAM重建的基础上进行额外的3DGS优化：
- 使用所有帧进行训练
- 不进行稠密化/剪枝
- 进一步提升渲染质量

### 10.2 PLY导出

```bash
python scripts/export_ply.py configs/<dataset>/splatam.py
```

导出的PLY文件可以在以下工具中可视化：
- [SuperSplat](https://playcanvas.com/supersplat/editor)
- [PolyCam](https://poly.cam/tools/gaussian-splatting)

### 10.3 交互式可视化

```bash
# 最终重建可视化
python viz_scripts/final_recon.py configs/<dataset>/splatam.py

# 在线重建可视化
python viz_scripts/online_recon.py configs/<dataset>/splatam.py
```

---

## 十一、性能特点

### 11.1 优势

1. **稠密表示**: 使用高斯实现稠密场景重建
2. **实时性能**: 支持在线SLAM
3. **可微渲染**: 端到端可微分优化
4. **灵活输出**: 支持多种可视化格式

### 11.2 计算复杂度

- **内存**: O(N)，N为高斯数量
- **追踪**: O(N * K)，K为迭代次数
- **建图**: O(N * K * M)，M为关键帧数量
- **渲染**: O(N)，高斯光栅化复杂度

### 11.3 典型运行时间

| 数据集 | 帧数 | 高斯数 | 运行时间 |
|--------|------|--------|----------|
| Replica | 1000 | ~100K | ~5分钟 |
| TUM-RGBD | 1000 | ~50K | ~3分钟 |
| ScanNet | 1000 | ~200K | ~10分钟 |

---

## 十二、扩展与应用

### 12.1 iPhone在线演示

```bash
# 使用NeRFCapture应用
bash bash_scripts/online_demo.bash configs/iphone/online_demo.py
```

### 12.2 Docker部署

```bash
docker pull nkeetha/splatam:v1
bash bash_scripts/start_docker.bash
```

### 12.3 新视角合成评估

```bash
python scripts/eval_novel_view.py configs/<dataset>/eval_novel_view.py
```

---

## 十三、参考文献

1. **SplaTAM**: Keetha et al., "SplaTAM: Splat, Track & Map 3D Gaussians for Dense RGB-D SLAM", CVPR 2024
2. **3D Gaussian Splatting**: Kerbl et al., "3D Gaussian Splatting for Real-Time Radiance Field Rendering", SIGGRAPH 2023
3. **Dynamic 3D Gaussians**: Luiten et al., "Dynamic 3D Gaussians: Tracking by Persistent Dynamic View Synthesis", 3DV 2024
4. **GradSLAM**: Murthy et al., "GradSLAM: Differentiable SLAM for Computer Vision", CVPR 2022

---

*本文档最后更新于 2026年8月*