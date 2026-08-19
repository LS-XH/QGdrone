# RGB-D 相机适配方案

本文档说明如何使用深度相机（RGB-D）数据替代 COLMAP SfM，跳过耗时的特征提取和匹配步骤。

---

## 1. 为什么可以跳过 COLMAP

| | COLMAP SfM | 深度相机 |
|---|---|---|
| 输入 | 多张 2D 图像 | 单帧 RGB + Depth |
| 3D 点来源 | 多视图三角化 | 直接测量 Z |
| 外参来源 | 估计各图像位姿 | 已知（传感器标定） |
| 精度 | 受匹配质量影响 | 受传感器噪声影响 |
| 速度 | 慢（分钟~小时） | 快（实时采集） |
| 适用场景 | 任意手持拍摄 | 需要专门硬件 |

**结论：** 如果你有深度相机（如 RealSense、Azure Kinect、iPhone LiDAR 等），可以直接获取 RGB + Depth + Poses，完全跳过 COLMAP。

---

## 2. 最小可用输入

对于深度相机数据，创建 Camera 对象的最小需要：

```python
import math
import numpy as np
from PIL import Image
from scene.cameras import Camera

camera = Camera(
    resolution=(W, H),
    colmap_id=0,
    R=rotation_matrix,       # 3×3 np.ndarray
    T=translation_vector,    # 3,  np.ndarray
    FoVx=2 * math.atan(W / (2 * fx)),
    FoVy=2 * math.atan(H / (2 * fy)),
    depth_params=None,       # 如果深度已对齐
    image=pil_image,         # PIL.Image
    invdepthmap=depth_map,   # np.ndarray, float32, 16-bit 或 float32
    image_name="frame_0001.png",
    uid=0
)
```

---

## 3. 数据准备流程

### 3.1 目录结构

```
your_dataset/
├── images/              # RGB 图像
│   ├── frame_0001.png
│   ├── frame_0002.png
│   └── ...
├── depths/              # 深度图（可选但推荐）
│   ├── frame_0001.png
│   ├── frame_0002.png
│   └── ...
└── poses.json           # 相机位姿
```

### 3.2 poses.json 格式

```json
{
    "fx": 616.36,
    "fy": 616.36,
    "cx": 320.0,
    "cy": 240.0,
    "width": 640,
    "height": 480,
    "frames": [
        {
            "file_path": "frame_0001.png",
            "depth_path": "frame_0001.png",
            "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "T": [0, 0, 0]
        },
        {
            "file_path": "frame_0002.png",
            "depth_path": "frame_0002.png",
            "R": [[0.99, 0, 0.1], [0, 1, 0], [-0.1, 0, 0.99]],
            "T": [0.1, 0, 0]
        }
    ]
}
```

---

## 4. 从深度图生成点云

### 4.1 单帧深度图 → 3D 点

```python
import numpy as np

def depth_to_pointcloud(depth_map, fx, fy, cx, cy, R, T):
    """
    将单帧深度图转换为世界坐标系下的 3D 点云
    
    Args:
        depth_map: (H, W) 深度图，单位：米
        fx, fy: 焦距（像素）
        cx, cy: 主点（像素）
        R: (3, 3) 旋转矩阵
        T: (3,) 平移向量
    
    Returns:
        points_world: (N, 3) 世界坐标系下的 3D 点
        colors: (N, 3) 对应的 RGB 颜色
    """
    H, W = depth_map.shape
    
    # 生成像素网格
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    
    # 深度有效掩码
    valid_mask = depth_map > 0
    
    # 反投影到相机坐标系
    Z = depth_map[valid_mask]
    X = (u[valid_mask] - cx) * Z / fx
    Y = (v[valid_mask] - cy) * Z / fy
    
    # 相机坐标系下的点
    points_camera = np.stack([X, Y, Z], axis=-1)  # (N, 3)
    
    # 转换到世界坐标系
    # 注意：本项目的 R, T 是 world-to-camera，所以需要逆变换
    points_world = (points_camera - T) @ R  # (N, 3)
    
    return points_world
```

### 4.2 多帧点云合并

```python
import numpy as np
from plyfile import PlyData, PlyElement

def merge_pointclouds(all_points, all_colors):
    """
    合并多帧点云并去重
    """
    merged_points = np.concatenate(all_points, axis=0)
    merged_colors = np.concatenate(all_colors, axis=0)
    
    # 简单去重：体素网格下采样
    voxel_size = 0.01  # 1cm
    voxel_indices = np.floor(merged_points / voxel_size).astype(int)
    
    # 使用字典去重
    seen = set()
    unique_points = []
    unique_colors = []
    
    for i in range(len(merged_points)):
        key = tuple(voxel_indices[i])
        if key not in seen:
            seen.add(key)
            unique_points.append(merged_points[i])
            unique_colors.append(merged_colors[i])
    
    return np.array(unique_points), np.array(unique_colors)

def save_ply(points, colors, path):
    """
    保存为 PLY 格式
    """
    dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
             ('nx', 'f4'), ('ny', 'f4'), ('nz', 'f4'),
             ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
    
    normals = np.zeros_like(points)
    elements = np.empty(points.shape[0], dtype=dtype)
    attributes = np.concatenate([points, normals, colors * 255], axis=1)
    elements[:] = list(map(tuple, attributes))
    
    el = PlyElement.describe(elements, 'vertex')
    PlyData([el]).write(path)
```

### 4.3 完整的点云生成脚本

```python
import json
import cv2
import numpy as np
from pathlib import Path

def generate_pointcloud_from_rgbd(dataset_path, output_path):
    """
    从 RGB-D 数据集生成点云
    """
    # 读取位姿数据
    with open(f"{dataset_path}/poses.json", "r") as f:
        data = json.load(f)
    
    fx, fy = data["fx"], data["fy"]
    cx, cy = data["cx"], data["cy"]
    
    all_points = []
    all_colors = []
    
    for frame in data["frames"]:
        # 读取 RGB 图像
        image_path = f"{dataset_path}/images/{frame['file_path']}"
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 读取深度图
        depth_path = f"{dataset_path}/depths/{frame['depth_path']}"
        depth = cv2.imread(depth_path, -1).astype(np.float32) / 1000.0  # 假设存储单位是 mm
        
        # 获取位姿
        R = np.array(frame["R"])
        T = np.array(frame["T"])
        
        # 深度图 → 3D 点
        points = depth_to_pointcloud(depth, fx, fy, cx, cy, R, T)
        
        # 获取对应颜色
        H, W = depth.shape
        valid_mask = depth > 0
        colors = image[valid_mask] / 255.0
        
        all_points.append(points)
        all_colors.append(colors)
    
    # 合并点云
    merged_points, merged_colors = merge_pointclouds(all_points, all_colors)
    
    # 保存
    save_ply(merged_points, merged_colors, output_path)
    print(f"Generated {len(merged_points)} points")
```

---

## 5. 自定义 Dataset Reader

### 5.1 在 dataset_readers.py 中添加新函数

```python
def readRGBDSceneInfo(path, depths_folder, white_background=False, eval=False):
    """
    从 RGB-D 数据集读取场景信息
    
    Args:
        path: 数据集根目录（包含 poses.json）
        depths_folder: 深度图子目录名
        white_background: 是否使用白色背景
        eval: 是否评估模式
    
    Returns:
        SceneInfo 对象
    """
    # 读取位姿数据
    with open(os.path.join(path, "poses.json"), "r") as f:
        data = json.load(f)
    
    fx, fy = data["fx"], data["fy"]
    cx, cy = data["cx"], data["cy"]
    width, height = data["width"], data["height"]
    
    # 计算 FoV
    FovX = 2 * math.atan(width / (2 * fx))
    FovY = 2 * math.atan(height / (2 * fy))
    
    # 读取相机信息
    cam_infos = []
    for idx, frame in enumerate(data["frames"]):
        # 读取 RGB 图像
        image_path = os.path.join(path, "images", frame["file_path"])
        image = Image.open(image_path)
        
        # 读取深度图
        depth_path = os.path.join(path, depths_folder, frame["depth_path"])
        invdepthmap = None
        if os.path.exists(depth_path):
            depth = cv2.imread(depth_path, -1).astype(np.float32)
            if depth.max() > 1:  # 假设是 mm 或 16-bit
                depth = depth / 1000.0  # mm → m
            invdepthmap = 1.0 / (depth + 1e-6)
        
        # 获取位姿
        R = np.array(frame["R"])
        T = np.array(frame["T"])
        
        # 分割训练/测试
        is_test = idx % 8 == 0 if eval else False
        
        cam_info = CameraInfo(
            uid=idx,
            R=R,
            T=T,
            FovY=FovY,
            FovX=FovX,
            depth_params=None,
            image_path=image_path,
            image_name=frame["file_path"],
            depth_path=depth_path if invdepthmap is not None else "",
            width=width,
            height=height,
            is_test=is_test
        )
        cam_infos.append(cam_info)
    
    # 分割训练集和测试集
    train_cam_infos = [c for c in cam_infos if not c.is_test]
    test_cam_infos = [c for c in cam_infos if c.is_test]
    
    # 生成或读取点云
    ply_path = os.path.join(path, "points3d.ply")
    if not os.path.exists(ply_path):
        print("Generating point cloud from depth maps...")
        generate_pointcloud_from_rgbd(path, ply_path)
    
    pcd = fetchPly(ply_path)
    
    # 计算归一化参数
    nerf_normalization = getNerfppNorm(train_cam_infos)
    
    return SceneInfo(
        point_cloud=pcd,
        train_cameras=train_cam_infos,
        test_cameras=test_cam_infos,
        nerf_normalization=nerf_normalization,
        ply_path=ply_path,
        is_nerf_synthetic=False
    )
```

### 5.2 注册新的 reader

在 `dataset_readers.py` 末尾添加：

```python
sceneLoadTypeCallbacks = {
    "Colmap": readColmapSceneInfo,
    "Blender": readNerfSyntheticInfo,
    "RGBD": readRGBDSceneInfo  # 新增
}
```

### 5.3 在 Scene.__init__ 中添加判断

修改 `scene/__init__.py`：

```python
if os.path.exists(os.path.join(args.source_path, "sparse")):
    scene_info = sceneLoadTypeCallbacks["Colmap"](args.source_path, args.images, args.depths, args.eval, args.train_test_exp)
elif os.path.exists(os.path.join(args.source_path, "transforms_train.json")):
    print("Found transforms_train.json file, assuming Blender data set!")
    scene_info = sceneLoadTypeCallbacks["Blender"](args.source_path, args.white_background, args.depths, args.eval)
elif os.path.exists(os.path.join(args.source_path, "poses.json")):
    print("Found poses.json file, assuming RGBD data set!")
    scene_info = sceneLoadTypeCallbacks["RGBD"](args.source_path, args.depths, args.white_background, args.eval)
else:
    assert False, "Could not recognize scene type!"
```

---

## 6. 训练命令

```bash
# 使用 RGB-D 数据集训练
python train.py \
    -s /path/to/your/rgbd_dataset \
    -m /path/to/output \
    --eval
```

---

## 7. 常见深度相机参数参考

| 相机 | 分辨率 | 深度范围 | fx | fy | cx | cy |
|------|--------|----------|-----|-----|-----|-----|
| RealSense D435i | 640×480 | 0.2-10m | 616 | 616 | 320 | 240 |
| Azure Kinect | 640×576 | 0.5-5.46m | 504 | 504 | 323 | 324 |
| iPhone LiDAR | 1920×1440 | 0.2-5m | ~1200 | ~1200 | 960 | 720 |

---

## 8. 注意事项

1. **坐标系对齐**：确保你的深度相机数据是 COLMAP 坐标系（Y-down, Z-forward）
2. **深度单位**：注意深度图的单位（mm, cm, m），需要统一转换为米
3. **深度对齐**：如果深度图有系统误差，使用 `depth_params` 进行线性对齐
4. **点云质量**：深度相机在远距离和反光表面噪声较大，建议设置深度阈值
5. **内存管理**：多帧点云合并时注意内存占用，建议使用体素下采样
