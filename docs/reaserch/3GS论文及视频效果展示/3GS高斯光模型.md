下面是一份仅定义“算法组交付什么”的版本，不约束图形组的 Unity 数据格式、渲染实现或加载方案。

# 3DGS 场景数据交付说明

**交付方**：重建算法组  
**接收方**：图形组  
**交付内容**：无人机场景重建完成后的 3D Gaussian Splatting 场景数据及必要元数据。

## 1. 交付目录

```text
<scene_id>/
├── point_cloud.ply
├── scene_metadata.json
├── georeference.json
├── cameras.json
├── preview/
│   ├── overview.jpg
│   └── validation/
│       ├── validation_cameras.json
│       ├── view_001.png
│       └── ...
├── quality_report.json
└── checksums.sha256
```

| 文件 | 必选 | 说明 |
|---|---:|---|
| `point_cloud.ply` | 是 | 训练完成后的原始 3DGS Gaussian 数据 |
| `scene_metadata.json` | 是 | 数据编码、坐标、SH、训练和统计信息 |
| `georeference.json` | 是 | 本地坐标与真实地理坐标的映射关系 |
| `cameras.json` | 是 | 训练图像对应的相机位姿与内参，用于调试和坐标校验 |
| `preview/validation/` | 是 | 标准验证视角与算法侧参考渲染图 |
| `quality_report.json` | 建议 | 重建覆盖、剔除帧、质量风险等 |
| `checksums.sha256` | 建议 | 完整性校验 |

---

## 2. 核心场景文件：`point_cloud.ply`

文件采用标准 PLY 格式：

```text
format binary_little_endian 1.0
```

每个 `vertex` 表示一个三维 Gaussian，而不是传统 Mesh 顶点。

### 必须包含的字段

```ply
property float x
property float y
property float z

property float f_dc_0
property float f_dc_1
property float f_dc_2

property float f_rest_0
...
property float f_rest_N

property float opacity

property float scale_0
property float scale_1
property float scale_2

property float rot_0
property float rot_1
property float rot_2
property float rot_3
```

若训练框架默认带有以下字段，也一并保留：

```ply
property float nx
property float ny
property float nz
```

但这些并非可靠表面法线，图形组不应将其作为 Mesh 法线使用。

### 字段语义

| 字段 | 含义 | 单位 / 范围 |
|---|---|---|
| `x, y, z` | Gaussian 中心位置 | 米 |
| `scale_0..2` | Gaussian 三轴尺度训练参数 | 默认是 `log(scale)` |
| `rot_0..3` | Gaussian 旋转四元数训练参数 | 见元数据中的顺序说明 |
| `opacity` | Gaussian 不透明度训练参数 | 默认是 sigmoid 前的 logit |
| `f_dc_0..2` | RGB 的零阶球谐颜色系数 | 浮点数 |
| `f_rest_*` | 高阶 SH 颜色系数 | 浮点数 |
| `nx, ny, nz` | PLY 兼容字段 | 不作为有效法线 |

### 参数解码要求

默认 3DGS 训练输出中，以下值处于训练域，并非直接可用于渲染：

```text
linear_scale = exp(scale_raw)

linear_opacity = sigmoid(opacity_raw)

rotation = normalize(quaternion_raw)
```

其中：

```text
sigmoid(v) = 1 / (1 + exp(-v))
```

如交付的 PLY 已提前解码，必须在 `scene_metadata.json` 中明确标记，避免重复解码。

---

## 3. 场景元数据：`scene_metadata.json`

示例：

```json
{
  "scene_id": "uav_site_20260315_001",
  "format_version": "1.0",
  "created_at_utc": "2026-03-15T08:30:00Z",

  "source": {
    "reconstruction_method": "3D Gaussian Splatting",
    "training_framework": "inria_3dgs",
    "image_count": 1842,
    "camera_pose_source": "RTK_GNSS_INS"
  },

  "coordinates": {
    "coordinate_system": "Local ENU",
    "unit": "meter",
    "axis_order": ["East", "Up", "North"],
    "world_to_unity_matrix_row_major": [
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    ]
  },

  "gaussian_data": {
    "gaussian_count": 5839201,

    "position_encoding": "float32_meter",
    "scale_encoding": "log_scale",
    "scale_decode": "exp",

    "opacity_encoding": "logit",
    "opacity_decode": "sigmoid",

    "quaternion_order": ["qw", "qx", "qy", "qz"],
    "quaternion_normalization_required": true,

    "sh_degree": 3,
    "sh_coefficients_per_channel": 16,
    "sh_color_channels": ["R", "G", "B"],
    "sh_basis": "real_spherical_harmonics",
    "sh_source_layout": "native_ply_property_order",

    "color_space": "linear_srgb"
  },

  "bounds_enu_m": {
    "min": [-120.2, -8.5, -95.0],
    "max": [135.7, 74.1, 110.3]
  },

  "filtering": {
    "opacity_threshold": 0.005,
    "invalid_gaussians_removed": 1293
  }
}
```

### 特别说明

`world_to_unity_matrix_row_major` 示例中的第三行应为：

```json
[0, 0, 1, 0]
```

完整单位矩阵如下：

```json
[
  1, 0, 0, 0,
  0, 1, 0, 0,
  0, 0, 1, 0,
  0, 0, 0, 1
]
```

如果数据坐标系已经按 ENU 映射为 Unity 的 `X=East, Y=Up, Z=North`，该矩阵为单位矩阵。

---

## 4. 坐标与地理参考：`georeference.json`

所有 Gaussian 的 `x/y/z` 使用本地 ENU 米制坐标，而不是直接使用经纬度、UTM 大坐标或飞控 NED 坐标。

坐标约定：

```text
X = East，东
Y = Up，天
Z = North，北
单位 = meter
```

这与约定的 Unity 坐标直接对应：

```text
Unity X = East
Unity Y = Up
Unity Z = North
```

`georeference.json` 示例：

```json
{
  "coordinate_system": "Local ENU",
  "unit": "meter",
  "origin_wgs84": {
    "latitude_deg": 31.230400,
    "longitude_deg": 121.473700,
    "ellipsoidal_height_m": 12.850
  },
  "axis_definition": {
    "x": "east",
    "y": "up",
    "z": "north"
  },
  "vertical_datum": "WGS84 ellipsoidal height",
  "notes": "All Gaussian centers and camera positions are expressed relative to this ENU origin."
}
```

要求：

- 所有 Gaussian、相机轨迹、参考渲染相机必须处于同一局部 ENU 坐标系。
- 场景原点选择起飞点、任务区域中心或指定控制点。
- 保留 `origin_wgs84`，以支持从 Unity 内部局部坐标反查真实地理位置。
- 若使用正高而非椭球高，必须写明垂直基准和大地水准面模型。

---

## 5. 相机数据：`cameras.json`

用于图形组校验场景位置、方向、坐标轴和参考视角，不要求图形组用于运行时渲染。

```json
{
  "coordinate_system": "Local ENU",
  "pose_convention": "camera_to_world",
  "quaternion_order": ["qw", "qx", "qy", "qz"],
  "cameras": [
    {
      "image_name": "frame_000001.jpg",
      "timestamp_ns": 1710000000000000000,

      "position_enu_m": [12.45, 38.20, -5.80],
      "rotation_camera_to_world_qwxyz": [
        0.9238795,
        0.0,
        0.3826834,
        0.0
      ],

      "image_width_px": 3840,
      "image_height_px": 2160,
      "fx_px": 2500.0,
      "fy_px": 2500.0,
      "cx_px": 1920.0,
      "cy_px": 1080.0,

      "distortion_removed": true,
      "rtk_status": "FIXED"
    }
  ]
}
```

必须明确：

- 位姿矩阵或四元数是 `camera_to_world` 还是 `world_to_camera`；
- 四元数存储顺序；
- 图像是否已经去畸变；
- 相机内参对应原图还是去畸变后的图像；
- 位姿是否经过图像优化，以及优化幅度。

---

## 6. 标准验证数据

为确认 Unity 显示结果与算法侧结果坐标一致、颜色一致、方向一致，交付至少 `10` 个标准视角。

目录：

```text
preview/validation/
├── validation_cameras.json
├── view_001.png
├── view_002.png
└── ...
```

`validation_cameras.json` 中每个视角需包含：

```json
{
  "view_id": "view_001",
  "position_enu_m": [10.0, 25.0, -15.0],
  "rotation_camera_to_world_qwxyz": [1.0, 0.0, 0.0, 0.0],
  "width_px": 1920,
  "height_px": 1080,
  "fx_px": 1200.0,
  "fy_px": 1200.0,
  "cx_px": 960.0,
  "cy_px": 540.0,
  "reference_image": "view_001.png",
  "render_mode": "sh_degree_3"
}
```

验证图应覆盖：

- 俯视正射视角；
- 建筑或目标物体的斜视视角；
- 近景；
- 远景；
- 场景边界；
- 高斯密集区域；
- 植被、细杆、立面边缘等容易产生伪影的区域。

---

## 7. 重建质量报告：`quality_report.json`

建议交付以下质量信息，供图形组判断是否属于渲染问题或源数据问题。

```json
{
  "image_count_input": 2000,
  "image_count_used": 1842,
  "image_count_excluded": 158,

  "gaussian_count_before_filter": 5840494,
  "gaussian_count_after_filter": 5839201,

  "scene_bounds_enu_m": {
    "min": [-120.2, -8.5, -95.0],
    "max": [135.7, 74.1, 110.3]
  },

  "known_limitations": [
    {
      "region_enu_m": {
        "min": [20.0, 0.0, -10.0],
        "max": [45.0, 20.0, 15.0]
      },
      "reason": "Vegetation movement during capture"
    },
    {
      "region_enu_m": {
        "min": [-30.0, 5.0, 50.0],
        "max": [-10.0, 30.0, 70.0]
      },
      "reason": "Insufficient side-view coverage"
    }
  ]
}
```

---

## 8. 交付前最低校验

算法组在交付前保证：

- PLY 文件可正常解析，且采用小端二进制格式；
- 所有 Gaussian 的位置、尺度、旋转、透明度和 SH 系数均为有限值，无 `NaN` / `Inf`；
- 高斯尺度解码后为正值；
- 四元数可归一化；
- 明确 `opacity`、`scale` 的编码方式；
- 明确 SH 阶数、颜色空间和 PLY 系数排列；
- 所有数据使用同一个局部 ENU 坐标系；
- `cameras.json` 中相机位置与 PLY 场景空间对齐；
- 参考视角渲染图可由算法侧稳定复现；
- 对场景中的已知缺失、动态物体、模糊或覆盖不足区域有说明。

图形组收到的数据核心上只需要无歧义地回答五件事：

1. 每个 Gaussian 在哪里：`x, y, z`。  
2. 它的空间形状和朝向：`scale_0..2`、`rot_0..3`。  
3. 它的透明度：`opacity` 及其解码方式。  
4. 它在不同观察方向下的颜色：`f_dc_*`、`f_rest_*`、SH 约定。  
5. 它在真实场地中的位置：局部 ENU 原点与 WGS84 地理参考。