# QGstudio 考核项目（图形组 / 软件组）

基于 Unity 6 的点云高斯泼溅（3D Gaussian Splatting）渲染项目。

## 环境 / Unity 版本

| 工程 | Unity 版本 | 说明 |
|---|---|---|
| `QGstudio_SummerCampPlus` | 6000.3.10f1（Unity 6） | 主工程，高斯泼溅渲染 |
| `QGstudio_SummerCamp` | 2022.3.62f3c1（LTS） | 早期原型（点云渲染，待废弃） |

## 目录结构

```
Project/
├── QGstudio_SummerCampPlus/    Unity 6 主工程
├── QGstudio_SummerCamp/        Unity 2022 原型
├── UnityGaussianSplatting/     三方开源库（魔改副本，见下）
├── data/                       原始点云下载区（不入库）
└── tools/                      数据下载脚本
```

## 三方库说明

`UnityGaussianSplatting/` 是开源项目 **aras-p/UnityGaussianSplatting** 的魔改副本，已随本仓库做版本管理。
- 上游地址：`https://github.com/aras-p/UnityGaussianSplatting.git`
- 上游基线：commit `2c6fed3`（Merge pull request #204）
- 主工程 `QGstudio_SummerCampPlus/Packages/manifest.json` 以相对路径 `file:../../UnityGaussianSplatting/package` 引用它，**请保持该相对布局**，否则工程无法打开。

## 数据恢复流程（重要）

本仓库**不包含**原始点云（`.ply`）和高斯泼溅生成数据（`Assets/GaussianAssets/`），克隆后场景中的高斯对象会提示缺少数据——这是预期的。需要以下步骤恢复：

1. 下载点云：
   ```bash
   python tools/bicycle_zip_extract.py
   # 或 v2 版本（无代理逻辑）：
   python tools/remote_zip_extract.py
   ```
   下载到的 `.ply` 会放到 `data/`。
2. 把需要的 `.ply` 拷入 `QGstudio_SummerCampPlus/Assets/Data/`。
3. 在 Unity 中选中 `.ply` 重新导入，即可生成 `Assets/GaussianAssets/`（高斯泼溅数据）。
4. `Library/`、`Temp/` 等缓存目录不入库，Unity 打开工程时自动重建。

## 说明

- 本分支仅包含代码 / 场景 / 设置 / 脚本，仓库约 12MB，无大体积二进制资产。
- 数据资产（点云、生成的高斯泼溅）由需要的人按上述流程重新下载/生成，避免仓库体积膨胀。
