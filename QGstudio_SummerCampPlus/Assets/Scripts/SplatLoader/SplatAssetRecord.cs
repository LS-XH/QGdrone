// SPDX-License-Identifier: MIT
// ============================================================================
// SplatAssetRecord.cs —— 缓存元数据（meta.json 结构，简报 §五 定稿）
// 约束：JsonUtility 可序列化 → 全 public 字段；Hash128 用 ToString/Parse（Unity 6000 无公开 u0-u3 字段）；bounds 拆 6×float
// 与 GaussianSplatPipeline.ConvertResult 字段一一映射（数据流对齐铁律：读回重组
// 后传 Factory.Build 的参数与转换路径完全一致）。
// ============================================================================
using System;
using GaussianSplatting.Runtime;

namespace QGStudio.SplatLoader
{
    /// <summary>缓存元数据（meta.json）。全 public 字段，JsonUtility 直接序列化。</summary>
    [Serializable]
    public class SplatAssetRecord
    {
        public int formatVersion;                          // = GaussianSplatAsset.kCurrentVersion（不兼容 → 清缓存重转）
        public string sceneId;                             // 目录名（Hash(路径)），读回校验一致性
        public string sourcePath;                          // 仅调试，不参与查找
        public int splatCount;                             // 点数
        public GaussianSplatAsset.VectorFormat formatPos;  // 4 formats（枚举，JsonUtility 存数字）
        public GaussianSplatAsset.VectorFormat formatScale;
        public GaussianSplatAsset.ColorFormat formatColor;
        public GaussianSplatAsset.SHFormat formatSH;
        public float boundsMinX, boundsMinY, boundsMinZ;   // boundsMin（Vector3 拆 3 float）
        public float boundsMaxX, boundsMaxY, boundsMaxZ;   // boundsMax
        public string dataHash;                            // Hash128.ToString()（32 位 hex；仅供重建 SO，不用于校验）
        public long fileSizeChunk = -1;                    // 校验凭据（大小）：-1 = 无 chunk bin
        public long fileSizePos;
        public long fileSizeOther;
        public long fileSizeColor;
        public long fileSizeSH;
        public ulong hashChunk;                            // 校验凭据（内容 FNV-1a 64）：0 = 无该 bin
        public ulong hashPos;
        public ulong hashOther;
        public ulong hashColor;
        public ulong hashSH;
        public bool hasChunk;                              // chunk bin 是否存在（Medium 档恒 true）
        public string timestamp;                           // ISO 8601，记录用
    }
}
