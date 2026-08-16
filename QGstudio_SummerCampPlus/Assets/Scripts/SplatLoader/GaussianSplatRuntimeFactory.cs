// SPDX-License-Identifier: MIT
// ============================================================================
// GaussianSplatRuntimeFactory.cs —— 构建运行时 SO（简报 §四.4 / 设计 §4）
// 构建方式参照 SmokeTest：CreateInstance + Initialize(同参数) + SetDataHash + SetRuntimeData
// 输入：GaussianSplatPipeline.ConvertResult（5 通道 byte[] + 元数据）
// 输出：GaussianSplatAsset（内存 SO，未落盘；阶段 2a 缓存直读走同一入口）
// ============================================================================
using GaussianSplatting.Runtime;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    public class GaussianSplatRuntimeFactory
    {
        public GaussianSplatAsset Build(GaussianSplatPipeline.ConvertResult result)
        {
            var asset = ScriptableObject.CreateInstance<GaussianSplatAsset>();
            asset.name = $"RuntimeSplat_{result.splatCount}";
            // Initialize 参数与 Creator 297 行完全一致（8 参数：count + 4 formats + bounds + cameras）
            asset.Initialize(
                result.splatCount,
                result.formatPos,
                result.formatScale,
                result.formatColor,
                result.formatSH,
                result.boundsMin,
                result.boundsMax,
                result.cameras);
            asset.SetDataHash(result.dataHash);
            // SetRuntimeData 参数顺序：pos, other, color, sh, chunk（GaussianSplatAsset 238 行）
            asset.SetRuntimeData(
                result.posData,
                result.otherData,
                result.colorData,
                result.shData,
                result.chunkData);
            return asset;
        }
    }
}
