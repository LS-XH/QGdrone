// SPDX-License-Identifier: MIT
// ============================================================================
// SplatLoadingManager.cs —— 异步编排（简报 §四.2 / 设计 §4）
// 方案 B（用户拍板）：解析分两段——Step1(IO+纯函数) 后台线程 Task.Run，
//                      Step2(LinearizeDataJob) 主线程；其余步骤全主线程 Job。
// ⚠️ Job.Schedule 主线程限定 → 本方法必须在主线程发起调用
//    （await 后经 Unity SynchronizationContext 自动回主线程）。
// 进度分段（简报 §六.5）：0-20 解析 / 20-60 加工 / 60-90 编码 / 90-100 落盘+挂载
//    （90-100 由 Service 在 Factory/Binder 后补报）。
// 数据流与 Creator CreateAsset 完全一致：解析→bounds→Morton→[聚类]→chunk→pos→oth→col→shs
// ============================================================================
using System;
using System.Threading.Tasks;
using GaussianSplatting.Runtime;
using Unity.Collections;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    public class SplatLoadingManager
    {
        // 质量档固定 Medium（阶段 1；阶段 3a 质量分级再参数化）
        GaussianSplatPipeline.DataQuality m_Quality = GaussianSplatPipeline.DataQuality.Medium;
        public GaussianSplatPipeline.DataQuality Quality
        {
            get => m_Quality;
            set => m_Quality = value;
        }

        /// <summary>进度事件（0~1，主线程回调）</summary>
        public event Action<float> ProgressChanged;
        /// <summary>错误事件（转换失败时触发）</summary>
        public event Action<string> ErrorOccurred;

        /// <summary>
        /// 异步转换：PLY → ConvertResult（5 通道 byte[] + 元数据）。
        /// 必须主线程调用。失败时返回 errorMessage 非空的结果并触发 ErrorOccurred。
        /// </summary>
        public async Task<GaussianSplatPipeline.ConvertResult> ConvertAsync(string plyPath)
        {
            var pipeline = new GaussianSplatPipeline();
            pipeline.ApplyQualityLevel(m_Quality);

            // Creator:264 相机 json（可选，小文件主线程读取）
            var cameras = pipeline.LoadCameras(plyPath);

            // 聚类内部进度（Creator 数值 0.2~0.7）映射进加工区间 20-60%
            pipeline.SetProgressCallback(clusterVal =>
                ProgressChanged?.Invoke(0.2f + (clusterVal - 0.2f) / 0.5f * 0.4f));

            // ===== 0-20% 解析 =====
            // 方案 F：IO 段（读 1.5GB 文件）后台 Task.Run；解析段（Allocator.Temp 主线程限定）主线程。
            ProgressChanged?.Invoke(0.05f);
            var io = await Task.Run(() => pipeline.LoadInputFileStep1(plyPath)); // 后台：读盘 + 属性校验（无 Temp 无 Job）
            if (!io.plyRawData.IsCreated || pipeline.LastErrorMessage != null)
            {
                string err = pipeline.LastErrorMessage ?? "PLY 解析失败或文件为空";
                ErrorOccurred?.Invoke(err);
                return ErrorResult(err);
            }
            ProgressChanged?.Invoke(0.12f);
            await Task.Yield(); // 回主线程，让 UI 刷新
            NativeArray<InputSplatData> splats;
            try
            {
                splats = pipeline.LoadInputFileStep2(io.plyRawData, io.splatCount, io.vertexStride, io.attributes); // 主线程：解析 + SH 重排 + Linearize
            }
            catch (Exception ex)
            {
                io.plyRawData.Dispose(); // 解析失败：释放 IO 段 Persistent 数据
                ErrorOccurred?.Invoke(ex.Message);
                return ErrorResult(ex.Message);
            }
            io.plyRawData.Dispose(); // Persistent 用完即释（原版 ReadFile 漏释 Persistent，这里补上）
            if (!splats.IsCreated || splats.Length == 0)
            {
                string err = "PLY 解析失败或文件为空";
                ErrorOccurred?.Invoke(err);
                return ErrorResult(err);
            }
            int splatCount = splats.Length; // 在 Dispose 前保存
            await Task.Yield();
            ProgressChanged?.Invoke(0.2f);

            // ===== 20-60% 加工 =====
            if (SplatCoordinateNormalizer.Enabled) // 简报 §四.6：减质心，默认开，须在 Morton 之前
                SplatCoordinateNormalizer.Normalize(splats);
            await Task.Yield();
            ProgressChanged?.Invoke(0.25f);
            var (boundsMin, boundsMax) = pipeline.CalcBounds(splats); // 主线程 Job
            await Task.Yield();
            ProgressChanged?.Invoke(0.3f);
            pipeline.ReorderMorton(splats, boundsMin, boundsMax); // 主线程 Job
            await Task.Yield();
            ProgressChanged?.Invoke(0.4f);
            pipeline.ClusterSHs(splats, out var clusteredSHs, out var splatSHIndices); // Medium 不触发
            await Task.Yield();

            // ===== 60-90% 编码（顺序与 Creator 310-314 一致）=====
            // Creator:300 hash 初始化（formatVersion == kCurrentVersion，数值等价）
            var dataHash = new Hash128((uint)splatCount, (uint)GaussianSplatAsset.kCurrentVersion, 0, 0);
            byte[] chunkData = null;
            if (pipeline.isUsingChunks) // Creator:308-310
            {
                ProgressChanged?.Invoke(0.5f);
                chunkData = pipeline.CreateChunkData(splats, ref dataHash); // 主线程 Job
                await Task.Yield();
            }
            ProgressChanged?.Invoke(0.6f);
            byte[] posData = pipeline.CreatePositionsData(splats, ref dataHash);
            await Task.Yield();
            ProgressChanged?.Invoke(0.7f);
            byte[] otherData = pipeline.CreateOtherData(splats, ref dataHash, splatSHIndices);
            await Task.Yield();
            ProgressChanged?.Invoke(0.8f);
            byte[] colorData = pipeline.CreateColorData(splats, ref dataHash);
            await Task.Yield();
            ProgressChanged?.Invoke(0.9f);
            byte[] shData = pipeline.CreateSHData(splats, ref dataHash, clusteredSHs);

            // 清理 NativeArray（Creator:317-318）
            splats.Dispose();
            splatSHIndices.Dispose();
            clusteredSHs.Dispose();

            // 90-100% 落盘+挂载由 Service 负责（Factory/Binder 后补报）
            return new GaussianSplatPipeline.ConvertResult
            {
                splatCount = splatCount,
                formatPos = pipeline.FormatPos,
                formatScale = pipeline.FormatScale,
                formatColor = pipeline.FormatColor,
                formatSH = pipeline.FormatSH,
                boundsMin = (Vector3)boundsMin,
                boundsMax = (Vector3)boundsMax,
                dataHash = dataHash,
                cameras = cameras,
                chunkData = chunkData,
                posData = posData,
                otherData = otherData,
                colorData = colorData,
                shData = shData,
            };
        }

        static GaussianSplatPipeline.ConvertResult ErrorResult(string message)
        {
            return new GaussianSplatPipeline.ConvertResult { errorMessage = message };
        }
    }
}
