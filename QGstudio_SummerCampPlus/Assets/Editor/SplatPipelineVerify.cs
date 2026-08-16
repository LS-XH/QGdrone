// SPDX-License-Identifier: MIT
// ============================================================================
// SplatPipelineVerify.cs —— 阶段 1 提取正确性验证（最强铁证）
// 两个菜单：
//   Tools/QG Studio/Verify Splat Pipeline (byte-exact)
//       —— ConvertAll 同步路径 vs Creator 产物（回归 Creator 对照）
//   Tools/QG Studio/Verify Split Pipeline (Step1+Step2)
//       —— 步骤化路径（含方案 B 的 ReadFileStep1/Step2 拆分 + Normalizer 关闭）
//         vs Creator 产物（验证 LoadingManager 数据流与 Creator 完全一致）
// 全 PASS = 提取无偏差 / 拆分数据流等价；任一 FAIL = 按差异偏移排查对应函数。
// 黄金基准（阶段 0 实测）：pos=24,527,816 / oth=49,055,632 / col=24,641,536
//                          shs=196,222,528 / chk=1,532,992
// ============================================================================
using System;
using System.IO;
using GaussianSplatting.Runtime;
using QGStudio.SplatLoader;
using Unity.Collections;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

public static class SplatPipelineVerify
{
    const string kPlyRel = "../../data/bicycle_iteration_30000_point_cloud.ply";
    const string kAssetBase = "bicycle_iteration_30000_point_cloud";
    static readonly string[] kChannels = { "chk", "pos", "oth", "col", "shs" };

    // ================= 菜单 1：ConvertAll 同步路径 =================
    [MenuItem("Tools/QG Studio/Verify Splat Pipeline (byte-exact)")]
    public static void Verify()
    {
        var pipeline = new GaussianSplatPipeline();
        pipeline.importCameras = false;
        pipeline.SetProgressCallback(p => EditorUtility.DisplayProgressBar("Verify", $"ConvertAll {p:P0}", p));
        try
        {
            var result = pipeline.ConvertAll(GetPlyPath(), GaussianSplatPipeline.DataQuality.Medium);
            if (!string.IsNullOrEmpty(result.errorMessage))
            {
                Debug.LogError($"[Verify] FAIL: 转换失败 {result.errorMessage}");
                return;
            }
            CompareAllChannels(result);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[Verify] FAIL: 异常 {ex}");
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }
    }

    // ================= 菜单 2：步骤化路径（含方案 F 拆分）================
    [MenuItem("Tools/QG Studio/Verify Split Pipeline (Step1+Step2)")]
    public static void VerifySplit()
    {
        SplatCoordinateNormalizer.Enabled = false; // 归一化会平移坐标，逐字节验证必须关闭
        var pipeline = new GaussianSplatPipeline();
        pipeline.importCameras = false;
        pipeline.ApplyQualityLevel(GaussianSplatPipeline.DataQuality.Medium);
        pipeline.SetProgressCallback(p => EditorUtility.DisplayProgressBar("VerifySplit", $"steps {p:P0}", p));
        try
        {
            // —— 与 SplatLoadingManager.ConvertAsync 相同的步骤顺序（同步版）——
            var io = pipeline.LoadInputFileStep1(GetPlyPath()); // 方案 F Step1（IO 段：读盘 + 属性校验）
            if (!io.plyRawData.IsCreated || pipeline.LastErrorMessage != null)
            {
                Debug.LogError($"[VerifySplit] FAIL: 解析失败 {pipeline.LastErrorMessage}");
                return;
            }
            NativeArray<InputSplatData> splats = pipeline.LoadInputFileStep2(io.plyRawData, io.splatCount, io.vertexStride, io.attributes); // 方案 F Step2（解析 + SH 重排 + Linearize）
            io.plyRawData.Dispose(); // Persistent 用完即释
            if (!splats.IsCreated || splats.Length == 0)
            {
                Debug.LogError($"[VerifySplit] FAIL: 解析失败 {pipeline.LastErrorMessage}");
                return;
            }
            var (boundsMin, boundsMax) = pipeline.CalcBounds(splats);
            pipeline.ReorderMorton(splats, boundsMin, boundsMax);
            pipeline.ClusterSHs(splats, out var clusteredSHs, out var shIndices);

            var dataHash = new Hash128((uint)splats.Length, (uint)GaussianSplatAsset.kCurrentVersion, 0, 0);
            byte[] chunkData = null;
            if (pipeline.isUsingChunks)
                chunkData = pipeline.CreateChunkData(splats, ref dataHash);
            byte[] posData = pipeline.CreatePositionsData(splats, ref dataHash);
            byte[] otherData = pipeline.CreateOtherData(splats, ref dataHash, shIndices);
            byte[] colorData = pipeline.CreateColorData(splats, ref dataHash);
            byte[] shData = pipeline.CreateSHData(splats, ref dataHash, clusteredSHs);

            splats.Dispose();
            shIndices.Dispose();
            clusteredSHs.Dispose();

            var result = new GaussianSplatPipeline.ConvertResult
            {
                splatCount = 0, // 对比用不到
                chunkData = chunkData,
                posData = posData,
                otherData = otherData,
                colorData = colorData,
                shData = shData,
            };
            CompareAllChannels(result);
        }
        catch (Exception ex)
        {
            Debug.LogError($"[VerifySplit] FAIL: 异常 {ex}");
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }
    }

    // ================= 公共：5 通道逐字节对比 =================
    static void CompareAllChannels(GaussianSplatPipeline.ConvertResult result)
    {
        string gaussDir = Path.Combine(Application.dataPath, "GaussianAssets");
        bool allPass = true;
        foreach (var ch in kChannels)
        {
            byte[] ours = ch switch
            {
                "chk" => result.chunkData,
                "pos" => result.posData,
                "oth" => result.otherData,
                "col" => result.colorData,
                "shs" => result.shData,
                _ => null,
            };
            string refPath = Path.Combine(gaussDir, $"{kAssetBase}_{ch}.bytes");
            if (!File.Exists(refPath))
            {
                Debug.LogError($"[Verify] FAIL({ch}): 基准文件不存在 {refPath}");
                allPass = false;
                continue;
            }
            allPass &= CompareBytes(ch, ours, File.ReadAllBytes(refPath));
        }
        Debug.Log(allPass
            ? "[Verify] ✅ 全部 PASS：运行时 Pipeline 与 Creator 产物逐字节一致（提取正确）"
            : "[Verify] ❌ 存在 FAIL：逐函数排查差异（对照 Creator 行号）");
    }

    static bool CompareBytes(string channel, byte[] ours, byte[] reference)
    {
        Debug.Log($"[Verify] {channel}: ours={ours.Length} ref={reference.Length}");
        if (ours.Length != reference.Length)
        {
            Debug.LogError($"[Verify] FAIL({channel}): 长度不一致 {ours.Length} vs {reference.Length}");
            return false;
        }
        for (int i = 0; i < ours.Length; ++i)
        {
            if (ours[i] != reference[i])
            {
                int from = Mathf.Max(0, i - 8), to = Mathf.Min(ours.Length, i + 8);
                Debug.LogError($"[Verify] FAIL({channel}): 首个差异偏移 {i} (0x{i:X})\n" +
                               $"  ours: {BitConverter.ToString(ours, from, to - from)}\n" +
                               $"   ref: {BitConverter.ToString(reference, from, to - from)}");
                return false;
            }
        }
        Debug.Log($"[Verify] {channel}: ✅ 逐字节一致（{ours.Length} B）");
        return true;
    }

    static string GetPlyPath() => Path.GetFullPath(Path.Combine(Application.dataPath, kPlyRel));
}
