// SPDX-License-Identifier: MIT
// ============================================================================
// SplatPipelineVerify.cs —— 阶段 1 提取正确性验证（最强铁证）
// 用法：Unity 编辑器 → 菜单 Tools/QG Studio/Verify Splat Pipeline (byte-exact)
// 行为：运行时 Pipeline.ConvertAll(同一 PLY + Medium) 输出 vs Creator 产物
//       （Assets/GaussianAssets/ 下 5 个 .bytes）逐字节对比。
// 全 PASS = 提取无偏差；任一 FAIL = 提取有偏差，按差异偏移排查对应函数。
// 黄金基准（阶段 0 实测）：pos=24,527,816 / oth=49,055,632 / col=24,641,536
//                          shs=196,222,528 / chk=1,532,992
// ============================================================================
using System;
using System.IO;
using QGStudio.SplatLoader;
using UnityEditor;
using UnityEngine;

public static class SplatPipelineVerify
{
    const string kPlyRel = "../../data/bicycle_iteration_30000_point_cloud.ply";
    const string kAssetBase = "bicycle_iteration_30000_point_cloud";
    static readonly string[] kChannels = { "chk", "pos", "oth", "col", "shs" };

    [MenuItem("Tools/QG Studio/Verify Splat Pipeline (byte-exact)")]
    public static void Verify()
    {
        string plyPath = Path.GetFullPath(Path.Combine(Application.dataPath, kPlyRel));
        string gaussDir = Path.Combine(Application.dataPath, "GaussianAssets");
        Debug.Log($"[Verify] PLY = {plyPath}");
        Debug.Log($"[Verify] 基准目录 = {gaussDir}");
        if (!File.Exists(plyPath))
        {
            Debug.LogError($"[Verify] FAIL: 找不到 PLY {plyPath}");
            return;
        }

        var pipeline = new GaussianSplatPipeline();
        pipeline.importCameras = false; // 相机不参与 byte 对比，关闭省事
        pipeline.SetProgressCallback(p =>
            EditorUtility.DisplayProgressBar("Splat Pipeline Verify", $"转换中 {p:P0}", p));

        try
        {
            var result = pipeline.ConvertAll(plyPath, GaussianSplatPipeline.DataQuality.Medium);
            if (!string.IsNullOrEmpty(result.errorMessage))
            {
                Debug.LogError($"[Verify] FAIL: 转换失败 {result.errorMessage}");
                return;
            }
            Debug.Log($"[Verify] 转换完成 splatCount={result.splatCount} dataHash={result.dataHash}");

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
                byte[] reference = File.ReadAllBytes(refPath);
                allPass &= CompareBytes(ch, ours, reference);
            }
            Debug.Log(allPass
                ? "[Verify] ✅ 全部 PASS：运行时 Pipeline 与 Creator 产物逐字节一致（提取正确）"
                : "[Verify] ❌ 存在 FAIL：逐函数排查差异（对照 Creator 行号）");
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
}
