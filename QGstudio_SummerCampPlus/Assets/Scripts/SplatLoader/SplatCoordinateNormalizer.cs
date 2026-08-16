// SPDX-License-Identifier: MIT
// ============================================================================
// SplatCoordinateNormalizer.cs —— 坐标归一化（简报 §四.6 / 设计 §4）
// 减质心：把模型坐标平移到原点附近（场景中心对齐），默认开，须在 Morton 之前。
// 注意：开归一化后产物与 Creator 产物不一致（坐标被平移）是预期行为——
//       逐字节验证须在 Enabled=false 下进行（验证脚本已内置关闭）。
// 纯函数（无 Job），可在主线程或后台线程调用。
// ============================================================================
using Unity.Collections;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    public static class SplatCoordinateNormalizer
    {
        /// <summary>默认开（简报 §四.6）。验证逐字节一致性时置 false。</summary>
        public static bool Enabled = true;

        /// <summary>减质心。返回质心（后续如需还原坐标可用）。</summary>
        public static Vector3 Normalize(NativeArray<InputSplatData> splats)
        {
            if (!splats.IsCreated || splats.Length == 0)
                return Vector3.zero;

            // 第一遍：求质心
            Vector3 centroid = Vector3.zero;
            for (int i = 0; i < splats.Length; ++i)
                centroid += (Vector3)splats[i].pos;
            centroid /= splats.Length;

            // 第二遍：平移
            for (int i = 0; i < splats.Length; ++i)
            {
                var s = splats[i];
                s.pos = (Vector3)s.pos - centroid;
                splats[i] = s;
            }
            return centroid;
        }
    }
}
