// SPDX-License-Identifier: MIT
// ============================================================================
// SplatRendererBinder.cs —— 挂载渲染（简报 §四.5 / 设计 §4）
// 职责：把 SO 挂到场景中的 GaussianSplatRenderer。
// 换资产时旧 GPU 缓冲的 Dispose 由 Renderer.Update() 自动完成
// （阶段 0 包改动：m_PrevAsset != m_Asset 检测 → DisposeResourcesForAsset + 重建，
//   见 GaussianSplatRenderer.cs 683-692 行）——Binder 只需赋引用，零手动 Dispose。
// ============================================================================
using GaussianSplatting.Runtime;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    public class SplatRendererBinder
    {
        /// <summary>挂载资产。返回被挂载的 renderer（场景已有则复用，否则创建）。</summary>
        public GaussianSplatRenderer Bind(GaussianSplatAsset asset)
        {
            var renderer = Object.FindFirstObjectByType<GaussianSplatRenderer>();
            if (renderer == null)
            {
                // 提示性警告（2026-08-17 用户实测：完全空的新场景仅挂 SplatLoadUI 时自动创建不可靠，
                // 规范做法 = 场景中手动放置挂 GaussianSplatRenderer 的空物体；自动创建仅作兜底）
                Debug.LogWarning("[SplatBinder] 场景中未找到 GaussianSplatRenderer，已自动创建兜底（建议在场景中手动放置一个挂该组件的空物体，见施工简报-20260817 新场景重建清单）");
                var go = new GameObject("GaussianSplatRuntimeRenderer");
                renderer = go.AddComponent<GaussianSplatRenderer>();
            }
            renderer.m_Asset = asset; // Update() 检测资产变化 → 自动 Dispose 旧缓冲 + 重建 GPU 数据
            return renderer;
        }
    }
}
