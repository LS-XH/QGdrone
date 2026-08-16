// SPDX-License-Identifier: MIT
// ============================================================================
// SplatRenderService.cs —— 门面：对外唯一入口（设计 §3，v4 定稿）
//
//   public async Task<SplatIngestResult> LoadAsync(string plyPath, bool forceReload = false)
//
// 内部自动完成：查缓存（阶段 2a 占位，暂恒未命中）→ 转换 → 建 SO → 挂载渲染。
// 调用者（网络同学 / UI / 时间轴）只学这一个函数。
// 幂等：同路径重复调用默认直接返回上次结果；forceReload=true 强制重新转换。
// sceneId = Hash(路径)（O(1) 定位缓存，阶段 2a 使用）。
//
// ⚠️ 须在主线程调用（内部 Job.Schedule 主线程限定；网络同学接入时需先切主线程）。
// 失败路径：OnError 事件 + 抛 InvalidOperationException（契约结构无错误字段）。
// ============================================================================
using System;
using System.Threading.Tasks;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    /// <summary>对外唯一入口。</summary>
    public class SplatRenderService
    {
        readonly SplatLoadingManager m_LoadingManager = new();
        readonly GaussianSplatRuntimeFactory m_Factory = new();
        readonly SplatRendererBinder m_Binder = new();

        // 幂等状态（设计 §3：同路径重复调用默认跳过）
        string m_CurrentPlyPath;
        SplatIngestResult m_LastResult;
        bool m_LastSucceeded;

        /// <summary>加载进度（0~1，主线程回调；UI 订阅）</summary>
        public event Action<float> OnProgress;
        /// <summary>加载错误（UI 订阅；同时 LoadAsync 抛异常）</summary>
        public event Action<string> OnError;

        public SplatRenderService()
        {
            m_LoadingManager.ProgressChanged += p => OnProgress?.Invoke(p);
            m_LoadingManager.ErrorOccurred += e => OnError?.Invoke(e);
        }

        /// <summary>唯一入口：plyPath = PLY 文件本地路径。内部自动：查缓存→(未命中则转换+落盘)→渲染。</summary>
        public async Task<SplatIngestResult> LoadAsync(string plyPath, bool forceReload = false)
        {
            // 幂等：同路径重复调用默认跳过（设计 §3）
            if (!forceReload && m_LastSucceeded && m_CurrentPlyPath == plyPath)
                return m_LastResult;

            // scene_id = Hash(路径)，O(1) 定位缓存（设计 §2/§3；阶段 2a 才用）
            string sceneId = Hash128.Compute(plyPath).ToString();

            var result = await m_LoadingManager.ConvertAsync(plyPath);
            if (!string.IsNullOrEmpty(result.errorMessage))
            {
                m_LastSucceeded = false;
                throw new InvalidOperationException($"SplatLoad failed: {result.errorMessage}");
            }

            // 90-100%：落盘(2a) + 挂载
            OnProgress?.Invoke(0.95f);
            var asset = m_Factory.Build(result);
            OnProgress?.Invoke(0.98f);
            m_Binder.Bind(asset);
            OnProgress?.Invoke(1f);

            m_CurrentPlyPath = plyPath;
            m_LastResult = new SplatIngestResult
            {
                sceneId = sceneId,
                splatCount = result.splatCount,
                fromCache = false, // 阶段 2a 缓存直读后为 true
            };
            m_LastSucceeded = true;
            return m_LastResult;
        }
    }

    /// <summary>加载结果（设计 §3 契约，字段不增不减）。</summary>
    public struct SplatIngestResult
    {
        /// <summary>内部派生（Hash(路径)），供流程记录。</summary>
        public string sceneId;
        public int splatCount;
        public bool fromCache;
    }
}
