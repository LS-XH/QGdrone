// SPDX-License-Identifier: MIT
// ============================================================================
// SplatRenderService.cs —— 门面：对外唯一入口（设计 §3，v4 定稿）
//
//   public async Task<SplatIngestResult> LoadAsync(string plyPath, bool forceReload = false)
//
// 内部自动完成：查缓存（命中直读）→（未命中则转换+落盘）→ 建 SO → 挂载渲染。
// 调用者（网络同学 / UI / 时间轴）只学这一个函数。
// 幂等：同路径重复调用默认直接返回上次结果；forceReload=true 强制重新转换。
// sceneId = Hash(路径)（O(1) 定位缓存，阶段 2a 使用）。
//
// ⚠️ 须在主线程调用（内部 Job.Schedule 主线程限定；网络同学接入时需先切主线程）。
// 失败路径：OnError 事件 + 抛 InvalidOperationException（契约结构无错误字段）。
// ============================================================================
using System;
using System.IO;
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
            // 幂等：同路径重复调用默认跳过（设计 §3；forceReload 同时绕过幂等与缓存）
            if (!forceReload && m_LastSucceeded && m_CurrentPlyPath == plyPath)
                return m_LastResult;

            // scene_id = Hash(路径)，O(1) 定位缓存（设计 §2/§3）
            string sceneId = Hash128.Compute(plyPath).ToString();
            // 缓存根路径在主线程取（Cache 不碰 Application API，后台线程安全）
            string cacheRoot = Path.Combine(Application.persistentDataPath, "splats");
            float startTime = Time.realtimeSinceStartup; // 完成日志计时

            // ===== 缓存分支（阶段 2a）：未强制刷新时先查缓存 =====
            if (!forceReload)
            {
                OnProgress?.Invoke(0.05f);
                // 纯 File IO + 托管 byte[]（无 Job 无 Temp）→ 整段后台
                var cache = await Task.Run(() => SplatAssetCache.Load(cacheRoot, sceneId));
                if (cache.Status == SplatAssetCache.LoadStatus.Hit)
                {
                    OnProgress?.Invoke(0.5f);
                    // 决策点①：相机不进缓存，现场重读 cameras.json（与转换路径同源）
                    cache.Result.cameras = new GaussianSplatPipeline().LoadCameras(plyPath);
                    OnProgress?.Invoke(0.7f);
                    var cachedAsset = m_Factory.Build(cache.Result); // 与转换路径同一条代码
                    OnProgress?.Invoke(0.9f);
                    m_Binder.Bind(cachedAsset);
                    OnProgress?.Invoke(1f);

                    m_CurrentPlyPath = plyPath;
                    m_LastResult = new SplatIngestResult
                    {
                        sceneId = sceneId,
                        splatCount = cache.Result.splatCount,
                        fromCache = true,
                    };
                    m_LastSucceeded = true;
                    Debug.Log($"SplatLoad done (cache): {m_LastResult.splatCount} splats, sceneId={m_LastResult.sceneId}, fromCache=True, 耗时 {Time.realtimeSinceStartup - startTime:F1}s");
                    return m_LastResult;
                }
                // Miss（无缓存）/ Corrupted（校验失败，Cache 已清理目录）：落回未命中路径重转
            }

            // ===== 未命中路径：转换 + 落盘 + 挂载 =====
            var result = await m_LoadingManager.ConvertAsync(plyPath);
            if (!string.IsNullOrEmpty(result.errorMessage))
            {
                m_LastSucceeded = false;
                throw new InvalidOperationException($"SplatLoad failed: {result.errorMessage}");
            }

            // 90-100%：落盘（270MB 纯 IO，后台写，不卡 UI）+ 挂载
            OnProgress?.Invoke(0.92f);
            try
            {
                await Task.Run(() => SplatAssetCache.Save(cacheRoot, sceneId, plyPath, result));
            }
            catch (Exception e)
            {
                // 落盘失败不阻断渲染：缓存只是加速，不是加载的必要条件（下次同路径重新转换即可）
                Debug.LogWarning($"缓存落盘失败（本次渲染不受影响，下次将重新转换）: {e.Message}");
            }
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
                fromCache = false,
            };
            m_LastSucceeded = true;
            Debug.Log($"SplatLoad done: {m_LastResult.splatCount} splats, sceneId={m_LastResult.sceneId}, fromCache={m_LastResult.fromCache}, 耗时 {Time.realtimeSinceStartup - startTime:F1}s");
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
