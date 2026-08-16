// SPDX-License-Identifier: MIT
//
// 主线程派发器：WebSocket 接收在后台线程，Unity API 只能在主线程调用。
// 后台线程用 Post/PostAsync 把回调压进队列，Update() 在主线程逐个执行。
// 用 PostAsync<T> 能在后台拿到主线程回调的返回值（如 OnStartRender 的 bool）。

using System;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using UnityEngine;

namespace RenderServer
{
    public class MainThreadDispatcher : MonoBehaviour
    {
        static MainThreadDispatcher s_Instance;
        readonly ConcurrentQueue<Action> m_Queue = new ConcurrentQueue<Action>();

        /// <summary>确保单例存在（没有就创建一个常驻 GameObject）。</summary>
        public static MainThreadDispatcher Instance
        {
            get
            {
                if (s_Instance == null)
                {
                    var go = new GameObject("[MainThreadDispatcher]");
                    s_Instance = go.AddComponent<MainThreadDispatcher>();
                    DontDestroyOnLoad(go);
                }
                return s_Instance;
            }
        }

        void Awake()
        {
            if (s_Instance != null && s_Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            s_Instance = this;
        }

        void Update()
        {
            while (m_Queue.TryDequeue(out var action))
            {
                try { action(); }
                catch (Exception e) { Debug.LogException(e); }
            }
        }

        void OnDestroy()
        {
            if (s_Instance == this) s_Instance = null;
        }

        /// <summary>后台线程调：把动作排到主线程执行（无返回值）。</summary>
        public static void Post(Action action)
        {
            if (action == null) return;
            Instance.m_Queue.Enqueue(action);
        }

        /// <summary>后台线程调：在主线程执行 func 并等待其结果。</summary>
        public static Task<T> PostAsync<T>(Func<T> func)
        {
            var tcs = new TaskCompletionSource<T>();
            Post(() =>
            {
                try { tcs.SetResult(func()); }
                catch (Exception e) { tcs.SetException(e); }
            });
            return tcs.Task;
        }
    }
}
