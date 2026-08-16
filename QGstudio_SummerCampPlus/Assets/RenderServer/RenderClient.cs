// SPDX-License-Identifier: MIT
//
// RenderClient —— Unity 图形端网络主入口。
//
// 职责：
//   1. 程序启动时主动连接服务器(WebSocket)并注册为"图形节点"（init 握手）
//   2. 断线自动重连（指数退避）
//   3. 收到 AI(经服务器转发)的消息 → 主线程派发 → 调 IRenderHandler 回调
//   4. 把回调结果（含 bool / savedPath / manifestPath）回给服务器
//
// 使用：
//   场景里放一个挂 RenderClient 的 GameObject，
//   再用菜单 RenderServer -> Setup Scene 一键生成（含示例回调）。
//   serverUrl 默认 ws://localhost:8080/ws/graphics。

using System;
using System.IO;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace RenderServer
{
    public class RenderClient : MonoBehaviour
    {
        [Header("服务器地址")]
        [Tooltip("独立服务器的 WebSocket 地址（Unity 启动时主动连这里）")]
        public string serverUrl = "ws://localhost:8080/ws/graphics";

        [Tooltip("找不到回调组件时是否自动在场景里查找实现 IRenderHandler 的组件")]
        public bool autoFindHandler = true;

        [Header("渲染回调（黑盒）")]
        [Tooltip("实现 IRenderHandler 的组件；留空则自动查找")]
        [SerializeField] MonoBehaviour m_RenderHandler;

        ClientWebSocket m_WebSocket;
        CancellationTokenSource m_StopCts;
        float m_ReconnectDelay = 2f;
        IRenderHandler m_Handler;

        public bool IsConnected => m_WebSocket != null && m_WebSocket.State == WebSocketState.Open;
        public event Action onConnected;
        public event Action onDisconnected;

        async void Start()
        {
            // 失焦也继续跑：AI 组从公网推模型时没人盯着 Unity，
            // 默认 runInBackground=false 会导致窗口失焦就暂停，WebSocket 消息积压超时。
            Application.runInBackground = true;

            // 确保主线程派发器存在（后台线程的 WebSocket 回调要用）
            var _ = MainThreadDispatcher.Instance;

            m_StopCts = new CancellationTokenSource();
            ResolveHandler();
            Debug.Log($"[RenderClient] 启动，连接服务器 {serverUrl} ...");
            await ConnectLoop(m_StopCts.Token);
        }

        void OnDestroy()
        {
            m_StopCts?.Cancel();
            m_StopCts?.Dispose();
            m_StopCts = null;
            CloseSocket();
        }

        // ------------------------------------------------------------------
        // 连接循环（断线自动重连，指数退避）
        // ------------------------------------------------------------------
        async Task ConnectLoop(CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                try
                {
                    await ConnectOnce(ct);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[RenderClient] 连接失败: {e.Message}");
                }

                if (ct.IsCancellationRequested) break;

                m_ReconnectDelay = Mathf.Min(m_ReconnectDelay * 1.5f, 15f);
                Debug.Log($"[RenderClient] {m_ReconnectDelay:F0}s 后重连...");
                try { await Task.Delay(TimeSpan.FromSeconds(m_ReconnectDelay), ct); }
                catch (OperationCanceledException) { break; }
            }
        }

        async Task ConnectOnce(CancellationToken ct)
        {
            m_WebSocket = new ClientWebSocket();
            await m_WebSocket.ConnectAsync(new Uri(serverUrl), ct);
            Debug.Log("[RenderClient] 已连接服务器");

            // init 握手：注册为图形节点
            await SendAsync(new GraphicsReadyMessage());
            Debug.Log("[RenderClient] 已发送 graphics_ready，注册为图形节点");
            m_ReconnectDelay = 2f;
            onConnected?.Invoke();

            await ReceiveLoop(ct);

            CloseSocket();
            onDisconnected?.Invoke();
        }

        // ------------------------------------------------------------------
        // 接收循环（后台线程，收完派发到主线程）
        // ------------------------------------------------------------------
        async Task ReceiveLoop(CancellationToken ct)
        {
            var buffer = new byte[16 * 1024];
            while (!ct.IsCancellationRequested)
            {
                WebSocketReceiveResult result;
                using (var ms = new MemoryStream())
                {
                    do
                    {
                        result = await m_WebSocket.ReceiveAsync(new ArraySegment<byte>(buffer), ct);
                        if (result.MessageType == WebSocketMessageType.Close)
                            return;
                        if (result.MessageType == WebSocketMessageType.Text)
                            ms.Write(buffer, 0, result.Count);
                    }
                    while (!result.EndOfMessage);

                    var json = Encoding.UTF8.GetString(ms.ToArray());
                    // 后台线程 → 主线程处理（HandleServerMessage 里会回调用户函数）
                    await MainThreadDispatcher.PostAsync(() => { HandleServerMessage(json); return true; });
                }
            }
        }

        // ------------------------------------------------------------------
        // 消息处理（在主线程执行）
        // ------------------------------------------------------------------
        void HandleServerMessage(string json)
        {
            ServerRequestMessage msg;
            try { msg = JsonUtility.FromJson<ServerRequestMessage>(json); }
            catch (Exception e)
            {
                Debug.LogError($"[RenderClient] 无法解析消息: {e.Message}\n{json}");
                return;
            }

            switch (msg.type)
            {
                case "graphics_ready_ack":
                    Debug.Log("[RenderClient] 服务器已确认注册");
                    break;
                case "start":
                    HandleStart(msg);
                    break;
                case "upload":
                    HandleUpload(msg);
                    break;
                case "end":
                    HandleEnd(msg);
                    break;
                default:
                    Debug.LogWarning($"[RenderClient] 未知消息类型: {msg.type}");
                    break;
            }
        }

        void HandleStart(ServerRequestMessage msg)
        {
            bool ok = false;
            string err = "";
            try
            {
                EnsureHandler();
                ok = m_Handler.OnStartRender();
                if (ok)
                {
                    SessionManager.BeginSession(msg.metadata);
                    err = "start ok";
                }
                else
                {
                    err = "OnStartRender 返回 false";
                }
            }
            catch (Exception e) { err = e.Message; }

            _ = SendAsync(new ClientResponseMessage
            {
                type = "start_response",
                requestId = msg.requestId,
                success = ok,
                message = err,
                sessionId = SessionManager.CurrentSessionId,
            });
        }

        void HandleUpload(ServerRequestMessage msg)
        {
            bool ok = false;
            string err = "";
            try
            {
                EnsureHandler();
                if (string.IsNullOrEmpty(msg.savedPath))
                    throw new Exception("服务器未传 savedPath");
                if (!File.Exists(msg.savedPath))
                    throw new Exception($"PLY 文件不存在: {msg.savedPath}");

                SessionManager.RecordPly(msg.savedPath, msg.metadata);
                m_Handler.RenderPly(msg.savedPath); // 黑盒：只传地址
                ok = true;
                err = "render ok";
            }
            catch (Exception e) { err = e.Message; }

            _ = SendAsync(new ClientResponseMessage
            {
                type = "upload_response",
                requestId = msg.requestId,
                success = ok,
                message = err,
                savedPath = msg.savedPath,
                sessionId = SessionManager.CurrentSessionId,
            });
        }

        void HandleEnd(ServerRequestMessage msg)
        {
            bool ok = false;
            string err = "";
            string manifestPath = "";
            try
            {
                EnsureHandler();
                m_Handler.OnRenderEnd();
                manifestPath = SessionManager.EndSession(true);
                ok = true;
                err = "end ok";
            }
            catch (Exception e)
            {
                err = e.Message;
                manifestPath = SessionManager.EndSession(false, err);
            }

            _ = SendAsync(new ClientResponseMessage
            {
                type = "end_response",
                requestId = msg.requestId,
                success = ok,
                message = err,
                sessionId = SessionManager.CurrentSessionId,
                manifestPath = manifestPath,
            });
        }

        // ------------------------------------------------------------------
        // 辅助
        // ------------------------------------------------------------------
        void ResolveHandler()
        {
            if (m_RenderHandler != null)
            {
                m_Handler = m_RenderHandler as IRenderHandler;
                if (m_Handler == null)
                    Debug.LogWarning("[RenderClient] m_RenderHandler 没有实现 IRenderHandler，将自动查找");
            }
            if (m_Handler == null && autoFindHandler)
            {
                foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
                {
                    if (mb is IRenderHandler h) { m_Handler = h; break; }
                }
            }
            if (m_Handler == null)
                Debug.LogWarning("[RenderClient] 场景里没有找到实现 IRenderHandler 的组件，收到请求将报错");
            else
                Debug.Log($"[RenderClient] 渲染回调 = {m_Handler.GetType().Name}");
        }

        void EnsureHandler()
        {
            if (m_Handler == null)
                throw new Exception("未找到 IRenderHandler 渲染回调组件");
        }

        Task SendAsync(object msg)
        {
            if (m_WebSocket == null || m_WebSocket.State != WebSocketState.Open)
                return Task.CompletedTask;
            var json = JsonUtility.ToJson(msg);
            var bytes = Encoding.UTF8.GetBytes(json);
            return m_WebSocket.SendAsync(
                new ArraySegment<byte>(bytes),
                WebSocketMessageType.Text,
                true,
                CancellationToken.None);
        }

        void CloseSocket()
        {
            if (m_WebSocket != null)
            {
                try
                {
                    if (m_WebSocket.State == WebSocketState.Open)
                        m_WebSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None)
                            .GetAwaiter().GetResult();
                }
                catch { }
                m_WebSocket.Dispose();
                m_WebSocket = null;
            }
        }
    }
}
