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
//   serverUrl 默认 ws://47.113.224.195:32506/ws/graphics（公网，经 npc 内网穿透）。

using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Http;
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
        public string serverUrl = "ws://47.113.224.195:32506/ws/graphics";

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
                    // roundId 是服务器本轮文件夹名，用同一 id 当会话 id，两边名字对齐
                    SessionManager.BeginSession(msg.metadata, msg.roundId);
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

        async void HandleUpload(ServerRequestMessage msg)
        {
            bool ok = false;
            string err = "";
            string localPath = null;
            try
            {
                EnsureHandler();
                if (string.IsNullOrEmpty(msg.savedPath) && string.IsNullOrEmpty(msg.fileUrl))
                    throw new Exception("服务器未传 savedPath / fileUrl");
                localPath = msg.savedPath;
                if (localPath != null && File.Exists(localPath))
                {
                    // 本机就有服务器路径的文件（服务器这台机器通常就是）：直接用，不走网络。
                    // 不特殊对待——下面照样记本机 allModels.json。
                }
                else
                {
                    // 本机没有服务器磁盘上的文件：按 fileUrl 下载一份进本机模型库
                    // Assets/Data/Incoming/<模型>/<轮次>/…，离线也能看。
                    if (string.IsNullOrEmpty(msg.fileUrl))
                        throw new Exception($"PLY 本地不存在且服务器未提供下载地址: {localPath}");
                    var fileName = !string.IsNullOrEmpty(localPath) ? Path.GetFileName(localPath) : msg.filename;
                    var modelName = msg.metadata != null ? msg.metadata.modelName : null;
                    var roundId = msg.roundId;
                    // 旧服务器没随 upload 传 roundId / 元数据缺 modelName 时，从下载 URL 拆
                    // （URL 形如 …/api/v1/files/<模型>/<轮次>/<文件>，每段都已 URL 编码）
                    if (TryParseFileUrl(msg.fileUrl, out var urlModel, out var urlRound, out var urlFile))
                    {
                        if (string.IsNullOrEmpty(roundId)) roundId = urlRound;
                        if (string.IsNullOrEmpty(modelName)) modelName = urlModel;
                        if (string.IsNullOrEmpty(fileName)) fileName = urlFile;
                    }
                    if (string.IsNullOrEmpty(fileName)) fileName = "download.ply";
                    fileName = SessionManager.SanitizeName(fileName); // 文件名也清洗，防路径穿越
                    modelName = SessionManager.SanitizeName(modelName);
                    roundId = SessionManager.SanitizeName(roundId);
                    // 本机库路径：Assets/Data/Incoming/<模型>/<轮次>/<文件>（与服务器落盘结构一致）
                    var libPath = Path.Combine(SessionManager.IncomingDir, modelName, roundId, fileName);
                    if (File.Exists(libPath))
                    {
                        // 之前已经存过同一份 → 直接用，不用再下载（索引由 UpsertLocalPly 兜底补）
                        Debug.Log($"[RenderClient] 本机库已有 {libPath}，跳过下载直接渲染");
                        localPath = libPath;
                    }
                    else
                    {
                        Debug.Log($"[RenderClient] 本地无 {localPath}，从 {msg.fileUrl} 下载到本机库 {modelName}/{roundId}/…");
                        localPath = await DownloadPlyAsync(msg.fileUrl, modelName, roundId, fileName);
                    }
                }
                if (!File.Exists(localPath))
                    throw new Exception($"PLY 文件不存在: {localPath}");

                SessionManager.RecordPly(localPath, msg.metadata);
                // 所有机器统一逻辑：收到的 PLY 都记进本机 allModels.json（去重由 UpsertLocalPly 保证）
                SessionManager.UpsertLocalPly(localPath, msg.metadata);
                m_Handler.RenderPly(localPath); // 黑盒：传本地绝对路径
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
                savedPath = localPath,
                sessionId = SessionManager.CurrentSessionId,
            });
        }

        /// <summary>下载大小上限（防恶意/超大 PLY 撑爆内存，1GB）。</summary>
        const long MaxDownloadBytes = 1024L * 1024 * 1024;

        /// <summary>
        /// 按公网 URL 把 PLY 流式下载到本机模型库
        /// Assets/Data/Incoming/&lt;模型&gt;/&lt;轮次&gt;/&lt;文件&gt;（与服务器落盘结构一致，离线可查看）。
        /// </summary>
        async Task<string> DownloadPlyAsync(string fileUrl, string modelName, string roundId, string fileName)
        {
            var dir = Path.Combine(SessionManager.IncomingDir, modelName, roundId ?? "downloads");
            Directory.CreateDirectory(dir);
            var localPath = Path.Combine(dir, fileName);
            try
            {
                using (var client = new HttpClient())
                {
                    client.Timeout = TimeSpan.FromSeconds(110); // 略小于服务器 120s 回包超时
                    using (var src = await client.GetStreamAsync(fileUrl))
                    using (var dst = new FileStream(localPath, FileMode.Create, FileAccess.Write))
                    {
                        long written = 0;
                        var buf = new byte[64 * 1024];
                        int n;
                        while ((n = await src.ReadAsync(buf, 0, buf.Length)) > 0)
                        {
                            written += n;
                            if (written > MaxDownloadBytes)
                                throw new IOException($"PLY 下载超过大小上限 ({MaxDownloadBytes / (1024 * 1024)}MB)：{fileUrl}");
                            await dst.WriteAsync(buf, 0, n);
                        }
                    }
                }
            }
            catch
            {
                if (File.Exists(localPath)) { try { File.Delete(localPath); } catch { } } // 失败不留半截文件
                throw;
            }
            Debug.Log($"[RenderClient] 已下载到本机库 {localPath} ({new FileInfo(localPath).Length / 1024} KB)");
            return localPath;
        }

        /// <summary>
        /// 从下载 URL 拆出 模型/轮次/文件名。
        /// URL 形如 …/api/v1/files/&lt;模型&gt;/&lt;轮次&gt;/&lt;文件&gt;，每段都已 URL 编码，这里逐个解码。
        /// </summary>
        static bool TryParseFileUrl(string url, out string model, out string round, out string file)
        {
            model = round = file = null;
            var segs = new List<string>();
            try
            {
                foreach (var raw in url.Split('/'))
                {
                    if (string.IsNullOrEmpty(raw)) continue;
                    var seg = raw;
                    var cut = seg.IndexOfAny(new[] { '?', '#' }); // 去掉 query/fragment，防 token 混进文件名
                    if (cut >= 0) seg = seg.Substring(0, cut);
                    if (seg.Length == 0) continue;
                    segs.Add(Uri.UnescapeDataString(seg));
                }
            }
            catch { return false; }
            if (segs.Count < 3) return false;
            file = segs[segs.Count - 1];
            round = segs[segs.Count - 2];
            model = segs[segs.Count - 3];
            return true;
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
