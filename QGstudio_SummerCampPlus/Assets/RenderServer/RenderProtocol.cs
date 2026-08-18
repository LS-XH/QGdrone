// SPDX-License-Identifier: MIT
//
// 网络协议的数据模型 —— AI(服务器) <-> Unity 图形端 的 WebSocket JSON 消息。
// 用 Unity 内置 JsonUtility 序列化（字段名与服务器一致，camelCase）。
//
// 扩展方式：AI 元数据要加字段，直接在 RenderMeta 里加 public 字段即可，
// 服务器原样转发 JSON，JsonUtility 会自动映射同名字段。

using System;

namespace RenderServer
{
    /// <summary>
    /// AI 随 start/upload 带过来的元数据（可扩展：加字段即可）。
    /// </summary>
    [Serializable]
    public class RenderMeta
    {
        public string precision;        // 精度信息，字符串（如 "high" / "low"）
        public string timestamp;        // 时间，字符串
        public string modelName;        // 模型名字
        public int submissionCount;     // 提交的次数
        // TODO: AI 组要加字段就加在这里，例如:
        // public string status;
    }

    /// <summary>
    /// Unity 启动时发给服务器的注册消息（init 握手）。
    /// </summary>
    [Serializable]
    public class GraphicsReadyMessage
    {
        public string type = "graphics_ready";
        public string client = "graphics";
        public string clientVersion = "1.0";
    }

    /// <summary>
    /// 服务器转发过来的请求（start / upload / end），都带 requestId。
    /// </summary>
    [Serializable]
    public class ServerRequestMessage
    {
        public string type;             // "start" | "upload" | "end" | "graphics_ready_ack"
        public string requestId;
        public RenderMeta metadata;     // start / upload 用
        public string roundId;          // start 用：本轮文件夹名（服务器生成，会话 id 与其对齐）
        public string savedPath;        // upload 用：PLY 的本地绝对路径（同机图形端直接读）
        public string fileUrl;          // upload 用：PLY 的公网下载 URL（远程图形端按需下载）
        public string filename;         // upload 用：原始文件名
        public RenderSummary summary;   // end 用
    }

    /// <summary>
    /// end 消息附带的信息。
    /// </summary>
    [Serializable]
    public class RenderSummary
    {
        public string message;
    }

    /// <summary>
    /// Unity 回给服务器的响应，requestId 必须和请求一致。
    /// </summary>
    [Serializable]
    public class ClientResponseMessage
    {
        public string type;             // "start_response" | "upload_response" | "end_response"
        public string requestId;
        public bool success;
        public string message;
        public string sessionId;
        public string savedPath;
        public string manifestPath;
    }
}
