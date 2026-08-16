// SPDX-License-Identifier: MIT
//
// 会话/模型存储：
//   1. 每个模型一个文件夹 + 一张表：
//        <PLY所在文件夹>/manifest.json   （模型文件夹里，记录该模型所有精度的 PLY）
//      PLY 由服务器存到 <SAVE_DIR>/<modelName>/[precision_]文件名，
//      Unity 收到后把清单表写到同一个模型文件夹里。
//   2. 本轮(会话)记录：
//        <persistentDataPath>/RenderSessions/<sessionId>/manifest.json
//      结束时写，供 UI 浏览"每一轮渲染"。

using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

namespace RenderServer
{
    /// <summary>一轮渲染（start→end）的会话清单。</summary>
    [Serializable]
    public class SessionManifest
    {
        public string sessionId;
        public string createdAt;      // ISO 时间
        public string endedAt;
        public string status;         // "running" | "done" | "error"
        public List<RecordedPly> files = new List<RecordedPly>();
    }

    /// <summary>会话清单里的单个 PLY 记录。</summary>
    [Serializable]
    public class RecordedPly
    {
        public string fileName;
        public string savedPath;
        public long sizeBytes;
        public RenderMeta metadata;
    }

    /// <summary>模型清单（一张表）：一个模型所有精度的记录。</summary>
    [Serializable]
    public class ModelManifest
    {
        public string modelName;
        public string lastUpdated;
        public List<ModelEntry> entries = new List<ModelEntry>();
    }

    /// <summary>模型清单里的单条记录（对应一次上传的 PLY）。</summary>
    [Serializable]
    public class ModelEntry
    {
        public string fileName;         // PLY 文件名
        public string savedPath;        // PLY 地址
        public long sizeBytes;          // 文件大小
        public string precision;        // 精度
        public string timestamp;        // 时间
        public int submissionCount;     // 提交次数
        public string metadataJson;     // 完整 JSON 信息（原样）
    }

    public static class SessionManager
    {
        public static string RootPath => Path.Combine(Application.persistentDataPath, "RenderSessions");

        /// <summary>当前会话 id（未开始为 null）。</summary>
        public static string CurrentSessionId { get; private set; }
        public static string CurrentSessionDir { get; private set; }

        static SessionManifest s_Manifest;

        /// <summary>开始一轮新会话，返回 sessionId。</summary>
        public static string BeginSession(RenderMeta meta)
        {
            CurrentSessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            CurrentSessionDir = Path.Combine(RootPath, CurrentSessionId);
            Directory.CreateDirectory(CurrentSessionDir);

            s_Manifest = new SessionManifest
            {
                sessionId = CurrentSessionId,
                createdAt = DateTime.Now.ToString("o"),
                status = "running",
            };
            Debug.Log($"[SessionManager] 开始会话 {CurrentSessionId} -> {CurrentSessionDir}");
            return CurrentSessionId;
        }

        /// <summary>
        /// 记录一个收到的 PLY：
        ///   1) 追加到本轮会话清单；
        ///   2) 追加到该模型文件夹的 manifest.json（每个模型一张表）。
        /// </summary>
        public static void RecordPly(string savedPath, RenderMeta meta)
        {
            // ---- 1) 本轮会话清单 ----
            if (s_Manifest == null)
                BeginSession(null);
            s_Manifest.files.Add(new RecordedPly
            {
                fileName = Path.GetFileName(savedPath),
                savedPath = savedPath,
                sizeBytes = File.Exists(savedPath) ? new FileInfo(savedPath).Length : 0,
                metadata = meta,
            });

            // ---- 2) 模型清单表（写到 PLY 所在模型文件夹）----
            string modelName = Sanitize(meta != null ? meta.modelName : null);
            string modelDir = Path.GetDirectoryName(savedPath);
            if (string.IsNullOrEmpty(modelDir))
                modelDir = Path.Combine(RootPath, modelName);
            Directory.CreateDirectory(modelDir);

            string manifestPath = Path.Combine(modelDir, "manifest.json");
            var model = new ModelManifest { modelName = modelName };
            if (File.Exists(manifestPath))
            {
                try
                {
                    var loaded = JsonUtility.FromJson<ModelManifest>(File.ReadAllText(manifestPath));
                    if (loaded != null && loaded.modelName == modelName)
                        model = loaded;
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[SessionManager] 读取模型清单失败，重建: {e.Message}");
                }
            }

            model.entries.Add(new ModelEntry
            {
                fileName = Path.GetFileName(savedPath),
                savedPath = savedPath,
                sizeBytes = File.Exists(savedPath) ? new FileInfo(savedPath).Length : 0,
                precision = meta != null ? meta.precision : null,
                timestamp = meta != null ? meta.timestamp : null,
                submissionCount = meta != null ? meta.submissionCount : 0,
                metadataJson = meta != null ? JsonUtility.ToJson(meta) : "{}",
            });
            model.lastUpdated = DateTime.Now.ToString("o");
            File.WriteAllText(manifestPath, JsonUtility.ToJson(model, true));
            Debug.Log($"[SessionManager] 模型表更新: {manifestPath}（{modelName} 共 {model.entries.Count} 条）");
        }

        /// <summary>结束会话并写会话清单，返回清单文件路径。</summary>
        public static string EndSession(bool success, string message = null)
        {
            if (s_Manifest == null)
                return null;

            s_Manifest.status = success ? "done" : "error";
            s_Manifest.endedAt = DateTime.Now.ToString("o");
            var path = Path.Combine(CurrentSessionDir, "manifest.json");
            File.WriteAllText(path, JsonUtility.ToJson(s_Manifest, true));
            Debug.Log($"[SessionManager] 会话结束 ({s_Manifest.status}), 清单 -> {path}");

            s_Manifest = null;
            return path;
        }

        /// <summary>把字符串变成能当文件夹名的安全名。</summary>
        static string Sanitize(string s)
        {
            if (string.IsNullOrEmpty(s)) return "unknown";
            var t = Regex.Replace(s, @"[\\/:*?""<>|]", "_").Trim();
            if (string.IsNullOrEmpty(t) || t == "." || t == "..") return "unknown";
            return t.Length > 80 ? t.Substring(0, 80) : t;
        }
    }
}
