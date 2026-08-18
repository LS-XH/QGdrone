// SPDX-License-Identifier: MIT
//
// 会话存储：
//   本轮(会话)记录：
//        <persistentDataPath>/RenderSessions/<sessionId>/manifest.json
//      结束时写，供 UI 浏览"每一轮渲染"。
//   模型级大清单（每个模型的文件夹绝对路径 + 每个 PLY 的绝对路径/JSON）由服务器
//   维护在 <SAVE_DIR>/allModels.json，Unity 这边不再写模型夹 manifest.json。

using System;
using System.Collections.Generic;
using System.IO;
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

    /// <summary>会话清单里的单个 PLY 记录（savedPath 是本地绝对路径）。</summary>
    [Serializable]
    public class RecordedPly
    {
        public string fileName;
        public string savedPath;
        public long sizeBytes;
        public RenderMeta metadata;
    }

    public static class SessionManager
    {
        public static string RootPath => Path.Combine(Application.persistentDataPath, "RenderSessions");

        /// <summary>当前会话 id（未开始为 null）。</summary>
        public static string CurrentSessionId { get; private set; }
        public static string CurrentSessionDir { get; private set; }

        static SessionManifest s_Manifest;

        /// <summary>开始一轮新会话，返回 sessionId（用服务器传来的 roundId，缺省才自生成）。</summary>
        public static string BeginSession(RenderMeta meta, string roundId = null)
        {
            CurrentSessionId = string.IsNullOrEmpty(roundId)
                ? DateTime.Now.ToString("yyyyMMdd_HHmmss")
                : roundId;
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
        /// 记录一个收到的 PLY（追加到本轮会话清单）。
        /// plyPath 是绝对路径；模型级大清单由服务器维护在
        /// &lt;SAVE_DIR&gt;/allModels.json，这里不再写模型夹 manifest.json。
        /// </summary>
        public static void RecordPly(string plyPath, RenderMeta meta)
        {
            if (s_Manifest == null)
                BeginSession(null);

            s_Manifest.files.Add(new RecordedPly
            {
                fileName = Path.GetFileName(plyPath),
                savedPath = plyPath,
                sizeBytes = File.Exists(plyPath) ? new FileInfo(plyPath).Length : 0,
                metadata = meta,
            });
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

        // ------------------------------------------------------------------
        // 删除本地文件接口（图形组 UI / 脚本直接调用，不经过服务器）
        // 操作的是本地 PLY 文件夹 + 大清单 <SAVE_DIR>/allModels.json
        // ------------------------------------------------------------------

        /// <summary>大清单文件：Assets/Data/Incoming/allModels.json（服务器写、这里读改写）。</summary>
        public static string ModelsIndexPath => Path.Combine(IncomingDir, "allModels.json");

        /// <summary>本地 PLY 保存根目录：Assets/Data/Incoming（与服务器 SAVE_DIR 一致）。</summary>
        public static string IncomingDir => Path.Combine(Application.dataPath, "Data", "Incoming");

        /// <summary>删整个模型：删模型文件夹（含所有 PLY）+ 大清单里整条模型记录。</summary>
        public static bool DeleteModel(string modelName)
        {
            if (string.IsNullOrEmpty(modelName)) return false;
            bool ok = false;
            // 1. 删本地模型文件夹
            var dir = Path.Combine(IncomingDir, modelName);
            if (Directory.Exists(dir))
            {
                try { Directory.Delete(dir, true); ok = true; }
                catch (Exception e) { Debug.LogError($"[SessionManager] 删除模型文件夹失败 {dir}: {e.Message}"); return false; }
            }
            // 2. 大清单移除该模型条目
            try
            {
                var idx = LoadModelsIndex();
                int before = idx.models.Count;
                idx.models.RemoveAll(m => m.modelName == modelName);
                if (idx.models.Count != before) SaveModelsIndex(idx);
                Debug.Log($"[SessionManager] 已删除模型「{modelName}」（文件夹 + 大清单记录）");
            }
            catch (Exception e) { Debug.LogError($"[SessionManager] 更新大清单失败: {e.Message}"); }
            return ok;
        }

        /// <summary>删单个 PLY：删文件 + 大清单那条记录；若该模型没有其它 PLY，模型条目和文件夹一起删。</summary>
        public static bool DeletePly(string modelName, string plyAbsPath)
        {
            if (string.IsNullOrEmpty(plyAbsPath)) return false;
            bool ok = false;
            // 1. 删本地 PLY 文件（plyAbsPath 是绝对路径）
            if (File.Exists(plyAbsPath))
            {
                try { File.Delete(plyAbsPath); ok = true; }
                catch (Exception e) { Debug.LogError($"[SessionManager] 删除 PLY 失败 {plyAbsPath}: {e.Message}"); return false; }
                // 向上清空空的轮次/模型目录（到 Incoming 为止）：该轮最后一个 PLY 时轮次夹一并删掉
                var saveRoot = Path.GetFullPath(IncomingDir);
                var dir = Path.GetDirectoryName(plyAbsPath);
                while (dir != null && dir.StartsWith(saveRoot, StringComparison.OrdinalIgnoreCase) && dir != saveRoot)
                {
                    if (Directory.Exists(dir) && Directory.GetFileSystemEntries(dir).Length == 0)
                        Directory.Delete(dir);
                    else
                        break;
                    dir = Path.GetDirectoryName(dir);
                }
            }
            // 2. 大清单移除该条记录
            try
            {
                var idx = LoadModelsIndex();
                var model = idx.models.Find(m => m.modelName == modelName);
                if (model != null)
                {
                    int before = model.plies.Count;
                    model.plies.RemoveAll(p => p.relativePath == plyAbsPath ||
                                               Path.GetFileName(p.relativePath) == Path.GetFileName(plyAbsPath));
                    if (model.plies.Count != before)
                    {
                        if (model.plies.Count == 0)
                        {
                            idx.models.Remove(model);
                            var dir = Path.Combine(IncomingDir, modelName);
                            if (Directory.Exists(dir)) { try { Directory.Delete(dir, true); } catch { } }
                        }
                        SaveModelsIndex(idx);
                    }
                }
                Debug.Log($"[SessionManager] 已删除 PLY: {plyAbsPath}");
            }
            catch (Exception e) { Debug.LogError($"[SessionManager] 更新大清单失败: {e.Message}"); }
            return ok;
        }

        /// <summary>读大清单 allModels.json（不存在返回空清单）。</summary>
        static ModelsIndex LoadModelsIndex()
        {
            var idx = new ModelsIndex();
            if (!File.Exists(ModelsIndexPath)) return idx;
            try { idx = JsonUtility.FromJson<ModelsIndex>(File.ReadAllText(ModelsIndexPath)); }
            catch (Exception e) { Debug.LogWarning($"[SessionManager] 读大清单失败，重建: {e.Message}"); }
            if (idx == null) idx = new ModelsIndex();
            if (idx.models == null) idx.models = new List<ModelEntry>();
            return idx;
        }

        /// <summary>写大清单 allModels.json（带 lastUpdated）。</summary>
        static void SaveModelsIndex(ModelsIndex idx)
        {
            idx.lastUpdated = DateTime.Now.ToString("o");
            File.WriteAllText(ModelsIndexPath, JsonUtility.ToJson(idx, true));
            Debug.Log($"[SessionManager] 大清单已更新: {ModelsIndexPath}");
        }
    }

    /// <summary>大清单顶层结构（与服务器 allModels.json 字段一致）。</summary>
    [Serializable]
    public class ModelsIndex
    {
        public string lastUpdated;
        public List<ModelEntry> models = new List<ModelEntry>();
    }

    /// <summary>大清单里的一个模型条目。</summary>
    [Serializable]
    public class ModelEntry
    {
        public string modelName;
        public string modelFolder;
        public List<PlyEntry> plies = new List<PlyEntry>();
    }

    /// <summary>大清单里的一条 PLY 记录。</summary>
    [Serializable]
    public class PlyEntry
    {
        public string relativePath;
        public RenderMeta metadata;
    }
}
