// SPDX-License-Identifier: MIT
// ============================================================================
// TestWindowsFilePicker.cs —— 调试工具（2026-08-18）
// 打包版文件选择器弹不出窗口的排查：在 Editor 里直接调用 WindowsFilePicker
// （P/Invoke 与打包版同路径），先排除"代码问题 vs 打包环境问题"。
// WindowsFilePicker 类被 #if UNITY_STANDALONE_WIN 包住，Windows Editor
// 同样定义该符号，可直接实例化。
// 用法：菜单 Tools -> Test WindowsFilePicker
// ============================================================================
#if UNITY_EDITOR && UNITY_STANDALONE_WIN
using QGStudio.SplatLoader;
using UnityEditor;
using UnityEngine;

namespace QGStudio.DebugTools
{
    public static class TestWindowsFilePicker
    {
        [MenuItem("Tools/Test WindowsFilePicker")]
        public static void Run()
        {
            // Editor 菜单回调处于"菜单模态"状态，此时弹 Win32 模态对话框会被立即取消
            // （表现为 GetOpenFileNameW 返回 false 且错误码 0）——延迟到菜单关闭后再执行
            EditorApplication.delayCall += () =>
            {
                Debug.Log("[TestWindowsFilePicker] 开始调用 WindowsFilePicker.OpenFile()...");
                var picker = new WindowsFilePicker();
                string path = picker.OpenFile();
                if (string.IsNullOrEmpty(path))
                    Debug.Log("[TestWindowsFilePicker] 返回 null（取消/失败——失败会弹错误码窗）");
                else
                    Debug.Log($"[TestWindowsFilePicker] 选中: {path}");
            };
        }
    }
}
#endif
