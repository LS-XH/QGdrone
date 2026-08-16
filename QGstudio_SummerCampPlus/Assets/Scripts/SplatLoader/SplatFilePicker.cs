// SPDX-License-Identifier: MIT
// ============================================================================
// SplatFilePicker.cs —— 文件选择抽象（简报 §四.8 / 设计 §4）
// 阶段 1：仅实现 Editor 平台（EditorUtility.OpenFilePanel 弹系统原生对话框）。
// 非 Editor 平台：CreatePicker() 返回 null，UI 明确报"未实现"（接口预留，
// 后续真机接入时补 Windows/Mobile 实现，UI 侧零改动）。
// ============================================================================

namespace QGStudio.SplatLoader
{
    /// <summary>平台文件选择抽象：返回所选文件绝对路径；用户取消返回 null。</summary>
    public interface IPlatformFilePicker
    {
        /// <summary>弹出文件选择对话框。返回绝对路径；用户取消或失败返回 null。</summary>
        string OpenFile();
    }

#if UNITY_EDITOR
    /// <summary>Editor 实现：Unity 编辑器专属的系统原生打开对话框。</summary>
    public class EditorFilePicker : IPlatformFilePicker
    {
        public string OpenFile()
        {
            string path = UnityEditor.EditorUtility.OpenFilePanel("选择 PLY 点云文件", "", "ply");
            return string.IsNullOrEmpty(path) ? null : path;
        }
    }
#endif
}
