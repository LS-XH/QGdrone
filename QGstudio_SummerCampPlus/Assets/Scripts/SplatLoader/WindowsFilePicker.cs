// SPDX-License-Identifier: MIT
// ============================================================================
// WindowsFilePicker.cs —— Windows 打包版文件选择（P/Invoke 系统对话框）
//
// 方案 A（2026-08-18 用户批准）：comdlg32.dll 的 GetOpenFileNameW，
//   零外部依赖、出包干净；Editor 版见 SplatFilePicker.cs（EditorUtility），互不干扰。
// 附带"记住上次目录"：进程内静态字段，下次打开对话框直接定位上次成功选择的目录。
//
// 行为与 Editor 版一致：模态阻塞；返回所选文件绝对路径；用户取消/失败返回 null。
// ============================================================================
#if UNITY_STANDALONE_WIN
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    /// <summary>Windows 打包版：调用系统"打开文件"对话框（经典款式）。</summary>
    public class WindowsFilePicker : IPlatformFilePicker
    {
        // ---- 记住上次目录（进程内共享：UI 重建/换场景也保留）----
        static string s_LastDirectory;

        // ---- WIN32 常量 ----
        const int OFN_FILEMUSTEXIST = 0x1000;   // 只能选真实存在的文件
        const int OFN_PATHMUSTEXIST = 0x800;    // 路径必须存在
        const int OFN_EXPLORER      = 0x80000;  // 新式对话框样式
        const int OFN_NOCHANGEDIR   = 0x8;      // 不改变进程当前目录（防 Unity 相对路径被对话框改掉）
        const int k_FileBufferSize  = 4096;     // 路径缓冲区（远超 MAX_PATH，防超长路径截断）

        [DllImport("comdlg32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        static extern bool GetOpenFileNameW(ref OpenFileName ofn);

        [DllImport("comdlg32.dll")]
        static extern uint CommDlgExtendedError();

        [DllImport("user32.dll")]
        static extern IntPtr GetActiveWindow();

        /// <summary>
        /// OPENFILENAME 结构体（Unicode 版）。
        /// 注意：lpstrFile 必须用 StringBuilder —— P/Invoke 封送器调用后会把
        /// 对话框写入的路径回拷到托管侧（string 字段是单向拷贝，写了也读不回来）。
        /// </summary>
        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        struct OpenFileName
        {
            public int lStructSize;
            public IntPtr hwndOwner;
            public IntPtr hInstance;
            public string lpstrFilter;
            public string lpstrCustomFilter;
            public int nMaxCustFilter;
            public int nFilterIndex;
            public StringBuilder lpstrFile;     // 可变缓冲区：对话框写入，调用后回读
            public int nMaxFile;
            public string lpstrFileTitle;
            public int nMaxFileTitle;
            public string lpstrInitialDir;
            public string lpstrTitle;
            public int Flags;
            public short nFileOffset;
            public short nFileExtension;
            public string lpstrDefExt;
            public IntPtr lCustData;
            public IntPtr lpfnHook;
            public string lpTemplateName;
            public IntPtr pvReserved;
            public int dwReserved;
            public int flagsEx;
        }

        public string OpenFile()
        {
            try
            {
                var ofn = new OpenFileName
                {
                    lStructSize = Marshal.SizeOf(typeof(OpenFileName)),
                    hwndOwner = GetActiveWindow(), // 对话框挂在 Unity 主窗口下，保证前置
                    lpstrFilter = "PLY 点云文件 (*.ply)\0*.ply\0所有文件 (*.*)\0*.*",
                    nFilterIndex = 1,
                    lpstrFile = new StringBuilder(k_FileBufferSize),
                    nMaxFile = k_FileBufferSize,
                    lpstrInitialDir = s_LastDirectory,
                    lpstrTitle = "选择 PLY 点云文件",
                    Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER | OFN_NOCHANGEDIR,
                };

                // 首次打开：默认定位"我的文档"（打包版当前目录 = exe 目录，不适合当默认）
                if (string.IsNullOrEmpty(ofn.lpstrInitialDir))
                    ofn.lpstrInitialDir = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);

                bool ok = GetOpenFileNameW(ref ofn);
                if (!ok)
                {
                    // 区分"用户取消"与"对话框失败"（CommDlgExtendedError 返回 0 = 用户取消）
                    uint err = CommDlgExtendedError();
                    if (err != 0)
                        Debug.LogWarning($"[WindowsFilePicker] 打开文件对话框失败，错误码 0x{err:X8}");
                    return null;
                }

                string path = ofn.lpstrFile.ToString().TrimEnd('\0');
                if (string.IsNullOrEmpty(path))
                    return null;

                s_LastDirectory = Path.GetDirectoryName(path); // 记住目录，下次直接定位
                return path;
            }
            catch (Exception e)
            {
                // 防御：任何封送异常都不应让 UI 崩溃（契约：失败返回 null）
                Debug.LogWarning($"[WindowsFilePicker] 打开文件对话框异常: {e.Message}");
                return null;
            }
        }
    }
}
#endif
