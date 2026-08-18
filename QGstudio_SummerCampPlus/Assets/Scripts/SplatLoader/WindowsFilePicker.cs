// SPDX-License-Identifier: MIT
// ============================================================================
// WindowsFilePicker.cs —— Windows 打包版文件选择（P/Invoke 系统对话框）
//
// 方案 A（2026-08-18 用户批准）：comdlg32.dll 的 GetOpenFileNameW，
//   零外部依赖、出包干净；Editor 版见 SplatFilePicker.cs（EditorUtility），互不干扰。
// 附带"记住上次目录"：进程内静态字段，下次打开对话框直接定位上次成功选择的目录。
//
// ⚠️ 2026-08-18 第二次重写（v3）：结构体全部 IntPtr 手动管理——
//   Mono 对结构体中 StringBuilder/string 字段的封送有差异（native 缓冲区可能按
//   Length 分配而非 Capacity），GetOpenFileNameW 写入 4096 字符时踩踏堆内存，
//   表现为 stack overflow / 弹出记事本等不可预测行为（已实测复现）。
//   纯值类型结构体（int/IntPtr/short）布局零歧义，内存由 AllocHGlobal 显式管理。
//
// 行为与 Editor 版一致：模态阻塞；返回所选文件绝对路径；用户取消/失败返回 null。
// ============================================================================
#if UNITY_STANDALONE_WIN
using System;
using System.IO;
using System.Runtime.InteropServices;
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
        const int k_FileBufferSize  = 4096;     // 路径缓冲区（字符数；远超 MAX_PATH，防超长路径截断）

        [DllImport("comdlg32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        static extern bool GetOpenFileNameW(ref OpenFileName ofn);

        [DllImport("comdlg32.dll")]
        static extern uint CommDlgExtendedError();

        [DllImport("user32.dll", CharSet = CharSet.Unicode)]
        static extern int MessageBoxW(IntPtr hWnd, string lpText, string lpCaption, uint uType);

        const uint MB_OK = 0x0;

        /// <summary>
        /// OPENFILENAME 结构体（Unicode 版，纯值类型——布局零歧义）。
        /// 所有字符串用 IntPtr 手动分配/释放（Marshal.StringToHGlobalUni /
        /// AllocHGlobal），不依赖 Mono 对 string/StringBuilder 字段的封送。
        /// 64 位下 lStructSize = 152（与 Win32 sizeof(OPENFILENAMEW) 一致）。
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        struct OpenFileName
        {
            public int lStructSize;
            public IntPtr hwndOwner;
            public IntPtr hInstance;
            public IntPtr lpstrFilter;
            public IntPtr lpstrCustomFilter;
            public int nMaxCustFilter;
            public int nFilterIndex;
            public IntPtr lpstrFile;        // 手动分配 4096 wchar 缓冲区
            public int nMaxFile;
            public IntPtr lpstrFileTitle;
            public int nMaxFileTitle;
            public IntPtr lpstrInitialDir;
            public IntPtr lpstrTitle;
            public int Flags;
            public short nFileOffset;
            public short nFileExtension;
            public IntPtr lpstrDefExt;
            public IntPtr lCustData;
            public IntPtr lpfnHook;
            public IntPtr lpTemplateName;
            public IntPtr pvReserved;
            public int dwReserved;
            public int flagsEx;
        }

        public string OpenFile()
        {
            // 手动分配的所有 native 内存（finally 统一释放，防泄漏/踩踏残留）
            IntPtr filterPtr = IntPtr.Zero;
            IntPtr titlePtr = IntPtr.Zero;
            IntPtr initialDirPtr = IntPtr.Zero;
            IntPtr fileBuf = IntPtr.Zero;
            try
            {
                int structSize = Marshal.SizeOf(typeof(OpenFileName)); // 诊断：应为 152
                var ofn = new OpenFileName
                {
                    lStructSize = structSize,
                    // 无 owner：对话框独立顶层显示（打包版全屏窗口下不跟随主窗口被盖住）
                    hwndOwner = IntPtr.Zero,
                    nFilterIndex = 1,
                    nMaxFile = k_FileBufferSize,
                    Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER | OFN_NOCHANGEDIR,
                };

                // filter：多段 \0 分隔（desc\0pattern\0...），StringToHGlobalUni 自动补尾 \0 → 双 \0 结尾
                filterPtr = Marshal.StringToHGlobalUni("PLY 点云文件 (*.ply)\0*.ply\0所有文件 (*.*)\0*.*");
                ofn.lpstrFilter = filterPtr;

                titlePtr = Marshal.StringToHGlobalUni("选择 PLY 点云文件");
                ofn.lpstrTitle = titlePtr;

                // 初始目录：记住的上次目录 > 我的文档（打包版当前目录 = exe 目录，不适合当默认）
                string initialDir = s_LastDirectory;
                if (string.IsNullOrEmpty(initialDir))
                    initialDir = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
                if (!string.IsNullOrEmpty(initialDir))
                {
                    initialDirPtr = Marshal.StringToHGlobalUni(initialDir);
                    ofn.lpstrInitialDir = initialDirPtr;
                }

                // 路径缓冲区：4096 wchar = 8192 字节（对话框写入，调用后读回）
                fileBuf = Marshal.AllocHGlobal(k_FileBufferSize * 2);
                ofn.lpstrFile = fileBuf;

                bool ok = GetOpenFileNameW(ref ofn);
                if (!ok)
                {
                    // 区分"用户取消"（err==0，静默）与"对话框失败"（err!=0，弹窗可见——Release 包无日志）
                    uint err = CommDlgExtendedError();
                    int win32Err = Marshal.GetLastWin32Error();
                    Debug.Log($"[WindowsFilePicker] GetOpenFileNameW=false: CommDlgErr=0x{err:X8}, Win32Err={win32Err}, structSize={structSize}, initialDir='{initialDir}'");
                    if (err != 0)
                    {
                        Debug.LogWarning($"[WindowsFilePicker] 打开文件对话框失败，错误码 0x{err:X8}");
                        MessageBoxW(IntPtr.Zero,
                            $"打开文件对话框失败（错误码 0x{err:X8}）。\n\n若在全屏模式下出现，请尝试窗口模式运行。",
                            "文件选择", MB_OK);
                    }
                    return null;
                }

                string path = Marshal.PtrToStringUni(fileBuf);
                if (string.IsNullOrEmpty(path))
                    return null;

                // 加固：对话框返回的路径必须真实存在（防异常缓冲区内容/幽灵路径进入加载链路）
                if (!File.Exists(path))
                {
                    Debug.LogWarning($"[WindowsFilePicker] 对话框返回的路径不存在: {path}");
                    MessageBoxW(IntPtr.Zero, $"所选文件不存在：\n{path}", "文件选择", MB_OK);
                    return null;
                }

                s_LastDirectory = Path.GetDirectoryName(path); // 记住目录，下次直接定位
                return path;
            }
            catch (Exception e)
            {
                // 防御：任何封送异常都不应让 UI 崩溃（契约：失败返回 null）
                Debug.LogWarning($"[WindowsFilePicker] 打开文件对话框异常: {e.Message}");
                return null;
            }
            finally
            {
                // 统一释放 native 内存（GetOpenFileNameW 失败/成功/异常都必须释放）
                if (filterPtr != IntPtr.Zero) Marshal.FreeHGlobal(filterPtr);
                if (titlePtr != IntPtr.Zero) Marshal.FreeHGlobal(titlePtr);
                if (initialDirPtr != IntPtr.Zero) Marshal.FreeHGlobal(initialDirPtr);
                if (fileBuf != IntPtr.Zero) Marshal.FreeHGlobal(fileBuf);
            }
        }
    }
}
#endif
