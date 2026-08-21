// SPDX-License-Identifier: MIT
// ============================================================================
// SplatLoadUI.cs —— 运行时 UI（阶段 1 基础 + 2026-08-17 UI 呈现改造 + 2026-08-21 2b 面板化）
// 使用方式：场景里放一个空 GameObject 挂本组件（零 Inspector 配置）。
// Awake 自动构建：
//   左上角垂直排布：进度条 Slider + 百分比 + 当前状态行（一行）
//     （原"选择 PLY 文件"按钮移交 SplatHistoryUI：左上角"选择模型"按钮 + 弹出历史面板）
//   左下角：消息框（控制台式：ScrollRect + 新消息追加底部 + 自动滚底 + 上限 30 条）
// 2b 公开成员（SplatHistoryUI 调用）：LoadPath(path) / OpenManualPick() / IsBusy
//   —— 与手动加载共用同一个 Service 实例（幂等/进度/消息栏全复用）。
// 进度条改造（2026-08-17）：
//   ① 根因修复：Fill 图像改 Simple 类型——UGUI Slider 的填充机制是改 fillRect.anchorMax.x，
//      原 Filled 类型 + fillAmount=0 导致填充永不渲染（数字能动、条不动）
//   ② 平滑动画：Update 里 MoveTowards 显示值→目标值，进度肉眼可见连续前进
//   ③ 失败变红：OnError 时填充色改红
// 消息栏触发点：开始加载（Info）/ 完成（Success）/ 失败（Error）/ 警告（Warning 预留）
// 事件流：按钮 → FilePicker.OpenFile() → await service.LoadAsync(path)。
// 字体：LegacyRuntime.ttf（Unity 6000 内置），Arial 兜底（旧版编辑器）。
// ============================================================================
using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem.UI;
#endif

namespace QGStudio.SplatLoader
{
    public class SplatLoadUI : MonoBehaviour
    {
        // ---- 布局参数（2026-08-18：const → [SerializeField]，Inspector 可调；默认值 = 原常量）----
        [Header("布局")]
        [SerializeField] float m_Margin = 20f;
        [SerializeField] float m_Spacing = 10f;
        [SerializeField] float m_ButtonWidth = 200f;
        [SerializeField] float m_ButtonHeight = 80f;
        [SerializeField] float m_SliderWidth = 300f;
        [SerializeField] float m_SliderHeight = 24f;
        [SerializeField] float m_PercentWidth = 64f;
        [SerializeField] float m_StatusWidth = 640f;
        [SerializeField] float m_StatusHeight = 30f;      // 改造：原 140 多行状态文本 → 一行当前状态
        [Header("字体")]
        [SerializeField] int m_FontSizeButton = 22;
        [SerializeField] int m_FontSizeText = 24;
        [SerializeField] int m_FontSizeStatus = 20;       // 当前状态行（小字）
        [SerializeField] int m_FontSizeMessage = 20;      // 消息栏条目
        [Header("消息栏")]
        [SerializeField] float m_MessageWidth = 520f;
        [SerializeField] float m_MessageHeight = 220f;
        [SerializeField] int m_MaxMessages = 30;          // 控制台式：保留最近 30 条，超出删最旧
        [Header("进度平滑")]
        [SerializeField] float m_ProgressSpeed = 0.5f;    // 显示值追赶目标值的速度（单位/秒）
        [Header("退出按钮")]
        [SerializeField] string m_ExitButtonLabel = "退出程序";
        [SerializeField] float m_ExitButtonWidth = 120f;
        [SerializeField] float m_ExitButtonHeight = 44f;
        [SerializeField] int m_ExitFontSize = 22;
        [SerializeField] Vector2 m_ExitOffset = new Vector2(-20f, -20f); // 右上角偏移（x 向左、y 向下）

        readonly SplatRenderService m_Service = new();
        IPlatformFilePicker m_Picker;

        Slider m_Progress;
        Image m_FillImage;                     // 进度条填充图（失败变红用）
        Text m_PercentText;
        Text m_CurrentStatusText;              // 改造：一行当前状态（原 640×140 多行状态文本）
        ScrollRect m_MessageScroll;
        RectTransform m_MessageContent;

        float m_DisplayProgress;               // 平滑显示值
        float m_TargetProgress;                // 目标值（OnProgress 设置）
        bool m_Busy;
        Stopwatch m_Watch;
        string m_LoadingFileName;

        // 网络层日志转发：Application.logMessageReceived 可能在后台线程触发（async 方法），
        // 用 ConcurrentQueue 收集，Update 主线程排空 → AddMessage（UGUI 主线程限定）。
        readonly ConcurrentQueue<(string msg, MessageType type)> m_NetworkLogQueue = new();

        enum MessageType { Info, Success, Error, Warning }

        void Awake()
        {
            m_Picker = CreatePicker();
            BuildUI();
            m_Service.OnProgress += OnProgress;
            m_Service.OnError += OnError;
            Application.logMessageReceived += OnLogMessageReceived;
            // 2b 历史模型面板：左上角"选择模型"按钮 + 弹出列表（零场景配置，自动挂载）
            if (GetComponent<SplatHistoryUI>() == null)
                gameObject.AddComponent<SplatHistoryUI>();
        }

        void OnDestroy()
        {
            m_Service.OnProgress -= OnProgress;
            m_Service.OnError -= OnError;
            Application.logMessageReceived -= OnLogMessageReceived;
        }

        void Update()
        {
            // 网络层日志排空（主线程，安全操作 UGUI）
            while (m_NetworkLogQueue.TryDequeue(out var item))
                AddMessage(item.msg, item.type);

            // 进度平滑：显示值向目标值靠拢（转换 12s 内肉眼可见连续前进，不再台阶跳变）
            if (Mathf.Abs(m_DisplayProgress - m_TargetProgress) > 0.001f)
            {
                m_DisplayProgress = Mathf.MoveTowards(m_DisplayProgress, m_TargetProgress, m_ProgressSpeed * Time.unscaledDeltaTime);
                m_Progress.SetValueWithoutNotify(m_DisplayProgress);
                m_PercentText.text = Mathf.RoundToInt(m_DisplayProgress * 100f) + "%";
            }
        }

        /// <summary>
        /// 捕获 [RenderClient] / [SessionManager] 前缀的 Debug.Log，转发到消息栏。
        /// 可能在后台线程触发（async WebSocket），只入队不在本回调里碰 UGUI。
        /// </summary>
        void OnLogMessageReceived(string condition, string stackTrace, LogType type)
        {
            if (!condition.Contains("[RenderClient]") &&
                !condition.Contains("[SessionManager]"))
                return;

            MessageType msgType = type switch
            {
                LogType.Error or LogType.Assert or LogType.Exception => MessageType.Error,
                LogType.Warning => MessageType.Warning,
                _ => MessageType.Info,
            };

            string msg = condition.Length > 220 ? condition.Substring(0, 220) + "…" : condition;
            m_NetworkLogQueue.Enqueue((msg, msgType));
        }

        static IPlatformFilePicker CreatePicker()
        {
#if UNITY_EDITOR
            return new EditorFilePicker();
#elif UNITY_STANDALONE_WIN
            return new WindowsFilePicker(); // 2026-08-18：打包版可用（P/Invoke 系统对话框 + 记忆上次目录）
#else
            return null; // 其他平台未实现（接口预留）
#endif
        }

        // ================= UI 构建 =================

        void BuildUI()
        {
            // Canvas：场景已有则复用（同 Binder 模式），否则自建 Overlay + CanvasScaler + GraphicRaycaster
            var canvas = FindFirstObjectByType<Canvas>();
            if (canvas == null)
            {
                var canvasGo = new GameObject("SplatLoadCanvas");
                canvas = canvasGo.AddComponent<Canvas>();
                canvas.renderMode = RenderMode.ScreenSpaceOverlay;
                var scaler = canvasGo.AddComponent<CanvasScaler>();
                scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
                scaler.referenceResolution = new Vector2(1920f, 1080f);
                scaler.matchWidthOrHeight = 0.5f;
                canvasGo.AddComponent<GraphicRaycaster>(); // 按钮点击必需
            }

            // EventSystem：场景无则自建（UGUI 点击事件派发器，缺失 = 按钮无任何反应）。
            // 项目为纯 InputSystem 模式（activeInputHandler:1），旧版 StandaloneInputModule
            // 运行时自我禁用 → 必须 InputSystemUIInputModule；条件编译兼容两种输入模式。
            var eventSystem = FindFirstObjectByType<EventSystem>();
            if (eventSystem == null)
            {
                var esGo = new GameObject("EventSystem");
                esGo.AddComponent<EventSystem>();
#if ENABLE_INPUT_SYSTEM
                esGo.AddComponent<InputSystemUIInputModule>();
#else
                esGo.AddComponent<StandaloneInputModule>(); // 旧输入模式兜底
#endif
            }

            // （2026-08-21 2b）左上角按钮移交 SplatHistoryUI"选择模型"（历史面板弹出）；
            // 手动选 PLY 移入面板底部按钮（OpenManualPick）。

            // 进度条 + 百分比文本（位置不变：原按钮下方）
            float sliderY = -(m_Margin + m_ButtonHeight + m_Spacing);
            m_Progress = CreateSlider(canvas.transform, "SplatLoadProgress", m_SliderWidth, m_SliderHeight);
            SetTopLeft(m_Progress.GetComponent<RectTransform>(), m_Margin, sliderY);
            m_PercentText = CreateText(canvas.transform, "SplatLoadPercent", "0%",
                m_PercentWidth, m_SliderHeight, m_FontSizeText, TextAnchor.MiddleLeft);
            SetTopLeft(m_PercentText.rectTransform, m_Margin + m_SliderWidth + m_Spacing, sliderY);

            // 当前状态行（改造：一行，原 640×140 多行状态文本的信息职责移交左下角消息栏）
            m_CurrentStatusText = CreateText(canvas.transform, "SplatLoadStatus",
                "就绪：点击左上角\"选择模型\"浏览或加载 PLY",
                m_StatusWidth, m_StatusHeight, m_FontSizeStatus, TextAnchor.UpperLeft);
            SetTopLeft(m_CurrentStatusText.rectTransform, m_Margin,
                -(m_Margin + m_ButtonHeight + m_Spacing + m_SliderHeight + m_Spacing));

            // 消息栏（左下角，控制台式滚动）
            CreateMessageBox(canvas.transform);
            AddMessage("就绪：点击左上角\"选择模型\"浏览或加载 PLY", MessageType.Info);

            // 退出按钮（右上角，2026-08-18 新增；打包版演示用）
            var exitBtn = CreateButton(canvas.transform, "SplatExitButton", m_ExitButtonLabel,
                m_ExitButtonWidth, m_ExitButtonHeight);
            exitBtn.onClick.AddListener(OnExitClicked);
            SetTopRight(exitBtn.GetComponent<RectTransform>(), m_ExitOffset.x, m_ExitOffset.y);
            var exitLabel = exitBtn.GetComponentInChildren<Text>();
            if (exitLabel != null) exitLabel.fontSize = m_ExitFontSize; // 应用退出按钮专属字体
        }

        /// <summary>左上角锚定（anchor/pivot 均为左上，anchoredPosition = 相对左上角偏移）。</summary>
        static void SetTopLeft(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(0f, 1f);
            rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = new Vector2(x, y);
        }

        /// <summary>左下角锚定（anchor/pivot 均为左下）。</summary>
        static void SetBottomLeft(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(0f, 0f);
            rt.anchorMax = new Vector2(0f, 0f);
            rt.pivot = new Vector2(0f, 0f);
            rt.anchoredPosition = new Vector2(x, y);
        }

        /// <summary>右上角锚定（anchor/pivot 均为右上）。</summary>
        static void SetTopRight(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(1f, 1f);
            rt.anchorMax = new Vector2(1f, 1f);
            rt.pivot = new Vector2(1f, 1f);
            rt.anchoredPosition = new Vector2(x, y);
        }

        /// <summary>退出程序（打包版）；Editor 里退 Play 模式方便测试。</summary>
        static void OnExitClicked()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }

        static Font s_Font;
        static Font GetBuiltinFont()
        {
            if (s_Font == null)
                s_Font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            if (s_Font == null)
                s_Font = Resources.GetBuiltinResource<Font>("Arial.ttf"); // 旧版编辑器兜底
            return s_Font;
        }

        Button CreateButton(Transform parent, string name, string label, float w, float h)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);

            var image = go.AddComponent<Image>();
            image.color = new Color(0.15f, 0.35f, 0.7f, 0.9f);

            var button = go.AddComponent<Button>();
            var colors = button.colors;
            colors.normalColor = new Color(0.15f, 0.35f, 0.7f, 0.9f);
            colors.highlightedColor = new Color(0.2f, 0.45f, 0.85f, 0.95f);
            colors.pressedColor = new Color(0.1f, 0.25f, 0.55f, 1f);
            colors.disabledColor = new Color(0.4f, 0.4f, 0.4f, 0.6f);
            button.colors = colors;

            var labelGo = new GameObject("Label", typeof(RectTransform));
            labelGo.transform.SetParent(go.transform, false);
            var labelRt = (RectTransform)labelGo.transform;
            labelRt.anchorMin = Vector2.zero;
            labelRt.anchorMax = Vector2.one;
            labelRt.offsetMin = Vector2.zero;
            labelRt.offsetMax = Vector2.zero;
            var text = labelGo.AddComponent<Text>();
            text.font = GetBuiltinFont();
            text.fontSize = m_FontSizeButton;
            text.alignment = TextAnchor.MiddleCenter;
            text.color = Color.white;
            text.text = label;
            text.raycastTarget = false; // 让点击穿透到按钮

            return button;
        }

        Slider CreateSlider(Transform parent, string name, float w, float h)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);

            var bg = go.AddComponent<Image>();
            bg.color = new Color(0.1f, 0.1f, 0.1f, 0.8f);

            var slider = go.AddComponent<Slider>();
            slider.interactable = false; // 纯展示，不可拖

            // Fill Area → Fill（Simple 类型！UGUI Slider 的填充机制 = 改 fillRect.anchorMax.x，
            // 不是 Image.fillAmount——Filled 类型 + fillAmount=0 会永远不渲染填充（2026-08-17 修复））
            var fillAreaGo = new GameObject("Fill Area", typeof(RectTransform));
            fillAreaGo.transform.SetParent(go.transform, false);
            var fillAreaRt = (RectTransform)fillAreaGo.transform;
            fillAreaRt.anchorMin = Vector2.zero;
            fillAreaRt.anchorMax = Vector2.one;
            fillAreaRt.offsetMin = new Vector2(5f, 5f);
            fillAreaRt.offsetMax = new Vector2(-5f, -5f);

            var fillGo = new GameObject("Fill", typeof(RectTransform));
            fillGo.transform.SetParent(fillAreaGo.transform, false);
            var fillRt = (RectTransform)fillGo.transform;
            fillRt.anchorMin = Vector2.zero;
            fillRt.anchorMax = Vector2.one;
            fillRt.offsetMin = Vector2.zero;
            fillRt.offsetMax = Vector2.zero;
            m_FillImage = fillGo.AddComponent<Image>(); // 默认 Simple 类型（Slider 用 anchorMax 拉伸）
            m_FillImage.color = new Color(0.2f, 0.8f, 0.35f, 1f);
            m_FillImage.raycastTarget = false;

            slider.fillRect = fillRt;
            slider.SetValueWithoutNotify(0f);
            return slider;
        }

        Text CreateText(Transform parent, string name, string content,
            float w, float h, int fontSize, TextAnchor align)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);

            var text = go.AddComponent<Text>();
            text.font = GetBuiltinFont();
            text.fontSize = fontSize;
            text.alignment = align;
            text.color = Color.white;
            text.text = content;
            text.raycastTarget = false;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            return text;
        }

        // ================= 消息栏（控制台式）=================

        /// <summary>
        /// 左下角消息框：半透明黑底 + Viewport(RectMask2D 裁剪) + Content(VerticalLayoutGroup
        /// 自动排布) + ScrollRect。新消息追加底部；用户停留在底部时自动滚底（旧消息上滚），
        /// 用户上翻查看历史时不打扰。
        /// </summary>
        void CreateMessageBox(Transform parent)
        {
            var go = new GameObject("SplatMessageBox", typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(m_MessageWidth, m_MessageHeight);
            SetBottomLeft(rt, m_Margin, m_Margin);
            var bg = go.AddComponent<Image>();
            bg.color = new Color(0f, 0f, 0f, 0.65f);

            // Viewport（裁剪内容）
            var viewportGo = new GameObject("Viewport", typeof(RectTransform));
            viewportGo.transform.SetParent(go.transform, false);
            var viewportRt = (RectTransform)viewportGo.transform;
            viewportRt.anchorMin = Vector2.zero;
            viewportRt.anchorMax = Vector2.one;
            viewportRt.offsetMin = new Vector2(8f, 8f);
            viewportRt.offsetMax = new Vector2(-8f, -8f);
            viewportGo.AddComponent<RectMask2D>();

            // Content（垂直自动排布 + 高度自适应）
            var contentGo = new GameObject("Content", typeof(RectTransform));
            contentGo.transform.SetParent(viewportGo.transform, false);
            var contentRt = (RectTransform)contentGo.transform;
            contentRt.anchorMin = new Vector2(0f, 1f); // 顶部锚定，宽度拉伸
            contentRt.anchorMax = new Vector2(1f, 1f);
            contentRt.pivot = new Vector2(0.5f, 1f);
            contentRt.sizeDelta = new Vector2(0f, 0f);
            var vlg = contentGo.AddComponent<VerticalLayoutGroup>();
            vlg.childAlignment = TextAnchor.UpperLeft;
            vlg.childForceExpandWidth = true;
            vlg.childForceExpandHeight = false;
            vlg.spacing = 2f;
            vlg.padding = new RectOffset(2, 2, 2, 2);
            var csf = contentGo.AddComponent<ContentSizeFitter>();
            csf.verticalFit = ContentSizeFitter.FitMode.PreferredSize; // 高度随消息增长

            var scroll = go.AddComponent<ScrollRect>();
            scroll.content = contentRt;
            scroll.viewport = viewportRt;
            scroll.horizontal = false;
            scroll.vertical = true;
            scroll.movementType = ScrollRect.MovementType.Clamped;
            scroll.scrollSensitivity = 20f;
            m_MessageScroll = scroll;
            m_MessageContent = contentRt;
        }

        /// <summary>追加一条消息（控制台行为：追加到底部，在底部时自动滚底）。</summary>
        void AddMessage(string text, MessageType type)
        {
            var entryGo = new GameObject($"Msg{m_MessageContent.childCount}", typeof(RectTransform));
            entryGo.transform.SetParent(m_MessageContent, false);
            var entryRt = (RectTransform)entryGo.transform;
            entryRt.sizeDelta = new Vector2(0f, 0f); // VerticalLayoutGroup 接管尺寸

            var t = entryGo.AddComponent<Text>();
            t.font = GetBuiltinFont();
            t.fontSize = m_FontSizeMessage;
            t.alignment = TextAnchor.UpperLeft;
            t.color = GetMessageColor(type);
            t.text = text;
            t.raycastTarget = false; // 不拦截滚动
            t.horizontalOverflow = HorizontalWrapMode.Wrap;
            t.verticalOverflow = VerticalWrapMode.Overflow;

            // 上限：删最旧
            while (m_MessageContent.childCount > m_MaxMessages)
                Destroy(m_MessageContent.GetChild(0).gameObject);

            // 立即重建布局；用户停在底部附近（verticalNormalizedPosition≈0=底部）才自动滚底
            LayoutRebuilder.ForceRebuildLayoutImmediate(m_MessageContent);
            if (m_MessageScroll.verticalNormalizedPosition < 0.01f)
                m_MessageScroll.verticalNormalizedPosition = 0f;
        }

        static Color GetMessageColor(MessageType type)
        {
            switch (type)
            {
                case MessageType.Info: return new Color(0.85f, 0.85f, 0.85f);
                case MessageType.Success: return new Color(0.35f, 0.85f, 0.4f);
                case MessageType.Error: return new Color(1f, 0.35f, 0.35f);
                case MessageType.Warning: return new Color(1f, 0.7f, 0.2f);
                default: return Color.white;
            }
        }

        // ================= 事件 =================

        /// <summary>加载进度（0~1，主线程回调）→ 目标值 + 当前状态行。</summary>
        void OnProgress(float p)
        {
            m_TargetProgress = p;
            int percent = Mathf.RoundToInt(p * 100f);
            m_CurrentStatusText.text = $"{PhaseName(p)} ({percent}%) · {Path.GetFileName(m_LoadingFileName)}";
        }

        static string PhaseName(float p)
        {
            if (p < 0.2f) return "解析 PLY 文件 (0-20%)";
            if (p < 0.6f) return "加工排序 (20-60%)";
            if (p < 0.9f) return "编码数据通道 (60-90%)";
            return "挂载渲染 (90-100%)";
        }

        /// <summary>加载错误（Service 契约：OnError + LoadAsync 抛异常，双通道）。</summary>
        void OnError(string message)
        {
            m_FillImage.color = new Color(1f, 0.35f, 0.35f, 1f); // 进度条变红
            m_CurrentStatusText.text = "加载失败：" + message;
            AddMessage("失败：" + message, MessageType.Error);
        }

        // ================ 2b：供 SplatHistoryUI 调用的公开成员 ================

        /// <summary>是否正在加载（历史面板防重入共用）。</summary>
        public bool IsBusy => m_Busy;

        /// <summary>
        /// 手动选择本地 PLY（历史面板底部按钮调用）。弹系统文件对话框，选完走 LoadPath。
        /// </summary>
        public void OpenManualPick()
        {
            if (m_Busy) return;
            if (m_Picker == null)
            {
                AddMessage("当前平台未实现文件选择", MessageType.Error);
                return;
            }

            string path = m_Picker.OpenFile();
            if (string.IsNullOrEmpty(path)) return; // 用户取消
            LoadPath(path);
        }

        /// <summary>
        /// 按路径加载（历史面板点击历史条目调用）：与手动加载同一条链路
        /// （幂等 / 缓存命中秒切 / 进度条 / 消息栏全复用）。
        /// </summary>
        public async void LoadPath(string path)
        {
            if (m_Busy) return;

            m_Busy = true;
            m_LoadingFileName = path;
            m_CurrentStatusText.text = $"正在加载：{Path.GetFileName(path)}";
            m_TargetProgress = 0f;
            m_DisplayProgress = 0f;
            m_Progress.SetValueWithoutNotify(0f);
            m_PercentText.text = "0%";
            if (m_FillImage != null) m_FillImage.color = new Color(0.2f, 0.8f, 0.35f, 1f); // 复位绿色
            m_Watch = Stopwatch.StartNew();
            AddMessage($"开始加载：{Path.GetFileName(path)}", MessageType.Info);

            try
            {
                SplatIngestResult result = await m_Service.LoadAsync(path);
                m_CurrentStatusText.text =
                    $"完成：{result.splatCount} splats（fromCache={result.fromCache}，耗时 {m_Watch.Elapsed.TotalSeconds:F1}s）";
                AddMessage(
                    $"完成：{result.splatCount} splats · fromCache={result.fromCache} · 耗时 {m_Watch.Elapsed.TotalSeconds:F1}s",
                    MessageType.Success);
            }
            catch (Exception e)
            {
                // OnError 已负责 UI 红色提示；此处兜底防 async void unhandled + 打印完整堆栈
                m_CurrentStatusText.text = "加载失败：" + e.Message;
                AddMessage("失败：" + e.Message, MessageType.Error);
                UnityEngine.Debug.LogException(e);
            }
            finally
            {
                m_Watch.Stop();
                m_Busy = false;
            }
        }
    }
}
