// SPDX-License-Identifier: MIT
// ============================================================================
// SplatLoadUI.cs —— 阶段 1 UI（简报 §四.7 / 进度 v2 §四④）
// 使用方式：场景里放一个空 GameObject 挂本组件（零 Inspector 配置）。
// Awake 自动构建：Canvas(复用或自建) + 按钮"选择 PLY 文件" + 进度条 Slider
//                + 百分比文本 + 状态文本（左上角垂直排布，CanvasScaler 1920x1080）。
// 事件流：按钮 → FilePicker.OpenFile() → await service.LoadAsync(path)。
// 信息分层：加载中 = 阶段描述+百分比+文件名；完成 = splatCount/sceneId/fromCache/耗时；
//           失败 = OnError 红色文本（catch 兜底打完整堆栈）。
// 字体：LegacyRuntime.ttf（Unity 6000 内置），Arial 兜底（旧版编辑器）。
// ============================================================================
using System;
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
        // ---- 布局常量（左上角垂直排布，margin 20，间距 10）----
        const float k_Margin = 20f;
        const float k_Spacing = 10f;
        const float k_ButtonWidth = 180f;
        const float k_ButtonHeight = 44f;
        const float k_SliderWidth = 360f;
        const float k_SliderHeight = 24f;
        const float k_PercentWidth = 64f;
        const float k_StatusWidth = 640f;
        const float k_StatusHeight = 140f;
        const int k_FontSizeButton = 22;
        const int k_FontSizeText = 24;

        readonly SplatRenderService m_Service = new();
        IPlatformFilePicker m_Picker;

        Button m_Button;
        Slider m_Progress;
        Text m_PercentText;
        Text m_StatusText;

        bool m_Busy;
        Stopwatch m_Watch;
        string m_LoadingFileName;

        void Awake()
        {
            m_Picker = CreatePicker();
            BuildUI();
            m_Service.OnProgress += OnProgress;
            m_Service.OnError += OnError;
        }

        void OnDestroy()
        {
            m_Service.OnProgress -= OnProgress;
            m_Service.OnError -= OnError;
        }

        static IPlatformFilePicker CreatePicker()
        {
#if UNITY_EDITOR
            return new EditorFilePicker();
#else
            return null; // 阶段 1：非 Editor 平台未实现（接口预留）
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

            // 按钮
            m_Button = CreateButton(canvas.transform, "SplatLoadButton", "选择 PLY 文件",
                k_ButtonWidth, k_ButtonHeight);
            SetTopLeft(m_Button.GetComponent<RectTransform>(), k_Margin, -k_Margin);

            // 进度条 + 百分比文本
            float sliderY = -(k_Margin + k_ButtonHeight + k_Spacing);
            m_Progress = CreateSlider(canvas.transform, "SplatLoadProgress", k_SliderWidth, k_SliderHeight);
            SetTopLeft(m_Progress.GetComponent<RectTransform>(), k_Margin, sliderY);
            m_PercentText = CreateText(canvas.transform, "SplatLoadPercent", "0%",
                k_PercentWidth, k_SliderHeight, k_FontSizeText, TextAnchor.MiddleLeft);
            SetTopLeft(m_PercentText.rectTransform, k_Margin + k_SliderWidth + k_Spacing, sliderY);

            // 状态文本（多行）
            m_StatusText = CreateText(canvas.transform, "SplatLoadStatus",
                "就绪：点击上方按钮选择 PLY 文件",
                k_StatusWidth, k_StatusHeight, k_FontSizeText, TextAnchor.UpperLeft);
            SetTopLeft(m_StatusText.rectTransform, k_Margin,
                -(k_Margin + k_ButtonHeight + k_Spacing + k_SliderHeight + k_Spacing));

            m_Button.onClick.AddListener(OnPickFileClicked);
        }

        /// <summary>左上角锚定（anchor/pivot 均为左上，anchoredPosition = 相对左上角偏移）。</summary>
        static void SetTopLeft(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(0f, 1f);
            rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = new Vector2(x, y);
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
            text.fontSize = k_FontSizeButton;
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

            // Fill Area → Fill（Filled 横向，Slider 接管 fillAmount）
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
            var fillImage = fillGo.AddComponent<Image>();
            fillImage.type = Image.Type.Filled;
            fillImage.fillMethod = Image.FillMethod.Horizontal;
            fillImage.fillOrigin = 0; // Left
            fillImage.fillAmount = 0f;
            fillImage.color = new Color(0.2f, 0.8f, 0.35f, 1f);
            fillImage.raycastTarget = false;

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

        // ================= 事件 =================

        /// <summary>加载进度（0~1，主线程回调）→ 进度条 + 阶段描述。</summary>
        void OnProgress(float p)
        {
            m_Progress.SetValueWithoutNotify(p);
            int percent = Mathf.RoundToInt(p * 100f);
            m_PercentText.text = percent + "%";
            m_StatusText.text = $"{PhaseName(p)} ({percent}%)\n正在加载：{Path.GetFileName(m_LoadingFileName)}";
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
            m_StatusText.color = Color.red;
            m_StatusText.text = "加载失败：\n" + message;
        }

        async void OnPickFileClicked()
        {
            if (m_Busy) return;
            if (m_Picker == null)
            {
                m_StatusText.color = Color.red;
                m_StatusText.text = "当前平台未实现文件选择（阶段 1 仅支持 Editor）";
                return;
            }

            string path = m_Picker.OpenFile();
            if (string.IsNullOrEmpty(path)) return; // 用户取消

            m_Busy = true;
            m_Button.interactable = false; // 防重入
            m_LoadingFileName = path;
            m_StatusText.color = Color.white;
            m_StatusText.text = $"正在加载：{Path.GetFileName(path)}";
            m_Watch = Stopwatch.StartNew();

            try
            {
                SplatIngestResult result = await m_Service.LoadAsync(path);
                m_StatusText.color = Color.white;
                m_StatusText.text =
                    $"完成：{result.splatCount} splats\n" +
                    $"sceneId={result.sceneId}\n" +
                    $"fromCache={result.fromCache}  耗时 {m_Watch.Elapsed.TotalSeconds:F1}s";
            }
            catch (Exception e)
            {
                // OnError 已负责 UI 红色提示；此处兜底防 async void unhandled + 打印完整堆栈
                m_StatusText.color = Color.red;
                m_StatusText.text = "加载失败：\n" + e.Message;
                UnityEngine.Debug.LogException(e);
            }
            finally
            {
                m_Watch.Stop();
                m_Busy = false;
                m_Button.interactable = true;
            }
        }
    }
}
