// SPDX-License-Identifier: MIT
// ============================================================================
// SplatRotationControl.cs —— 右下角 X/Z 旋转控制面板（2026-08-18）
//
// 目的：真实数据训练出的高斯模型 up 方向与引擎 up 不对齐时，手动旋转
//       **渲染器对象**（GaussianSplatRenderer.transform）对齐，帮助渲染测试。
// 交互：右下角 X / Z 两个黑框显示当前角度，点击输入具体角度；每框下方
//       [+5°] [-5°] 两个按钮微调。
// 行为：rotation = Quaternion.Euler(x, 0, z) 每帧应用到渲染器对象——
//       Binder 换资产只赋 m_Asset 不重建 GameObject，对象稳定，每帧应用
//       即可自动跟随"重新加载/换版本"。
//       角度为静态字段（进程内记忆：重新加载模型后保留，测试流程
//       "调好角度 → 重新加载验证"成立）。
// 挂载：RuntimeInitializeOnLoadMethod 自动挂载（场景零配置，同相机控制器）。
// 参数：全部 UI 尺寸/位置/步进为 [SerializeField]，Inspector 可调。
// ============================================================================
using GaussianSplatting.Runtime;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem.UI;
#endif

namespace QGStudio.SplatLoader
{
    public class SplatRotationControl : MonoBehaviour
    {
        // ================= Inspector 暴露参数（面板可调） =================
        [Header("面板布局")]
        [SerializeField] Vector2 m_PanelOffset = new Vector2(-20f, 20f); // 右下角偏移（x 向左、y 向上）
        [SerializeField] float m_ColumnSpacing = 16f;  // X 组与 Z 组水平间距
        [SerializeField] float m_GroupSpacing = 4f;   // 组内元素纵向间距
        [SerializeField] float m_LabelWidth = 48f;
        [SerializeField] float m_LabelHeight = 30f;
        [SerializeField] float m_InputWidth = 100f;
        [SerializeField] float m_InputHeight = 32f;
        [SerializeField] float m_ButtonWidth = 52f;
        [SerializeField] float m_ButtonHeight = 28f;
        [SerializeField] float m_ButtonSpacing = 4f;  // [+5][-5] 两按钮水平间距

        [Header("字体")]
        [SerializeField] int m_FontSizeLabel = 18;
        [SerializeField] int m_FontSizeInput = 20;
        [SerializeField] int m_FontSizeButton = 18;

        [Header("行为")]
        [SerializeField] float m_StepDegrees = 5f;    // +5/-5 步进（可改）
        [SerializeField] string m_XLabel = "X 旋转";
        [SerializeField] string m_ZLabel = "Z 旋转";

        // ================= 角度状态（静态：进程内记忆） =================
        static float s_AngleX;
        static float s_AngleZ;

        // ================= 运行时引用 =================
        RectTransform m_Panel;
        InputField m_XInput;
        InputField m_ZInput;
        GaussianSplatRenderer m_Renderer;

        // ================= 自动挂载（场景零配置） =================
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void AutoAttach()
        {
            if (FindFirstObjectByType<SplatRotationControl>() != null) return;
            var go = new GameObject("SplatRotationControl");
            DontDestroyOnLoad(go); // 组件跨场景保留（角度记忆在静态字段；UI 重建逻辑兜底）
            go.AddComponent<SplatRotationControl>();
        }

        void Awake()
        {
            TryBuildUI();
        }

        void Update()
        {
            // UI 兜底重建（场景切换导致 Canvas/面板丢失时）
            if (m_Panel == null)
                TryBuildUI();

            // 渲染器对象缓存（场景唯一；销毁后 fake-null 自动重查）
            if (m_Renderer == null)
                m_Renderer = FindFirstObjectByType<GaussianSplatRenderer>();

            // 每帧应用旋转（换资产/重新加载后自动跟随）
            if (m_Renderer != null)
                m_Renderer.transform.rotation = Quaternion.Euler(s_AngleX, 0f, s_AngleZ);
        }

        // ================= UI 构建 =================

        void TryBuildUI()
        {
            var canvas = FindFirstObjectByType<Canvas>();
            if (canvas == null)
            {
                var canvasGo = new GameObject("SplatRotationCanvas");
                canvas = canvasGo.AddComponent<Canvas>();
                canvas.renderMode = RenderMode.ScreenSpaceOverlay;
                canvasGo.AddComponent<CanvasScaler>().uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
                canvasGo.AddComponent<GraphicRaycaster>();
                DontDestroyOnLoad(canvasGo);
            }

            // EventSystem：场景无则自建（InputField/Button 点击必需；SplatLoadUI 可能已建）
            if (FindFirstObjectByType<EventSystem>() == null)
            {
                var esGo = new GameObject("EventSystem");
                esGo.AddComponent<EventSystem>();
#if ENABLE_INPUT_SYSTEM
                esGo.AddComponent<InputSystemUIInputModule>();
#else
                esGo.AddComponent<StandaloneInputModule>();
#endif
            }

            // ---- 面板根（右下锚定）----
            float colWidth = Mathf.Max(m_InputWidth, m_LabelWidth, m_ButtonWidth * 2f + m_ButtonSpacing);
            float colHeight = m_LabelHeight + m_GroupSpacing + m_InputHeight + m_GroupSpacing + m_ButtonHeight;
            float panelW = colWidth * 2f + m_ColumnSpacing;
            float panelH = colHeight;

            var panelGo = new GameObject("SplatRotationPanel", typeof(RectTransform));
            panelGo.transform.SetParent(canvas.transform, false);
            m_Panel = (RectTransform)panelGo.transform;
            m_Panel.sizeDelta = new Vector2(panelW, panelH);
            SetBottomRight(m_Panel, m_PanelOffset.x, m_PanelOffset.y);

            // ---- X 组（左侧列）----
            m_XInput = BuildColumn(m_Panel, "X", m_XLabel, colWidth, colHeight);
            // ---- Z 组（右侧列）----
            m_ZInput = BuildColumn(m_Panel, "Z", m_ZLabel, colWidth, colHeight);
            var zRt = (RectTransform)m_ZInput.transform.parent;
            zRt.anchoredPosition = new Vector2(colWidth + m_ColumnSpacing, 0f);
        }

        /// <summary>构建一列：标签（上）→ 输入框（中）→ [+步进][-步进]（下）。返回输入框。</summary>
        InputField BuildColumn(Transform parent, string axis, string label, float colWidth, float colHeight)
        {
            var colGo = new GameObject($"{axis}Group", typeof(RectTransform));
            colGo.transform.SetParent(parent, false);
            var colRt = (RectTransform)colGo.transform;
            colRt.sizeDelta = new Vector2(colWidth, colHeight);
            SetTopLeft(colRt, 0f, 0f);

            // 标签
            var labelText = CreateText(colRt, $"{axis}Label", label, m_LabelWidth, m_LabelHeight,
                m_FontSizeLabel, TextAnchor.MiddleLeft);
            SetTopLeft((RectTransform)labelText.transform, 0f, 0f);

            // 黑框输入（lambda 内通过方法取 ref，避免捕获 ref 参数 CS1628）
            var inputField = CreateInputField(colRt, $"{axis}Input", m_InputWidth, m_InputHeight,
                axis == "X" ? s_AngleX : s_AngleZ);
            inputField.onEndEdit.AddListener(s => ApplyInput(inputField, ref GetAngleRef(axis), s));
            SetTopLeft((RectTransform)inputField.transform, 0f, -(m_LabelHeight + m_GroupSpacing));

            // +步进 / -步进
            float buttonY = -(m_LabelHeight + m_GroupSpacing + m_InputHeight + m_GroupSpacing);
            float step = m_StepDegrees;
            var plusBtn = CreateButton(colRt, $"{axis}Plus", $"+{step:0.#}°", m_ButtonWidth, m_ButtonHeight,
                () => StepAngle(axis, +step));
            SetTopLeft((RectTransform)plusBtn.transform, 0f, buttonY);
            var minusBtn = CreateButton(colRt, $"{axis}Minus", $"-{step:0.#}°", m_ButtonWidth, m_ButtonHeight,
                () => StepAngle(axis, -step));
            SetTopLeft((RectTransform)minusBtn.transform, m_ButtonWidth + m_ButtonSpacing, buttonY);

            return inputField;
        }

        ref float GetAngleRef(string axis) => ref (axis == "X" ? ref s_AngleX : ref s_AngleZ);

        void ApplyInput(InputField field, ref float angle, string text)
        {
            if (float.TryParse(text, out float v))
            {
                angle = v;
                field.text = FormatAngle(angle);
            }
            else
            {
                field.text = FormatAngle(angle); // 非法输入：恢复显示当前值
            }
        }

        void StepAngle(string axis, float delta)
        {
            ref float angle = ref GetAngleRef(axis);
            angle += delta;
            var field = axis == "X" ? m_XInput : m_ZInput;
            if (field != null)
                field.text = FormatAngle(angle);
        }

        static string FormatAngle(float v) => v.ToString("0.##");

        // ================= UGUI 构建原语 =================

        static Font s_Font;
        static Font GetBuiltinFont()
        {
            if (s_Font == null)
                s_Font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            if (s_Font == null)
                s_Font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            return s_Font;
        }

        static void SetTopLeft(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(0f, 1f);
            rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = new Vector2(x, y);
        }

        static void SetBottomRight(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(1f, 0f);
            rt.anchorMax = new Vector2(1f, 0f);
            rt.pivot = new Vector2(1f, 0f);
            rt.anchoredPosition = new Vector2(x, y);
        }

        Text CreateText(Transform parent, string name, string content, float w, float h, int fontSize, TextAnchor align)
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
            return text;
        }

        InputField CreateInputField(Transform parent, string name, float w, float h, float initialAngle)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);

            // 黑底
            var img = go.AddComponent<Image>();
            img.color = new Color(0f, 0f, 0f, 0.85f);

            var input = go.AddComponent<InputField>();
            input.characterValidation = InputField.CharacterValidation.Decimal; // 只允许数字/小数点/负号

            // 文本组件
            var textGo = new GameObject("Text", typeof(RectTransform));
            textGo.transform.SetParent(go.transform, false);
            var textRt = (RectTransform)textGo.transform;
            textRt.anchorMin = Vector2.zero;
            textRt.anchorMax = Vector2.one;
            textRt.offsetMin = new Vector2(6f, 0f);
            textRt.offsetMax = new Vector2(-6f, 0f);
            var text = textGo.AddComponent<Text>();
            text.font = GetBuiltinFont();
            text.fontSize = m_FontSizeInput;
            text.alignment = TextAnchor.MiddleLeft;
            text.color = Color.white;
            text.raycastTarget = false;
            input.textComponent = text;

            input.text = FormatAngle(initialAngle);
            return input;
        }

        Button CreateButton(Transform parent, string name, string label, float w, float h,
            UnityEngine.Events.UnityAction onClick)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);

            var img = go.AddComponent<Image>();
            img.color = new Color(0.15f, 0.35f, 0.7f, 0.9f);

            var button = go.AddComponent<Button>();
            button.onClick.AddListener(onClick);

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
            text.raycastTarget = false;
            return button;
        }
    }
}
