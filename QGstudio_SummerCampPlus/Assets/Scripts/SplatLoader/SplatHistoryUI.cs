// SPDX-License-Identifier: MIT
// ============================================================================
// SplatHistoryUI.cs —— 2b 历史模型面板（2026-08-21 v3 扁平手风琴）
// 由 SplatLoadUI.Awake 自动 AddComponent 挂载（零场景配置）。
//
// 左上角"选择模型"按钮 → 弹出面板（960×600，扁平深色风格）：
//   单列手风琴列表：
//     ▶ 建筑A                    (3)  [删]
//     ▼ 建筑B                    (2)  [删]       ← 点场景展开/收起版本
//         high · high_high_high_model_red.ply
//         6,131,954 splats · 2026-08-18          [删]
//     ── 独立模型 ──
//       [本地] demo3.ply                        [删]
//   左下角小按钮：手动选择
//
// 数据源：allModels.json（场景=modelName，版本=ply）+ splats/ 缓存（手动→独立模型）
// 删除：二次确认；场景整删=DeleteModel+逐版本清缓存；版本删=DeletePly+清缓存；
//       独立模型删=仅清缓存；缓存=splats/<Hash128.Compute(path)>/
// 加载：复用 SplatLoadUI.LoadPath（同一个 Service 实例，幂等/缓存命中秒切）。
// ============================================================================

using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using RenderServer;

namespace QGStudio.SplatLoader
{
    public class SplatHistoryUI : MonoBehaviour
    {
        [Header("选择模型按钮 / Toggle Button")]
        [Tooltip("按钮宽度 Width")] [SerializeField] float m_ToggleButtonWidth = 200f;
        [Tooltip("按钮高度 Height")] [SerializeField] float m_ToggleButtonHeight = 80f;
        [Tooltip("按钮文字大小 Font size")] [SerializeField] int m_ToggleFontSize = 24;
        [Tooltip("按钮文字 Label text")] [SerializeField] string m_ToggleButtonLabel = "选择模型";
        [Tooltip("按钮颜色 Button color")] [SerializeField] Color m_ToggleButtonColor = new(0.14f, 0.32f, 0.52f, 0.95f);

        [Header("面板 / Panel")]
        [Tooltip("面板宽度 Width")] [SerializeField] float m_PanelWidth = 960f;
        [Tooltip("面板高度 Height")] [SerializeField] float m_PanelHeight = 600f;
        [Tooltip("标题文字大小 Title font size")] [SerializeField] int m_TitleFontSize = 26;
        [Tooltip("标题文字 Title text")] [SerializeField] string m_TitleText = "选择模型";
        [Tooltip("面板背景色 Background color")] [SerializeField] Color m_PanelBgColor = new(0.06f, 0.07f, 0.09f, 0.93f);

        [Header("列表行 / List Rows")]
        [Tooltip("场景行高度 Scene row height")] [SerializeField] float m_SceneRowHeight = 56f;
        [Tooltip("版本行高度 Version row height")] [SerializeField] float m_VersionRowHeight = 56f;
        [Tooltip("分隔标题高度 Section header height")] [SerializeField] float m_SectionHeaderHeight = 36f;
        [Tooltip("场景名字号 Scene name font size")] [SerializeField] int m_SceneFontSize = 24;
        [Tooltip("版本数/次要文字字号 Count font size")] [SerializeField] int m_SceneCountFontSize = 18;
        [Tooltip("版本主文字字号 Version primary font size")] [SerializeField] int m_VersionPrimaryFontSize = 20;
        [Tooltip("版本详情字号 Version detail font size")] [SerializeField] int m_VersionDetailFontSize = 17;
        [Tooltip("删除按钮宽度 Delete button width")] [SerializeField] float m_DeleteButtonWidth = 44f;
        [Tooltip("版本行缩进像素 Version indent")] [SerializeField] float m_VersionIndent = 36f;
        [Tooltip("场景行背景色 Scene row color")] [SerializeField] Color m_SceneRowColor = new(0.12f, 0.14f, 0.18f, 0.88f);
        [Tooltip("场景行展开色 Scene row expanded color")] [SerializeField] Color m_SceneRowExpandedColor = new(0.16f, 0.28f, 0.42f, 0.92f);
        [Tooltip("版本行背景色 Version row color")] [SerializeField] Color m_VersionRowColor = new(0.08f, 0.09f, 0.12f, 0.75f);
        [Tooltip("独立模型行背景色 Local row color")] [SerializeField] Color m_LocalRowColor = new(0.10f, 0.12f, 0.16f, 0.85f);
        [Tooltip("删除按钮颜色 Delete button color")] [SerializeField] Color m_DeleteButtonColor = new(0.55f, 0.15f, 0.15f, 0.88f);
        [Tooltip("删除按钮文字 Delete label")] [SerializeField] string m_DeleteButtonLabel = "删";
        [Tooltip("关闭按钮文字 Close label")] [SerializeField] string m_CloseButtonLabel = "✕";

        [Header("文字颜色 / Text Colors")]
        [Tooltip("主文字色 Main text color")] [SerializeField] Color m_TextMainColor = Color.white;
        [Tooltip("次要文字色 Secondary text color")] [SerializeField] Color m_TextSubColor = new(0.62f, 0.64f, 0.68f);
        [Tooltip("提示/空状态文字色 Hint text color")] [SerializeField] Color m_TextHintColor = new(0.45f, 0.47f, 0.5f);

        [Header("交互颜色 / Interaction Colors")]
        [Tooltip("悬停色 Hover color")] [SerializeField] Color m_HoverColor = new(0.18f, 0.22f, 0.3f, 0.95f);
        [Tooltip("按下色 Pressed color")] [SerializeField] Color m_PressedColor = new(0.1f, 0.12f, 0.18f, 1f);
        [Tooltip("禁用色 Disabled color")] [SerializeField] Color m_DisabledColor = new(0.3f, 0.3f, 0.3f, 0.5f);

        [Header("手动选择按钮 / Manual Pick Button")]
        [Tooltip("按钮宽度 Width")] [SerializeField] float m_ManualButtonWidth = 140f;
        [Tooltip("按钮高度 Height")] [SerializeField] float m_ManualButtonHeight = 40f;
        [Tooltip("文字大小 Font size")] [SerializeField] int m_ManualFontSize = 17;
        [Tooltip("按钮文字 Label text")] [SerializeField] string m_ManualButtonLabel = "手动选择";
        [Tooltip("按钮颜色 Button color")] [SerializeField] Color m_ManualButtonColor = new(0.14f, 0.32f, 0.52f, 0.92f);

        [Header("确认弹窗 / Confirm Dialog")]
        [Tooltip("弹窗宽度 Width")] [SerializeField] float m_ConfirmWidth = 460f;
        [Tooltip("弹窗高度 Height")] [SerializeField] float m_ConfirmHeight = 220f;
        [Tooltip("文字大小 Font size")] [SerializeField] int m_ConfirmFontSize = 20;
        [Tooltip("确认按钮文字 Confirm label")] [SerializeField] string m_ConfirmDeleteLabel = "确认删除";
        [Tooltip("取消按钮文字 Cancel label")] [SerializeField] string m_CancelLabel = "取消";

        [Header("空状态提示 / Empty Hint")]
        [Tooltip("无模型时的提示文字 Empty hint text")] [SerializeField] string m_EmptyHintText = "暂无历史模型\n点击左下角\"手动选择\"加载 PLY";
        [Tooltip("独立模型分区标题 Section header")] [SerializeField] string m_LocalSectionTitle = "独立模型";

        [Header("引用 / References")]
        [Tooltip("SplatLoadUI 引用（拖入场景中挂 SplatLoadUI 的物体；留空自动查找）\nSplatLoadUI reference (drag here or auto-find)")]
        [SerializeField] SplatLoadUI m_LoadUI;

        SplatLoadUI LoadUI
        {
            get
            {
                if (m_LoadUI == null) m_LoadUI = FindFirstObjectByType<SplatLoadUI>();
                return m_LoadUI;
            }
        }
        Button m_ToggleButton;
        GameObject m_Panel;
        RectTransform m_ListContent;
        ScrollRect m_Scroll;
        GameObject m_ConfirmDialog;
        Text m_ConfirmText;
        Action m_PendingConfirmAction;

        string m_ExpandedScene; // 当前展开的场景（null=全收起）
        List<SceneGroup> m_Scenes = new();
        List<LocalEntry> m_Locals = new();

        static Font s_Font;

        class SceneGroup { public string name; public List<VersionEntry> versions = new(); }
        class VersionEntry { public string plyPath; public string fileName; public string precision; public string timestamp; public long splatCount; }
        class LocalEntry { public string plyPath; public string fileName; public string timestamp; public long splatCount; }

        static Font GetFont()
        {
            if (s_Font == null) s_Font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            if (s_Font == null) s_Font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            return s_Font;
        }

        static string SplatsRoot => Path.Combine(Application.persistentDataPath, "splats");
        static string NormPath(string p) => string.IsNullOrEmpty(p) ? "" : p.Replace('\\', '/').ToLowerInvariant();
        static DateTime ParseTime(string s) => !string.IsNullOrEmpty(s) && DateTime.TryParse(s, out var d) ? d : DateTime.MinValue;

        void Awake() {}
        void Start() { BuildUI(); }

        // ================= 构建 =================

        void BuildUI()
        {
            var canvas = FindFirstObjectByType<Canvas>();
            if (canvas == null) { Debug.LogError("[SplatHistoryUI] 无 Canvas"); return; }
            var root = canvas.transform;

            m_ToggleButton = CreateButton(root, "SelectModelButton", m_ToggleButtonLabel,
                m_ToggleButtonWidth, m_ToggleButtonHeight, m_ToggleButtonColor, m_ToggleFontSize);
            SetTopLeft(m_ToggleButton.GetComponent<RectTransform>(), 20f, -20f);
            m_ToggleButton.onClick.AddListener(OnToggleClicked);

            BuildPanel(root);
            BuildConfirmDialog(root);
        }

        void BuildPanel(Transform root)
        {
            var go = new GameObject("HistoryPanel", typeof(RectTransform));
            go.transform.SetParent(root, false);
            var rt = (RectTransform)go.transform;
            rt.anchorMin = new Vector2(0.5f, 0.5f);
            rt.anchorMax = new Vector2(0.5f, 0.5f);
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = new Vector2(m_PanelWidth, m_PanelHeight);
            go.AddComponent<Image>().color = m_PanelBgColor;
            m_Panel = go;
            m_Panel.SetActive(false);

            // 标题
            var title = CreateText(go.transform, "Title", m_TitleText,
                m_PanelWidth - 80f, 40f, m_TitleFontSize, TextAnchor.MiddleLeft, m_TextMainColor);
            SetTopLeft(title.rectTransform, 24f, -8f);

            // 关闭
            var closeBtn = CreateButton(go.transform, "CloseBtn", m_CloseButtonLabel, 40f, 40f, m_DeleteButtonColor, 20);
            var cr = closeBtn.GetComponent<RectTransform>();
            cr.anchorMin = new Vector2(1f, 1f); cr.anchorMax = new Vector2(1f, 1f);
            cr.pivot = new Vector2(1f, 1f); cr.anchoredPosition = new Vector2(-10f, -10f);
            closeBtn.onClick.AddListener(ClosePanel);

            // 列表视口（占满标题与底部之间，底部留手动按钮空间）
            var vpGo = new GameObject("ListViewport", typeof(RectTransform));
            vpGo.transform.SetParent(go.transform, false);
            var vpRt = (RectTransform)vpGo.transform;
            vpRt.anchorMin = Vector2.zero; vpRt.anchorMax = Vector2.one;
            vpRt.offsetMin = new Vector2(12f, 56f);   // 底部留 56px 给手动按钮
            vpRt.offsetMax = new Vector2(-12f, -52f);  // 顶部留标题
            vpGo.AddComponent<RectMask2D>();

            var contentGo = new GameObject("ListContent", typeof(RectTransform));
            contentGo.transform.SetParent(vpGo.transform, false);
            m_ListContent = (RectTransform)contentGo.transform;
            m_ListContent.anchorMin = new Vector2(0f, 1f); m_ListContent.anchorMax = new Vector2(1f, 1f);
            m_ListContent.pivot = new Vector2(0.5f, 1f); m_ListContent.sizeDelta = new Vector2(0f, 0f);
            var vlg = contentGo.AddComponent<VerticalLayoutGroup>();
            vlg.childAlignment = TextAnchor.UpperLeft;
            vlg.childForceExpandWidth = true; vlg.childForceExpandHeight = false;
            vlg.spacing = 2f; vlg.padding = new RectOffset(0, 0, 4, 4);
            var csf = contentGo.AddComponent<ContentSizeFitter>();
            csf.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            m_Scroll = vpGo.AddComponent<ScrollRect>();
            m_Scroll.content = m_ListContent; m_Scroll.viewport = vpRt;
            m_Scroll.horizontal = false; m_Scroll.vertical = true;
            m_Scroll.movementType = ScrollRect.MovementType.Elastic;
            m_Scroll.scrollSensitivity = 28f;

            // 左下角手动选择按钮（小）
            var manualBtn = CreateButton(go.transform, "ManualPickBtn", m_ManualButtonLabel,
                m_ManualButtonWidth, m_ManualButtonHeight, m_ManualButtonColor, m_ManualFontSize);
            var mrt = manualBtn.GetComponent<RectTransform>();
            mrt.anchorMin = new Vector2(0f, 0f); mrt.anchorMax = new Vector2(0f, 0f);
            mrt.pivot = new Vector2(0f, 0f); mrt.anchoredPosition = new Vector2(12f, 10f);
            manualBtn.onClick.AddListener(OnManualPickClicked);
        }

        void BuildConfirmDialog(Transform root)
        {
            var go = new GameObject("ConfirmDialog", typeof(RectTransform));
            go.transform.SetParent(root, false);
            var rt = (RectTransform)go.transform;
            rt.anchorMin = new Vector2(0.5f, 0.5f); rt.anchorMax = new Vector2(0.5f, 0.5f);
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = new Vector2(m_ConfirmWidth, m_ConfirmHeight);
            go.AddComponent<Image>().color = new Color(0.04f, 0.05f, 0.07f, 0.97f);
            m_ConfirmDialog = go; m_ConfirmDialog.SetActive(false);

            m_ConfirmText = CreateText(go.transform, "ConfirmText", "",
                m_ConfirmWidth - 40f, 100f, m_ConfirmFontSize, TextAnchor.UpperLeft, m_TextMainColor);
            SetTopLeft(m_ConfirmText.rectTransform, 20f, -18f);

            var cancelBtn = CreateButton(go.transform, "CancelBtn", m_CancelLabel, 150f, 48f,
                new Color(0.25f, 0.26f, 0.3f, 0.95f), m_ConfirmFontSize);
            var cr = cancelBtn.GetComponent<RectTransform>();
            cr.anchorMin = new Vector2(0f, 0f); cr.anchorMax = new Vector2(0f, 0f);
            cr.pivot = new Vector2(0f, 0f); cr.anchoredPosition = new Vector2(36f, 18f);
            cancelBtn.onClick.AddListener(() => { m_ConfirmDialog.SetActive(false); m_PendingConfirmAction = null; });

            var okBtn = CreateButton(go.transform, "OkBtn", m_ConfirmDeleteLabel, 150f, 48f, m_DeleteButtonColor, m_ConfirmFontSize);
            var or = okBtn.GetComponent<RectTransform>();
            or.anchorMin = new Vector2(1f, 0f); or.anchorMax = new Vector2(1f, 0f);
            or.pivot = new Vector2(1f, 0f); or.anchoredPosition = new Vector2(-36f, 18f);
            okBtn.onClick.AddListener(OnConfirmDelete);
        }

        // ================= 数据采集 =================

        void CollectData()
        {
            m_Scenes.Clear();
            m_Locals.Clear();
            var manifestPaths = new HashSet<string>();

            ModelsIndex idx;
            try { idx = SessionManager.GetModelsIndex(); } catch { idx = new ModelsIndex(); }
            if (idx == null) idx = new ModelsIndex();
            if (idx.models == null) idx.models = new List<ModelEntry>();

            foreach (var model in idx.models)
            {
                if (model.plies == null || model.plies.Count == 0) continue;
                var group = new SceneGroup { name = model.modelName };
                foreach (var ply in model.plies)
                {
                    manifestPaths.Add(NormPath(ply.relativePath));
                    long splats = 0; string ts = ply.metadata != null ? ply.metadata.timestamp : "";
                    var rec = TryLoadCacheRecord(ply.relativePath);
                    if (rec != null) { splats = rec.splatCount; if (string.IsNullOrEmpty(ts)) ts = rec.timestamp; }
                    group.versions.Add(new VersionEntry
                    {
                        plyPath = ply.relativePath,
                        fileName = Path.GetFileName(ply.relativePath),
                        precision = ply.metadata != null ? ply.metadata.precision : "",
                        timestamp = ts,
                        splatCount = splats,
                    });
                }
                group.versions.Sort((a, b) => ParseTime(b.timestamp).CompareTo(ParseTime(a.timestamp)));
                m_Scenes.Add(group);
            }
            m_Scenes.Sort((a, b) => string.Compare(a.name, b.name, StringComparison.Ordinal));

            var root = SplatsRoot;
            if (Directory.Exists(root))
            {
                foreach (var dir in Directory.GetDirectories(root))
                {
                    var metaPath = Path.Combine(dir, "meta.json");
                    if (!File.Exists(metaPath)) continue;
                    SplatAssetRecord rec;
                    try { rec = JsonUtility.FromJson<SplatAssetRecord>(File.ReadAllText(metaPath)); } catch { continue; }
                    if (rec == null || string.IsNullOrEmpty(rec.sourcePath)) continue;
                    if (manifestPaths.Contains(NormPath(rec.sourcePath))) continue;
                    m_Locals.Add(new LocalEntry
                    {
                        plyPath = rec.sourcePath,
                        fileName = Path.GetFileName(rec.sourcePath),
                        timestamp = rec.timestamp,
                        splatCount = rec.splatCount,
                    });
                }
            }
            m_Locals.Sort((a, b) => ParseTime(b.timestamp).CompareTo(ParseTime(a.timestamp)));
        }

        static SplatAssetRecord TryLoadCacheRecord(string plyPath)
        {
            try
            {
                string sceneId = Hash128.Compute(plyPath).ToString();
                string metaPath = Path.Combine(SplatsRoot, sceneId, "meta.json");
                if (!File.Exists(metaPath)) return null;
                return JsonUtility.FromJson<SplatAssetRecord>(File.ReadAllText(metaPath));
            }
            catch { return null; }
        }

        // ================= 列表构建（手风琴单列）=================

        void RebuildList()
        {
            for (int i = m_ListContent.childCount - 1; i >= 0; i--)
                Destroy(m_ListContent.GetChild(i).gameObject);

            if (m_Scenes.Count == 0 && m_Locals.Count == 0)
            {
                AddHint(m_EmptyHintText);
                return;
            }

            foreach (var scene in m_Scenes)
            {
                AddSceneRow(scene);
                if (m_ExpandedScene == scene.name)
                {
                    foreach (var v in scene.versions)
                        AddVersionRow(scene.name, v);
                }
            }

            if (m_Locals.Count > 0)
            {
                AddSectionHeader(m_LocalSectionTitle);
                foreach (var loc in m_Locals)
                    AddLocalRow(loc);
            }

            LayoutRebuilder.ForceRebuildLayoutImmediate(m_ListContent);
            m_Scroll.verticalNormalizedPosition = 1f;
        }

        void AddHint(string text)
        {
            var t = CreateText(m_ListContent, "Hint", text, 0f, 80f,
                m_VersionPrimaryFontSize, TextAnchor.MiddleCenter, m_TextHintColor);
            var r = t.rectTransform;
            r.anchorMin = new Vector2(0f, 0.5f); r.anchorMax = new Vector2(1f, 0.5f);
            r.pivot = new Vector2(0.5f, 0.5f); r.anchoredPosition = Vector2.zero;
        }

        void AddSectionHeader(string title)
        {
            var go = new GameObject("Header_" + title, typeof(RectTransform));
            go.transform.SetParent(m_ListContent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(0f, m_SectionHeaderHeight);
            go.AddComponent<LayoutElement>().preferredHeight = m_SectionHeaderHeight;
            var t = go.AddComponent<Text>();
            t.font = GetFont(); t.fontSize = m_SceneCountFontSize; t.color = m_TextHintColor;
            t.text = "— " + title + " —"; t.alignment = TextAnchor.MiddleCenter;
            t.raycastTarget = false;
        }

        // ---- 场景行（可展开/收起）----

        void AddSceneRow(SceneGroup scene)
        {
            bool expanded = m_ExpandedScene == scene.name;
            string arrow = expanded ? "▼" : "▶";

            var rowGo = new GameObject("Scene_" + scene.name, typeof(RectTransform));
            rowGo.transform.SetParent(m_ListContent, false);
            var rowRt = (RectTransform)rowGo.transform;
            rowRt.sizeDelta = new Vector2(0f, m_SceneRowHeight);
            rowGo.AddComponent<LayoutElement>().preferredHeight = m_SceneRowHeight;

            var bg = new GameObject("BG", typeof(RectTransform));
            bg.transform.SetParent(rowGo.transform, false);
            var bgRt = (RectTransform)bg.transform;
            bgRt.anchorMin = Vector2.zero; bgRt.anchorMax = Vector2.one;
            bgRt.offsetMin = Vector2.zero; bgRt.offsetMax = Vector2.zero;
            var bgImg = bg.AddComponent<Image>();
            bgImg.color = expanded ? m_SceneRowExpandedColor : m_SceneRowColor;

            var btn = bg.AddComponent<Button>();
            ApplyButtonColors(btn, expanded ? m_SceneRowExpandedColor : m_SceneRowColor);
            btn.onClick.AddListener(() => OnSceneClicked(scene.name));

            // 展开箭头
            var arrowTxt = CreateText(bg.transform, "Arrow", arrow, 28f, m_SceneRowHeight,
                m_SceneCountFontSize, TextAnchor.MiddleCenter, m_TextSubColor);
            var ar = arrowTxt.rectTransform;
            ar.anchorMin = new Vector2(0f, 0.5f); ar.anchorMax = new Vector2(0f, 0.5f);
            ar.pivot = new Vector2(0f, 0.5f); ar.anchoredPosition = new Vector2(12f, 0f);
            arrowTxt.raycastTarget = false;

            // 场景名
            var nameTxt = CreateText(bg.transform, "Name", scene.name, 0f, m_SceneRowHeight,
                m_SceneFontSize, TextAnchor.MiddleLeft, m_TextMainColor);
            var nr = nameTxt.rectTransform;
            nr.anchorMin = Vector2.zero; nr.anchorMax = Vector2.one;
            nr.offsetMin = new Vector2(44f, 0f); nr.offsetMax = new Vector2(-(m_DeleteButtonWidth + 60f), 0f);
            nameTxt.raycastTarget = false;

            // 版本数
            var countTxt = CreateText(bg.transform, "Count", $"({scene.versions.Count})", 50f, m_SceneRowHeight,
                m_SceneCountFontSize, TextAnchor.MiddleRight, m_TextSubColor);
            var cr2 = countTxt.rectTransform;
            cr2.anchorMin = new Vector2(1f, 0.5f); cr2.anchorMax = new Vector2(1f, 0.5f);
            cr2.pivot = new Vector2(1f, 0.5f); cr2.anchoredPosition = new Vector2(-(m_DeleteButtonWidth + 8f), 0f);
            countTxt.raycastTarget = false;

            // 删除按钮
            var delBtn = CreateButton(bg.transform, "Del", m_DeleteButtonLabel,
                m_DeleteButtonWidth, m_SceneRowHeight, m_DeleteButtonColor, m_SceneCountFontSize);
            var dr = delBtn.GetComponent<RectTransform>();
            dr.anchorMin = new Vector2(1f, 0.5f); dr.anchorMax = new Vector2(1f, 0.5f);
            dr.pivot = new Vector2(1f, 0.5f); dr.anchoredPosition = Vector2.zero;
            delBtn.onClick.AddListener(() => RequestDeleteScene(scene));
        }

        // ---- 版本行（缩进，展开时显示）----

        void AddVersionRow(string sceneName, VersionEntry v)
        {
            var rowGo = new GameObject("Ver_" + v.fileName, typeof(RectTransform));
            rowGo.transform.SetParent(m_ListContent, false);
            var rowRt = (RectTransform)rowGo.transform;
            rowRt.sizeDelta = new Vector2(0f, m_VersionRowHeight);
            rowGo.AddComponent<LayoutElement>().preferredHeight = m_VersionRowHeight;

            var img = rowGo.AddComponent<Image>();
            img.color = m_VersionRowColor;
            var btn = rowGo.AddComponent<Button>();
            ApplyButtonColors(btn, m_VersionRowColor);
            btn.onClick.AddListener(() => OnVersionClicked(v));

            // 左侧缩进条（视觉层次）
            var indent = new GameObject("Indent", typeof(RectTransform));
            indent.transform.SetParent(rowGo.transform, false);
            var indRt = (RectTransform)indent.transform;
            indRt.anchorMin = new Vector2(0f, 0f); indRt.anchorMax = new Vector2(0f, 1f);
            indRt.pivot = new Vector2(0f, 0.5f);
            indRt.sizeDelta = new Vector2(4f, 0f);
            indRt.anchoredPosition = new Vector2(m_VersionIndent - 8f, 0f);
            indent.AddComponent<Image>().color = new Color(0.3f, 0.5f, 0.75f, 0.6f);

            // 两行文字：第一行=精度+文件名，第二行=点数+时间
            var label = CreateText(rowGo.transform, "Label", "", 0f, m_VersionRowHeight,
                m_VersionPrimaryFontSize, TextAnchor.MiddleLeft, m_TextMainColor);
            var lr = label.rectTransform;
            lr.anchorMin = Vector2.zero; lr.anchorMax = Vector2.one;
            lr.offsetMin = new Vector2(m_VersionIndent, 0f);
            lr.offsetMax = new Vector2(-(m_DeleteButtonWidth + 8f), 0f);
            label.raycastTarget = false;
            label.lineSpacing = 0.8f;
            label.supportRichText = true;

            string prec = !string.IsNullOrEmpty(v.precision) ? v.precision : "未知精度";
            string line2 = v.splatCount > 0 ? $"{v.splatCount:N0} splats" : "点数未知";
            if (!string.IsNullOrEmpty(v.timestamp)) line2 += "  ·  " + v.timestamp;
            label.text = $"<size={m_VersionPrimaryFontSize}>{prec}</size>" +
                         $"  <size={m_VersionDetailFontSize}><color=#{ColorToHex(m_TextSubColor)}>{v.fileName}</color></size>\n" +
                         $"<size={m_VersionDetailFontSize}><color=#{ColorToHex(m_TextSubColor)}>{line2}</color></size>";

            var delBtn = CreateButton(rowGo.transform, "Del", m_DeleteButtonLabel,
                m_DeleteButtonWidth, m_VersionRowHeight, m_DeleteButtonColor, m_SceneCountFontSize);
            var dr = delBtn.GetComponent<RectTransform>();
            dr.anchorMin = new Vector2(1f, 0.5f); dr.anchorMax = new Vector2(1f, 0.5f);
            dr.pivot = new Vector2(1f, 0.5f); dr.anchoredPosition = Vector2.zero;
            delBtn.onClick.AddListener(() => RequestDeleteVersion(sceneName, v));
        }

        // ---- 独立模型行 ----

        void AddLocalRow(LocalEntry loc)
        {
            var rowGo = new GameObject("Local_" + loc.fileName, typeof(RectTransform));
            rowGo.transform.SetParent(m_ListContent, false);
            var rowRt = (RectTransform)rowGo.transform;
            rowRt.sizeDelta = new Vector2(0f, m_SceneRowHeight);
            rowGo.AddComponent<LayoutElement>().preferredHeight = m_SceneRowHeight;

            var img = rowGo.AddComponent<Image>();
            img.color = m_LocalRowColor;
            var btn = rowGo.AddComponent<Button>();
            ApplyButtonColors(btn, m_LocalRowColor);
            btn.onClick.AddListener(() => OnLocalClicked(loc));

            var label = CreateText(rowGo.transform, "Label", "", 0f, m_SceneRowHeight,
                m_VersionPrimaryFontSize, TextAnchor.MiddleLeft, m_TextMainColor);
            var lr = label.rectTransform;
            lr.anchorMin = Vector2.zero; lr.anchorMax = Vector2.one;
            lr.offsetMin = new Vector2(m_VersionIndent, 0f);
            lr.offsetMax = new Vector2(-(m_DeleteButtonWidth + 8f), 0f);
            label.raycastTarget = false;
            label.supportRichText = true;
            string line2 = loc.splatCount > 0 ? $"{loc.splatCount:N0} splats" : "";
            if (!string.IsNullOrEmpty(loc.timestamp)) line2 += (line2.Length > 0 ? "  ·  " : "") + loc.timestamp;
            label.text = $"<size={m_SceneCountFontSize}><color=#{ColorToHex(m_TextSubColor)}>[本地]</color></size>  " +
                         $"<size={m_VersionPrimaryFontSize}>{loc.fileName}</size>" +
                         (line2.Length > 0 ? $"\n<size={m_VersionDetailFontSize}><color=#{ColorToHex(m_TextSubColor)}>{line2}</color></size>" : "");

            var delBtn = CreateButton(rowGo.transform, "Del", m_DeleteButtonLabel,
                m_DeleteButtonWidth, m_SceneRowHeight, m_DeleteButtonColor, m_SceneCountFontSize);
            var dr = delBtn.GetComponent<RectTransform>();
            dr.anchorMin = new Vector2(1f, 0.5f); dr.anchorMax = new Vector2(1f, 0.5f);
            dr.pivot = new Vector2(1f, 0.5f); dr.anchoredPosition = Vector2.zero;
            delBtn.onClick.AddListener(() => RequestDeleteLocal(loc));
        }

        // ================= 交互 =================

        void OnToggleClicked()
        {
            if (m_Panel.activeSelf) ClosePanel();
            else OpenPanel();
        }

        void OpenPanel()
        {
            CollectData();
            if (!string.IsNullOrEmpty(m_ExpandedScene) && !m_Scenes.Exists(s => s.name == m_ExpandedScene))
                m_ExpandedScene = null;
            RebuildList();
            m_Panel.SetActive(true);
            m_ConfirmDialog.SetActive(false);
            LoadUI?.SetUIVisible(false); // 隐去其他 UI（进度条/消息栏/退出按钮），消息记录保留
        }

        void ClosePanel()
        {
            m_Panel.SetActive(false);
            m_ConfirmDialog.SetActive(false);
            m_PendingConfirmAction = null;
            LoadUI?.SetUIVisible(true); // 复原其他 UI
        }

        void OnSceneClicked(string sceneName)
        {
            // 点击已展开场景 → 收起；点击别的 → 展开它（收起原来的）
            m_ExpandedScene = (m_ExpandedScene == sceneName) ? null : sceneName;
            RebuildList();
        }

        void OnVersionClicked(VersionEntry v)
        {
            if (LoadUI == null || LoadUI.IsBusy) { Debug.Log("[SplatHistoryUI] 正在加载，请稍候"); return; }
            ClosePanel();
            LoadUI.LoadPath(v.plyPath);
        }

        void OnLocalClicked(LocalEntry loc)
        {
            if (LoadUI == null || LoadUI.IsBusy) { Debug.Log("[SplatHistoryUI] 正在加载，请稍候"); return; }
            ClosePanel();
            LoadUI.LoadPath(loc.plyPath);
        }

        void OnManualPickClicked()
        {
            ClosePanel();
            LoadUI?.OpenManualPick();
        }

        // ---- 删除（二次确认）----

        void RequestDeleteScene(SceneGroup scene)
        {
            m_ConfirmText.text = $"确认删除整个场景「{scene.name}」？\n将删除其 {scene.versions.Count} 个版本的所有 PLY + 清单记录 + 本地缓存";
            m_PendingConfirmAction = () => DoDeleteScene(scene);
            m_ConfirmDialog.SetActive(true);
            m_ConfirmDialog.transform.SetAsLastSibling();
        }

        void RequestDeleteVersion(string sceneName, VersionEntry v)
        {
            m_ConfirmText.text = $"确认删除版本「{v.fileName}」？\n将删除该 PLY + 清单记录 + 本地缓存";
            m_PendingConfirmAction = () => DoDeleteVersion(sceneName, v);
            m_ConfirmDialog.SetActive(true);
            m_ConfirmDialog.transform.SetAsLastSibling();
        }

        void RequestDeleteLocal(LocalEntry loc)
        {
            m_ConfirmText.text = $"确认删除「{loc.fileName}」？\n将仅清理本地缓存（原 PLY 文件保留）";
            m_PendingConfirmAction = () => DoDeleteLocal(loc);
            m_ConfirmDialog.SetActive(true);
            m_ConfirmDialog.transform.SetAsLastSibling();
        }

        void OnConfirmDelete()
        {
            var action = m_PendingConfirmAction;
            m_ConfirmDialog.SetActive(false);
            m_PendingConfirmAction = null;
            action?.Invoke();
            CollectData();
            if (!string.IsNullOrEmpty(m_ExpandedScene) && !m_Scenes.Exists(s => s.name == m_ExpandedScene))
                m_ExpandedScene = null;
            RebuildList();
        }

        void DoDeleteScene(SceneGroup scene)
        {
            try { SessionManager.DeleteModel(scene.name); }
            catch (Exception e) { Debug.LogError($"[SplatHistoryUI] DeleteModel 失败: {e.Message}"); }
            foreach (var v in scene.versions) DeleteCacheFor(v.plyPath);
            if (m_ExpandedScene == scene.name) m_ExpandedScene = null;
        }

        void DoDeleteVersion(string sceneName, VersionEntry v)
        {
            try { SessionManager.DeletePly(sceneName, v.plyPath); }
            catch (Exception e) { Debug.LogError($"[SplatHistoryUI] DeletePly 失败: {e.Message}"); }
            DeleteCacheFor(v.plyPath);
        }

        void DoDeleteLocal(LocalEntry loc) => DeleteCacheFor(loc.plyPath);

        static void DeleteCacheFor(string plyPath)
        {
            var root = SplatsRoot;
            try
            {
                string sceneId = Hash128.Compute(plyPath).ToString();
                string dir = Path.Combine(root, sceneId);
                if (Directory.Exists(dir)) Directory.Delete(dir, true);
            }
            catch (Exception e) { Debug.LogWarning($"[SplatHistoryUI] 按 Hash 删缓存失败: {e.Message}"); }
            if (!Directory.Exists(root)) return;
            var target = NormPath(plyPath);
            foreach (var dir in Directory.GetDirectories(root))
            {
                var metaPath = Path.Combine(dir, "meta.json");
                if (!File.Exists(metaPath)) continue;
                try
                {
                    var rec = JsonUtility.FromJson<SplatAssetRecord>(File.ReadAllText(metaPath));
                    if (rec != null && NormPath(rec.sourcePath) == target && Directory.Exists(dir))
                        Directory.Delete(dir, true);
                }
                catch { }
            }
        }

        // ================= UGUI 辅助 =================

        static string ColorToHex(Color c)
        {
            byte r = (byte)(c.r * 255), g = (byte)(c.g * 255), b = (byte)(c.b * 255);
            return $"{r:X2}{g:X2}{b:X2}";
        }

        void ApplyButtonColors(Button btn, Color normal)
        {
            var colors = btn.colors;
            colors.normalColor = normal;
            colors.highlightedColor = m_HoverColor;
            colors.pressedColor = m_PressedColor;
            colors.disabledColor = m_DisabledColor;
            btn.colors = colors;
        }

        static void SetTopLeft(RectTransform rt, float x, float y)
        {
            rt.anchorMin = new Vector2(0f, 1f); rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f); rt.anchoredPosition = new Vector2(x, y);
        }

        Button CreateButton(Transform parent, string name, string label, float w, float h, Color bg, int fontSize)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);
            go.AddComponent<Image>().color = bg;
            var button = go.AddComponent<Button>();
            ApplyButtonColors(button, bg);

            var labelGo = new GameObject("Label", typeof(RectTransform));
            labelGo.transform.SetParent(go.transform, false);
            var labelRt = (RectTransform)labelGo.transform;
            labelRt.anchorMin = Vector2.zero; labelRt.anchorMax = Vector2.one;
            labelRt.offsetMin = Vector2.zero; labelRt.offsetMax = Vector2.zero;
            var text = labelGo.AddComponent<Text>();
            text.font = GetFont(); text.fontSize = fontSize;
            text.alignment = TextAnchor.MiddleCenter; text.color = Color.white;
            text.text = label; text.raycastTarget = false;
            return button;
        }

        Text CreateText(Transform parent, string name, string content,
            float w, float h, int fontSize, TextAnchor align, Color color)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            var rt = (RectTransform)go.transform;
            rt.sizeDelta = new Vector2(w, h);
            var text = go.AddComponent<Text>();
            text.font = GetFont(); text.fontSize = fontSize;
            text.alignment = align; text.color = color;
            text.text = content; text.raycastTarget = false;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            return text;
        }
    }
}
