using System.IO;
using GaussianSplatting.Runtime;
using UnityEngine;
using UnityEngine.InputSystem;

// 阶段0 验证脚本 v2：byte[] 通道 = 磁盘直读 Creator 产物（与 TextAsset 完全同源）。
// 背景（2026-08-15 实测）：TextAsset.bytes 在 Play 模式下 60s+ 不加载（Unity 资源惰性/异步加载时序问题，
// 返回空数组），而 .bytes 文件本体就在磁盘上 → byte[] 通道无需依赖 TextAsset，直接读文件注入。
// 这也正是改造的意义：运行时数据流不依赖 Unity 资源加载时序。
// 挂载：运行时自动创建；按 T 键切换 byte[] / TextAsset 通道。
public class RuntimeDataSmokeTest : MonoBehaviour
{
    GaussianSplatRenderer m_Renderer;
    GaussianSplatAsset m_OriginalAsset; // 场景里 Creator 转的资产（TextAsset 通道）
    GaussianSplatAsset m_RuntimeAsset;  // 运行时构建（byte[] 通道，数据=磁盘 Creator 产物）
    bool m_UsingRuntime;
    bool m_RuntimeAssetReady;
    string m_FileBase; // Creator 产物文件前缀（磁盘绝对路径）

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AutoAttach()
    {
        // 阶段 0 验证脚本默认关闭（2026-08-17 起：它每次 Play 白占 ~270MB，且 T 键会
        // 把渲染器切回 Creator 资产、覆盖 Service 挂载的缓存资产）。
        // 需要重新验证 byte[] 通道时：kEnabled 改为 true 重新编译即可，脚本与验证逻辑保留。
        const bool kEnabled = false;
        if (!kEnabled) return;
        var go = new GameObject("RuntimeDataSmokeTest");
        DontDestroyOnLoad(go);
        go.AddComponent<RuntimeDataSmokeTest>();
    }

    void Start()
    {
        m_Renderer = FindFirstObjectByType<GaussianSplatRenderer>();
        if (m_Renderer == null || m_Renderer.asset == null)
        {
            Debug.LogError("[SmokeTest] 场景里没找到挂资产的 GaussianSplatRenderer");
            return;
        }
        m_OriginalAsset = m_Renderer.asset;

#if UNITY_EDITOR
        // 由 .asset 路径推导 Creator 产物文件路径（命名规则见 GaussianSplatAssetCreator.cs 301-305 行）
        string assetPath = UnityEditor.AssetDatabase.GetAssetPath(m_OriginalAsset); // "Assets/GaussianAssets/xxx.asset"
        string baseRel = assetPath.Substring(0, assetPath.Length - ".asset".Length); // "Assets/GaussianAssets/xxx"
        m_FileBase = Path.Combine(Path.GetDirectoryName(Application.dataPath), baseRel); // 磁盘绝对路径
#else
        // 构建产物无 AssetDatabase —— 运行时场景应走阶段 2a 的持久化缓存（SplatAssetCache）
        m_FileBase = null;
#endif

        // 症状速查：只有天空盒时，这行日志直接指出 GatherSplatsForCamera 检查的哪个条件为 false
        Debug.Log($"[SmokeTest] 初始状态: HasValidAsset={m_Renderer.HasValidAsset} " +
                  $"HasValidRenderSetup={m_Renderer.HasValidRenderSetup} " +
                  $"(True/False = 结构有效但缓冲未建成，即 TextAsset 数据未就绪)");
        Debug.Log($"[SmokeTest] 原资产: {m_OriginalAsset.splatCount} 点 | 产物前缀: {m_FileBase}");
    }

    void Update()
    {
        if (m_OriginalAsset == null || m_FileBase == null) return;

        // 数据就绪前不响应按键（构建完成后自动继续）
        if (!m_RuntimeAssetReady)
        {
            TryBuildRuntimeAsset();
            return;
        }

        if (Keyboard.current != null && Keyboard.current.tKey.wasPressedThisFrame)
        {
            m_UsingRuntime = !m_UsingRuntime;
            m_Renderer.m_Asset = m_UsingRuntime ? m_RuntimeAsset : m_OriginalAsset; // Update() 自动重建
            Debug.Log($"[SmokeTest] 切换到 {(m_UsingRuntime ? "byte[] 通道(磁盘直读)" : "TextAsset 通道")}, " +
                      $"HasValidRenderSetup={m_Renderer.HasValidRenderSetup} (下一帧应变为 True)");
        }
    }

    void TryBuildRuntimeAsset()
    {
        // 磁盘直读 Creator 产物（与 TextAsset 完全同源：.bytes 文件就是 TextAsset 的资源体）
        byte[] pos   = ReadFileOrNull(m_FileBase + "_pos.bytes");
        byte[] other = ReadFileOrNull(m_FileBase + "_oth.bytes");
        byte[] color = ReadFileOrNull(m_FileBase + "_col.bytes");
        byte[] sh    = ReadFileOrNull(m_FileBase + "_shs.bytes");
        byte[] chunk = ReadFileOrNull(m_FileBase + "_chk.bytes");
        if (pos == null || pos.Length < 4 || other == null || other.Length < 4 ||
            color == null || color.Length < 4 || sh == null || sh.Length < 4)
        {
            Debug.LogError($"[SmokeTest] 产物文件缺失或为空: pos={DataLen(pos)} other={DataLen(other)} " +
                           $"color={DataLen(color)} sh={DataLen(sh)} chunk={DataLen(chunk)}");
            m_FileBase = null; // 停止重试，避免刷屏
            return;
        }
        Debug.Log($"[SmokeTest] 磁盘直读成功: pos={pos.Length}B other={other.Length}B " +
                  $"color={color.Length}B sh={sh.Length}B chunk={DataLen(chunk)}");

        m_RuntimeAsset = ScriptableObject.CreateInstance<GaussianSplatAsset>();
        m_RuntimeAsset.Initialize(
            m_OriginalAsset.splatCount,
            m_OriginalAsset.posFormat, m_OriginalAsset.scaleFormat,
            m_OriginalAsset.colorFormat, m_OriginalAsset.shFormat,
            m_OriginalAsset.boundsMin, m_OriginalAsset.boundsMax,
            m_OriginalAsset.cameras);
        m_RuntimeAsset.SetDataHash(m_OriginalAsset.dataHash);
        m_RuntimeAsset.SetRuntimeData(pos, other, color, sh, chunk);

        m_RuntimeAssetReady = true;
        Debug.Log("[SmokeTest] byte[] 资产构建完成（数据源=磁盘 Creator 产物）。默认停在 TextAsset 通道，按 T 切换。");
    }

    static byte[] ReadFileOrNull(string path) => File.Exists(path) ? File.ReadAllBytes(path) : null;
    static string DataLen(byte[] bytes) => bytes == null ? "null" : bytes.Length.ToString();
}
