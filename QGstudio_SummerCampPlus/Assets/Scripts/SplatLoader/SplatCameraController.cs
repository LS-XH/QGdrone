// SPDX-License-Identifier: MIT
// ============================================================================
// SplatCameraController.cs —— 相机控制（2026-08-17）
// 经典移动：W/S 水平前后（沿相机朝向的水平投影，不爬坡）、A/D 水平左右、
//           Q/E 世界 Y 上下；按住 Shift 加速（×3）。
// 视角：右键按住拖动旋转（偏航 + 俯仰，俯仰 ±89° 防翻转）。
// 边界：硬钳制——相机位置每帧强制限制在 SplatRenderService.LastLoadedBounds
//       （模型包围盒，减质心后 = 世界系范围）内；未加载模型时不限制。
// 挂载：RuntimeInitializeOnLoadMethod 自动挂到 Main Camera（场景零配置），
//       速度/灵敏度为 [SerializeField]，Inspector 可调。
// ============================================================================
using UnityEngine;
using UnityEngine.InputSystem;

namespace QGStudio.SplatLoader
{
    public class SplatCameraController : MonoBehaviour
    {
        [SerializeField] float m_MoveSpeed = 5f;          // 移动速度（m/s）
        [SerializeField] float m_SprintMultiplier = 3f;   // Shift 加速倍数
        [SerializeField] float m_LookSensitivity = 0.15f; // 右键拖动灵敏度（度/像素）
        [SerializeField] float m_PitchClamp = 89f;        // 俯仰角限制（防翻转）

        float m_Yaw;
        float m_Pitch;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void AutoAttach()
        {
            var cam = Object.FindFirstObjectByType<Camera>();
            if (cam != null && cam.GetComponent<SplatCameraController>() == null)
                cam.gameObject.AddComponent<SplatCameraController>();
        }

        void Start()
        {
            // 从当前朝向初始化 yaw/pitch（避免右键首帧跳动）
            var e = transform.eulerAngles;
            m_Yaw = e.y;
            m_Pitch = e.x;
            ClampToBounds(); // 硬钳制：Play 开始就拉进包围盒
        }

        void Update()
        {
            // ---- 右键按住：视角旋转 ----
            if (Mouse.current != null && Mouse.current.rightButton.isPressed)
            {
                var delta = Mouse.current.delta.ReadValue();
                m_Yaw += delta.x * m_LookSensitivity;
                m_Pitch -= delta.y * m_LookSensitivity;
                m_Pitch = Mathf.Clamp(m_Pitch, -m_PitchClamp, m_PitchClamp);
                transform.rotation = Quaternion.Euler(m_Pitch, m_Yaw, 0f);
            }

            // ---- 移动（WASD 水平 + QE 世界 Y 上下）----
            if (Keyboard.current == null) return;
            var kbd = Keyboard.current;
            Vector3 move = Vector3.zero;
            // 水平前向：相机朝向的水平投影（QE 负责上下，WASD 不爬坡）
            Vector3 forward = Vector3.ProjectOnPlane(transform.forward, Vector3.up).normalized;
            Vector3 right = transform.right; // 纯偏航+俯仰旋转下右向仍水平
            if (kbd.wKey.isPressed) move += forward;
            if (kbd.sKey.isPressed) move -= forward;
            if (kbd.dKey.isPressed) move += right;
            if (kbd.aKey.isPressed) move -= right;
            if (kbd.qKey.isPressed) move += Vector3.up;
            if (kbd.eKey.isPressed) move -= Vector3.up;

            if (move.sqrMagnitude > 0f)
            {
                float speed = m_MoveSpeed * (kbd.shiftKey.isPressed ? m_SprintMultiplier : 1f);
                transform.position += move.normalized * speed * Time.deltaTime;
            }

            // ---- 硬钳制：相机永远在模型包围盒内（每帧强制，含新模型加载后的拉入）----
            ClampToBounds();
        }

        void ClampToBounds()
        {
            if (SplatRenderService.LastLoadedBounds is not { } b) return; // 未加载模型：不限制
            var p = transform.position;
            p.x = Mathf.Clamp(p.x, b.min.x, b.max.x);
            p.y = Mathf.Clamp(p.y, b.min.y, b.max.y);
            p.z = Mathf.Clamp(p.z, b.min.z, b.max.z);
            transform.position = p;
        }
    }
}
