// SPDX-License-Identifier: MIT
//
// 一键把 RenderClient + 示例回调放到当前场景，方便联调。
// 用法：菜单 RenderServer -> Setup Scene

using UnityEditor;
using UnityEngine;

namespace RenderServer.EditorTools
{
    public static class RenderServerSetupMenu
    {
        [MenuItem("RenderServer/Setup Scene")]
        public static void SetupScene()
        {
            var existing = Object.FindFirstObjectByType<RenderClient>();
            if (existing != null)
            {
                Selection.activeGameObject = existing.gameObject;
                Debug.Log($"[RenderServer] 场景已有 RenderClient（{existing.gameObject.name}），已选中");
                return;
            }

            var go = new GameObject("RenderServer");
            go.AddComponent<RenderClient>();
            var example = go.AddComponent<RenderHandlerExample>();

            // 把示例回调直接接进 RenderClient 的"渲染回调"字段，
            // 避免 Inspector 显示 None / Console 报"找不到渲染回调"。
            var so = new SerializedObject(go.GetComponent<RenderClient>());
            so.FindProperty("m_RenderHandler").objectReferenceValue = example;
            so.ApplyModifiedProperties();

            Selection.activeGameObject = go;
            Debug.Log("[RenderServer] 已在当前场景创建 RenderServer 节点（RenderClient + 示例回调，已接入）");
        }
    }
}
