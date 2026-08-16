// SPDX-License-Identifier: MIT
//
// IRenderHandler 的示例实现 —— 只打日志。
// 照着这个结构写你自己的真实渲染逻辑（黑盒），把 RenderPly 内部换成
// 真实的"PLY -> 高斯资产 -> 渲染"即可。

using UnityEngine;

namespace RenderServer
{
    public class RenderHandlerExample : MonoBehaviour, IRenderHandler
    {
        [Tooltip("测试用：设为 false 可模拟图形端拒绝开始渲染")]
        public bool acceptStart = true;

        public bool OnStartRender()
        {
            Debug.Log("[RenderHandlerExample] AI 发起实时渲染 -> 返回 " + acceptStart);
            return acceptStart;
        }

        public void RenderPly(string plyPath)
        {
            Debug.Log($"[RenderHandlerExample] RenderPly(\"{plyPath}\")");
            // TODO: 在这里实现真实渲染：
            //   1. 读 plyPath 的 PLY
            //   2. 转成 GaussianSplatAsset（Editor-only，见 GaussianSplatAssetCreator）
            //   3. 交给 GaussianSplatRenderer 渲染
        }

        public void OnRenderEnd()
        {
            Debug.Log("[RenderHandlerExample] 本轮渲染结束，可以清场景/释放资源");
        }
    }
}
