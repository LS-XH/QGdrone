// SPDX-License-Identifier: MIT
//
// 渲染回调接口（黑盒）—— 由用户在场景里自己实现，接到消息时被调用。
// RenderClient 会自动找场景里实现该接口的组件。

namespace RenderServer
{
    public interface IRenderHandler
    {
        /// <summary>
        /// AI 发起"实时渲染"请求。返回 true = 图形端接受、开始渲染；返回 false = 拒绝。
        /// 返回值会原样回给 AI。
        /// </summary>
        bool OnStartRender();

        /// <summary>
        /// 收到一个 PLY（服务器已把文件存好，这里只给本地地址，不用管传输/保存）。
        /// 内部逻辑（把 PLY 转成高斯资产并渲染）由你实现。
        /// </summary>
        void RenderPly(string plyPath);

        /// <summary>
        /// 本轮渲染结束，可在这里收尾（清场景、释放资源等）。
        /// </summary>
        void OnRenderEnd();
    }
}
