namespace PointCloud
{
    /// <summary>
    /// 解析输出：纯数据容器，不碰引擎。
    /// 由 PLYtoPointCloud.Load() 填充，供渲染层（MonoBehaviour）消费。
    /// </summary>
    public class PointCloudData
    {
        public float[] Positions;   // 每点 3 个 float 连续排：x,y,z, x,y,z, ...
        public byte[]  Colors;      // 每点 3 个 byte 连续排：r,g,b, r,g,b, ...；无颜色时为 null
        public int     Count;       // 点数
        public bool    HasColors => Colors != null;

        public float MinX, MinY, MinZ;   // 包围盒（解析时顺带算出来，
        public float MaxX, MaxY, MaxZ;   // 后面视锥剔除、相机聚焦全要用）
    }
}
