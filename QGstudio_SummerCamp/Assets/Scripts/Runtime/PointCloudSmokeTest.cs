using System.IO;
using UnityEngine;
using PointCloud;

namespace PointCloud
{
    /// <summary>
    /// 冒烟测试：调用 PLYtoPointCloud.Load() 解析 PLY，
    /// 验证数据正确性，并把点云转成 Mesh 显示出来。
    /// 用法：挂到任意 GameObject 上，Inspector 里选文件，Play。
    /// </summary>
    public class PointCloudSmokeTest : MonoBehaviour
    {
        [Header("冒烟数据（Assets/Data/ 下）")]
        public string fileName = "smoke_ascii.ply";   // 也可切 "smoke_binary.ply" 互验两条解析路线

        void Start()
        {
            //调用：解析器读文件 → 纯数据
            string path = Path.Combine(Application.dataPath, "Data", fileName);
            var data = PLYtoPointCloud.Load(path);

            // 验证：打印关键信息，人工核对
            Debug.Log($"[冒烟] 文件: {fileName} | 点数: {data.Count} | 有颜色: {data.HasColors}");
            Debug.Log($"[冒烟] 包围盒: [{data.MinX:F3}, {data.MinY:F3}, {data.MinZ:F3}] ~ " +
                      $"[{data.MaxX:F3}, {data.MaxY:F3}, {data.MaxZ:F3}]");

            // 转换：纯 C# 的 float[] → Unity 的 Vector3[]
            //    （这一步只有 MonoBehaviour 能干：纯 C# 类不依赖 UnityEngine）
            var vertices = new Vector3[data.Count];
            var indices  = new int[data.Count];
            for (int i = 0; i < data.Count; i++)
            {
                vertices[i] = new Vector3(data.Positions[i * 3], data.Positions[i * 3 + 1], data.Positions[i * 3 + 2]);
                indices[i]  = i;
            }

            // 塞进 Mesh：Points 拓扑（点云专属——没有三角形，每个点独立渲染
            var mesh = new Mesh();
            mesh.SetVertices(vertices);
            mesh.SetIndices(indices, MeshTopology.Points, 0); // Unity自带的点mesh 但是大小不能调整

            // 颜色也一并塞进去
            if (data.HasColors)
            {
                var colors = new Color32[data.Count];
                for (int i = 0; i < data.Count; i++)
                    colors[i] = new Color32(data.Colors[i * 3 + 1], data.Colors[i * 3], data.Colors[i * 3 + 2], 255);
                mesh.SetColors(colors);
            }

            // 显示：挂上 MeshFilter + MeshRenderer
            var mf = gameObject.AddComponent<MeshFilter>();
            var mr = gameObject.AddComponent<MeshRenderer>();
            mf.sharedMesh = mesh;
            mr.sharedMaterial = new Material(Shader.Find("PointCloud/VertexColor"));

            Debug.Log("[冒烟] 完成：点云已显示。切换到 Scene 视图查看彩色球。");
        }
    }
}
