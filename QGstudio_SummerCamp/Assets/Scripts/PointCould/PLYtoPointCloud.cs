using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace PointCloud
{
    /// <summary>
    /// PLY 解析器（纯 C#，不依赖 UnityEngine）。
    /// 输入：PLY 文件路径；输出：PointCloudData。
    /// 支持 binary_little_endian 与 ascii 两种数据段编码。
    /// </summary>
    public static class PLYtoPointCloud
    {
        // ── PLY 官方规范定义的字段类型（官方标准，共 8 种）──
        private enum PlyType { Char, UChar, Short, UShort, Int, UInt, Float, Double }

        // 类型 → 字节数映射（按 PLY 规范：Char=1, Short=2, Int=4, Float=4, Double=8）
        private static readonly Dictionary<PlyType, int> TypeSize = new Dictionary<PlyType, int>
        {
            { PlyType.Char, 1 },   { PlyType.UChar, 1 },
            { PlyType.Short, 2 },  { PlyType.UShort, 2 },
            { PlyType.Int, 4 },    { PlyType.UInt, 4 },
            { PlyType.Float, 4 },  { PlyType.Double, 8 },
        };

        // ── header 解析结果：顶点数 + 字段布局 ──
        private class PlyHeader
        {
            public bool IsBinary;
            public bool IsLittleEndian = true;
            public int VertexCount;
            // 字段声明顺序（按 header 中 property 的出现顺序）
            public List<(string name, PlyType type)> VertexProps = new List<(string, PlyType)>();
            // 字段名 → 字节偏移（binary 数据段用）
            public Dictionary<string, int> VertexOffsets = new Dictionary<string, int>();
            // 字段名 → 声明序号（ascii 数据段用，对应每行空格分割后的下标）
            public Dictionary<string, int> FieldIndex = new Dictionary<string, int>();
            public int BytesPerVertex;   // 一个点占多少字节
        }

        // ── 入口：路径 → 数据。失败抛异常，由调用方（MonoBehaviour）处理 ──
        public static PointCloudData Load(string path)
        {
            using var fs = File.OpenRead(path);
            var header = ParseHeader(fs);
            if (header.VertexCount <= 0)
                throw new InvalidDataException("PLY 中没有 vertex 元素");

            return header.IsBinary ? ReadBinary(fs, header) : ReadAscii(fs, header);
        }

        // ── ① Header 解析（ASCII 文本行）──
        private static PlyHeader ParseHeader(FileStream fs)
        {
            var header = new PlyHeader();
            string line;
            while ((line = ReadLineExact(fs)) != null)
            {
                var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                switch (parts[0])
                {
                    case "ply":        // 魔数，第一行，可在此加校验
                        break;
                    case "format":     // "format binary_little_endian 1.0"
                        header.IsBinary       = parts[1].StartsWith("binary");
                        header.IsLittleEndian = parts[1].Contains("little");
                        break;
                    case "element":    // "element vertex 123456"
                        if (parts.Length >= 3 && parts[1] == "vertex")
                            header.VertexCount = int.Parse(parts[2]);
                        break;
                    case "property":
                        // 普通字段: "property float x"
                        // list 字段: "property list uchar int vertex_indices"（mesh 的 face 才有，点云跳过）
                        if (parts[1] == "list") break;
                        header.VertexProps.Add((parts[2], ParseType(parts[1])));
                        break;
                    case "end_header":
                        ComputeLayout(header);   // ② 算偏移表
                        return header;           // 返回时 fs.Position 精确停在数据段起点
                }
            }
            throw new InvalidDataException("未找到 end_header");
        }

        // 按字节读一行（兼容 \r\n / \n），不经任何缓冲——fs.Position 精确。
        // ⚠️ 不能用 StreamReader：它内部有 1KB 缓冲区会预读数据段，导致 Position 定位不准。
        private static string ReadLineExact(FileStream fs)
        {
            var sb = new StringBuilder();
            int b;
            while ((b = fs.ReadByte()) != -1)
            {
                if (b == '\n') break;
                if (b != '\r') sb.Append((char)b);
            }
            return sb.Length > 0 || b != -1 ? sb.ToString() : null;
        }

        private static PlyType ParseType(string s) =>
            (PlyType)Enum.Parse(typeof(PlyType), s, ignoreCase: true);

        // ── ② 偏移表：header 里 property 的声明顺序 = 数据段里每个点的内存排列顺序 ──
        //    例：x(4) y(4) z(4) r(1) g(1) b(1) → x@0, y@4, z@8, r@12, 每点 15 字节
        private static void ComputeLayout(PlyHeader h)
        {
            int offset = 0;
            for (int i = 0; i < h.VertexProps.Count; i++)
            {
                var (name, type) = h.VertexProps[i];
                h.VertexOffsets[name] = offset;
                h.FieldIndex[name] = i;
                offset += TypeSize[type];
            }
            h.BytesPerVertex = offset;
        }

        // ── ③ binary 数据段：整块读入内存，按偏移表逐点抠字段 ──
        private static PointCloudData ReadBinary(FileStream fs, PlyHeader h)
        {
            int count = h.VertexCount;
            var data = new PointCloudData { Count = count };
            var pos = new float[count * 3];
            byte[] col = null;
            bool hasColor = h.VertexProps.Exists(p => p.name == "red");   // 读到就用，读不到不崩
            if (hasColor) col = new byte[count * 3];

            int offX = h.VertexOffsets["x"], offY = h.VertexOffsets["y"], offZ = h.VertexOffsets["z"];
            int offR = hasColor ? h.VertexOffsets["red"] : -1;
            int offG = hasColor ? h.VertexOffsets["green"] : -1;
            int offB = hasColor ? h.VertexOffsets["blue"] : -1;

            int bytesPer = h.BytesPerVertex;
            long total = (long)count * bytesPer;   // 千万点 × 15B ≈ 150MB，必须用 long 防 int 溢出
            var buffer = new byte[total];
            int read = 0;
            while (read < total)
            {
                int n = fs.Read(buffer, read, (int)(total - read));   // Read 可能读不满，循环补齐
                if (n <= 0) throw new InvalidDataException("文件提前结束");
                read += n;
            }

            for (int i = 0; i < count; i++)
            {
                int baseIdx = i * bytesPer;
                pos[i * 3 + 0] = BitConverter.ToSingle(buffer, baseIdx + offX);
                pos[i * 3 + 1] = BitConverter.ToSingle(buffer, baseIdx + offY);
                pos[i * 3 + 2] = BitConverter.ToSingle(buffer, baseIdx + offZ);
                if (hasColor)
                {
                    col[i * 3 + 0] = buffer[baseIdx + offR];
                    col[i * 3 + 1] = buffer[baseIdx + offG];
                    col[i * 3 + 2] = buffer[baseIdx + offB];
                }
            }

            data.Positions = pos;
            data.Colors = col;
            ComputeBounds(data);
            return data;
        }

        // ── ④ ascii 数据段：每行一个点，字段按 header 顺序空格分隔 ──
        private static PointCloudData ReadAscii(FileStream fs, PlyHeader h)
        {
            int count = h.VertexCount;
            var data = new PointCloudData { Count = count };
            var pos = new float[count * 3];
            byte[] col = null;
            bool hasColor = h.VertexProps.Exists(p => p.name == "red");
            if (hasColor) col = new byte[count * 3];

            int idxX = h.FieldIndex["x"], idxY = h.FieldIndex["y"], idxZ = h.FieldIndex["z"];
            int idxR = hasColor ? h.FieldIndex["red"] : -1;   // red/green/blue 在 header 中连续声明

            using var reader = new StreamReader(fs, Encoding.ASCII);
            for (int i = 0; i < count; i++)
            {
                string line = reader.ReadLine();
                if (line == null) throw new InvalidDataException($"第 {i} 行提前结束");
                var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                // InvariantCulture：防止系统区域设置把小数点当逗号（法国等地区）
                pos[i * 3 + 0] = float.Parse(parts[idxX], CultureInfo.InvariantCulture);
                pos[i * 3 + 1] = float.Parse(parts[idxY], CultureInfo.InvariantCulture);
                pos[i * 3 + 2] = float.Parse(parts[idxZ], CultureInfo.InvariantCulture);
                if (hasColor)
                {
                    col[i * 3 + 0] = byte.Parse(parts[idxR], CultureInfo.InvariantCulture);
                    col[i * 3 + 1] = byte.Parse(parts[idxR + 1], CultureInfo.InvariantCulture);
                    col[i * 3 + 2] = byte.Parse(parts[idxR + 2], CultureInfo.InvariantCulture);
                }
            }

            data.Positions = pos;
            data.Colors = col;
            ComputeBounds(data);
            return data;
        }

        // ── ⑤ 包围盒（解析时顺带算，后面剔除/聚焦相机都要用）──
        private static void ComputeBounds(PointCloudData d)
        {
            float minX = float.MaxValue, minY = float.MaxValue, minZ = float.MaxValue;
            float maxX = float.MinValue, maxY = float.MinValue, maxZ = float.MinValue;
            for (int i = 0; i < d.Count; i++)
            {
                float x = d.Positions[i * 3], y = d.Positions[i * 3 + 1], z = d.Positions[i * 3 + 2];
                if (x < minX) minX = x; if (x > maxX) maxX = x;
                if (y < minY) minY = y; if (y > maxY) maxY = y;
                if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
            }
            d.MinX = minX; d.MaxX = maxX;
            d.MinY = minY; d.MaxY = maxY;
            d.MinZ = minZ; d.MaxZ = maxZ;
        }
    }
}
