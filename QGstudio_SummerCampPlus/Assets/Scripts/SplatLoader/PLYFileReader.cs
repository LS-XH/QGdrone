// SPDX-License-Identifier: MIT

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Unity.Collections;

namespace QGStudio.SplatLoader
{
    public static class PLYFileReader
    {
        public static void ReadFileHeader(string filePath, out int vertexCount, out int vertexStride, out List<(string, ElementType)> attrs)
        {
            vertexCount = 0;
            vertexStride = 0;
            attrs = new List<(string, ElementType)>();
            if (!File.Exists(filePath))
                return;
            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read);
            ReadHeaderImpl(filePath, out vertexCount, out vertexStride, out attrs, fs);
        }

        static void ReadHeaderImpl(string filePath, out int vertexCount, out int vertexStride, out List<(string, ElementType)> attrs, FileStream fs)
        {
            // C# arrays and NativeArrays make it hard to have a "byte" array larger than 2GB :/
            if (fs.Length >= 2 * 1024 * 1024 * 1024L)
                throw new IOException($"PLY {filePath} read error: currently files larger than 2GB are not supported");

            // read header
            vertexCount = 0;
            vertexStride = 0;
            attrs = new List<(string, ElementType)>();
            const int kMaxHeaderLines = 9000;
            bool got_binary_le = false;
            for (int lineIdx = 0; lineIdx < kMaxHeaderLines; ++lineIdx)
            {
                var line = ReadLine(fs);
                if (line == "end_header" || line.Length == 0)
                    break;
                var tokens = line.Split(' ');
                if (tokens.Length == 3 && tokens[0] == "format" && tokens[1] == "binary_little_endian" && tokens[2] == "1.0")
                    got_binary_le = true;
                if (tokens.Length == 3 && tokens[0] == "element" && tokens[1] == "vertex")
                    vertexCount = int.Parse(tokens[2]);
                if (tokens.Length == 3 && tokens[0] == "property")
                {
                    ElementType type = tokens[1] switch
                    {
                        "float" => ElementType.Float,
                        "double" => ElementType.Double,
                        "uchar" => ElementType.UChar,
                        _ => ElementType.None
                    };
                    vertexStride += TypeToSize(type);
                    attrs.Add((tokens[2], type));
                }
            }

            if (!got_binary_le)
            {
                throw new IOException($"PLY {filePath} not supported: needs to be binary, little endian PLY format");
            }
        }

        public static void ReadFile(string filePath, out int vertexCount, out int vertexStride, out List<(string, ElementType)> attrs, out NativeArray<byte> vertices)
        {
            using var fs = new FileStream(filePath, FileMode.Open, FileAccess.Read);
            ReadHeaderImpl(filePath, out vertexCount, out vertexStride, out attrs, fs);

            // 完整性预检（2026-08-17 加固）：header 声明的数据量 vs 文件剩余字节。
            // 半截文件（网络传输中断）在读盘前就报错，错误信息明确，不浪费整盘 IO。
            long expectedData = (long)vertexCount * vertexStride;
            long remaining = fs.Length - fs.Position;
            if (remaining < expectedData)
                throw new IOException($"PLY 文件不完整: header 声明 {vertexCount} 点 × {vertexStride}B = {expectedData} 数据字节，文件实际剩余 {remaining} 字节（可能传输中断或文件损坏）: {filePath}");

            vertices = new NativeArray<byte>((int)expectedData, Allocator.Persistent); // 预检保证 < 2GB（ReadHeaderImpl 限制），int 安全
            // 补满循环：FileStream.Read 不保证一次读满（红榜：Read 补满循环是 PLY 必踩坑）
            var span = vertices.AsSpan();
            int total = 0;
            while (total < span.Length)
            {
                int n = fs.Read(span[total..]);
                if (n <= 0)
                    break; // EOF
                total += n;
            }
            if (total != span.Length)
                throw new IOException($"PLY {filePath} read error, expected {span.Length} data bytes got {total}");
        }

        public enum ElementType
        {
            None,
            Float,
            Double,
            UChar
        }

        public static int TypeToSize(ElementType t)
        {
            return t switch
            {
                ElementType.None => 0,
                ElementType.Float => 4,
                ElementType.Double => 8,
                ElementType.UChar => 1,
                _ => throw new ArgumentOutOfRangeException(nameof(t), t, null)
            };
        }

        static string ReadLine(FileStream fs)
        {
            var byteBuffer = new List<byte>();
            while (true)
            {
                int b = fs.ReadByte();
                if (b == -1 || b == '\n')
                    break;
                byteBuffer.Add((byte)b);
            }
            // if line had CRLF line endings, remove the CR part
            if (byteBuffer.Count > 0 && byteBuffer.Last() == '\r')
                byteBuffer.RemoveAt(byteBuffer.Count-1);
            return Encoding.UTF8.GetString(byteBuffer.ToArray());
        }
    }
}
