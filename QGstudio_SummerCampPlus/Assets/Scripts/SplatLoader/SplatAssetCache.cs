// SPDX-License-Identifier: MIT
// ============================================================================
// SplatAssetCache.cs —— 缓存读写（简报 §五 定稿 + 2026-08-17 加固）
// 校验三层（加固后）：
//   ① 大小比对：meta 记录的 5 个 bin 字节数 vs 实际文件大小（stat，零成本）
//   ② 内容校验：FNV-1a 64（读 bin 后对内存字节计算，IO 零增加 ~100ms 后台）
//      —— 覆盖"大小没变但内容损坏"的盲区
//   ③ 元数据合法性：formatVersion / sceneId / 4 formats 枚举 / splatCount
// 损坏处理：校验失败 → 删整个 sceneId 目录 → 返回 Corrupted（调用方走未命中
//           路径重转；重转结果直接挂载、不再回读校验 = 防死循环，红榜 #12）
// 纯静态无状态：仅 File IO + 托管 byte[]（无 Job 无 Temp）→ 可整段 Task.Run 后台。
// 不引用 Application API（root 由调用方主线程传入）→ 后台线程安全。
// 落盘顺序：bin 先写、meta.json 最后写（meta 存在 = 缓存完整；
//           写一半崩溃 → 下次 Miss/Corrupted → 重转兜底）
// ============================================================================
using System;
using System.IO;
using GaussianSplatting.Runtime;
using UnityEngine;

namespace QGStudio.SplatLoader
{
    public static class SplatAssetCache
    {
        public enum LoadStatus { Miss, Hit, Corrupted }

        public struct CacheLoadResult
        {
            public LoadStatus Status;
            public GaussianSplatPipeline.ConvertResult Result; // Hit 时可用；cameras 恒 null（由 Service 现场补）
        }

        // 5 个 bin 文件名（与简报 §五 一致）
        const string kChunkFile = "data_chunk.bin";
        const string kPosFile = "data_pos.bin";
        const string kOtherFile = "data_other.bin";
        const string kColorFile = "data_color.bin";
        const string kSHFile = "data_sh.bin";
        const string kMetaFile = "meta.json";

        public static string GetCacheDir(string root, string sceneId) => Path.Combine(root, sceneId);

        // ===== 落盘（未命中路径：转换完成后调用；后台线程安全）=====
        public static void Save(string root, string sceneId, string sourcePath, GaussianSplatPipeline.ConvertResult result)
        {
            string dir = GetCacheDir(root, sceneId);
            Directory.CreateDirectory(dir);

            var record = new SplatAssetRecord
            {
                formatVersion = GaussianSplatAsset.kCurrentVersion,
                sceneId = sceneId,
                sourcePath = sourcePath,
                splatCount = result.splatCount,
                formatPos = result.formatPos,
                formatScale = result.formatScale,
                formatColor = result.formatColor,
                formatSH = result.formatSH,
                boundsMinX = result.boundsMin.x, boundsMinY = result.boundsMin.y, boundsMinZ = result.boundsMin.z,
                boundsMaxX = result.boundsMax.x, boundsMaxY = result.boundsMax.y, boundsMaxZ = result.boundsMax.z,
                dataHash = result.dataHash.ToString(),
                timestamp = DateTime.Now.ToString("o"),
            };

            // bin 先写；meta.json 最后写（见文件头注释）
            if (result.chunkData != null)
            {
                File.WriteAllBytes(Path.Combine(dir, kChunkFile), result.chunkData);
                record.hasChunk = true;
                record.fileSizeChunk = result.chunkData.Length;
                record.hashChunk = Fnv1a64(result.chunkData);
            }
            File.WriteAllBytes(Path.Combine(dir, kPosFile), result.posData);
            record.fileSizePos = result.posData.Length;
            record.hashPos = Fnv1a64(result.posData);
            File.WriteAllBytes(Path.Combine(dir, kOtherFile), result.otherData);
            record.fileSizeOther = result.otherData.Length;
            record.hashOther = Fnv1a64(result.otherData);
            File.WriteAllBytes(Path.Combine(dir, kColorFile), result.colorData);
            record.fileSizeColor = result.colorData.Length;
            record.hashColor = Fnv1a64(result.colorData);
            File.WriteAllBytes(Path.Combine(dir, kSHFile), result.shData);
            record.fileSizeSH = result.shData.Length;
            record.hashSH = Fnv1a64(result.shData);

            File.WriteAllText(Path.Combine(dir, kMetaFile), JsonUtility.ToJson(record, prettyPrint: true));
        }

        // ===== 直读（命中路径；后台线程安全）=====
        // Miss：无 meta（未缓存）；Corrupted：校验失败（已清理目录，调用方走重转）；Hit：Result 可用
        public static CacheLoadResult Load(string root, string sceneId)
        {
            string dir = GetCacheDir(root, sceneId);
            string metaPath = Path.Combine(dir, kMetaFile);
            if (!File.Exists(metaPath))
                return new CacheLoadResult { Status = LoadStatus.Miss };

            try
            {
                var record = JsonUtility.FromJson<SplatAssetRecord>(File.ReadAllText(metaPath));
                if (record == null || record.formatVersion != GaussianSplatAsset.kCurrentVersion || record.sceneId != sceneId)
                    return CorruptedAndClean(dir); // 版本不兼容 / 目录串了 → 清理重转

                // ③ 元数据合法性：4 formats 枚举 + 点数（防 meta 被改成非法值后 Initialize 崩）
                if (record.splatCount <= 0 ||
                    !Enum.IsDefined(typeof(GaussianSplatAsset.VectorFormat), record.formatPos) ||
                    !Enum.IsDefined(typeof(GaussianSplatAsset.VectorFormat), record.formatScale) ||
                    !Enum.IsDefined(typeof(GaussianSplatAsset.ColorFormat), record.formatColor) ||
                    !Enum.IsDefined(typeof(GaussianSplatAsset.SHFormat), record.formatSH))
                    return CorruptedAndClean(dir);

                // ① 大小校验：meta 记录的文件大小 vs 实际（先 stat 比对，全过再读，损坏时不白读 270MB）
                if (record.hasChunk && !FileSizeMatches(dir, kChunkFile, record.fileSizeChunk)) return CorruptedAndClean(dir);
                if (!FileSizeMatches(dir, kPosFile, record.fileSizePos)) return CorruptedAndClean(dir);
                if (!FileSizeMatches(dir, kOtherFile, record.fileSizeOther)) return CorruptedAndClean(dir);
                if (!FileSizeMatches(dir, kColorFile, record.fileSizeColor)) return CorruptedAndClean(dir);
                if (!FileSizeMatches(dir, kSHFile, record.fileSizeSH)) return CorruptedAndClean(dir);

                // 全部通过 → 读 bin（byte[] 直接进 ConvertResult，零拷贝）
                // dataHash 字符串解析（Unity 6000 Hash128 只有 Parse 无 TryParse；非法 → 抛异常 → 外层 catch → Corrupted）
                var dataHash = Hash128.Parse(record.dataHash);

                byte[] chunkData = record.hasChunk ? File.ReadAllBytes(Path.Combine(dir, kChunkFile)) : null;
                byte[] posData = File.ReadAllBytes(Path.Combine(dir, kPosFile));
                byte[] otherData = File.ReadAllBytes(Path.Combine(dir, kOtherFile));
                byte[] colorData = File.ReadAllBytes(Path.Combine(dir, kColorFile));
                byte[] shData = File.ReadAllBytes(Path.Combine(dir, kSHFile));

                // ② 内容校验：FNV-1a 64（覆盖"大小没变但内容损坏"；字节已在内存，IO 零增加）
                if (record.hasChunk && Fnv1a64(chunkData) != record.hashChunk) return CorruptedAndClean(dir);
                if (Fnv1a64(posData) != record.hashPos) return CorruptedAndClean(dir);
                if (Fnv1a64(otherData) != record.hashOther) return CorruptedAndClean(dir);
                if (Fnv1a64(colorData) != record.hashColor) return CorruptedAndClean(dir);
                if (Fnv1a64(shData) != record.hashSH) return CorruptedAndClean(dir);

                var result = new GaussianSplatPipeline.ConvertResult
                {
                    splatCount = record.splatCount,
                    formatPos = record.formatPos,
                    formatScale = record.formatScale,
                    formatColor = record.formatColor,
                    formatSH = record.formatSH,
                    boundsMin = new Vector3(record.boundsMinX, record.boundsMinY, record.boundsMinZ),
                    boundsMax = new Vector3(record.boundsMaxX, record.boundsMaxY, record.boundsMaxZ),
                    dataHash = dataHash,
                    cameras = null, // 决策点①：相机不进缓存，由 Service 现场重读 cameras.json
                    chunkData = chunkData,
                    posData = posData,
                    otherData = otherData,
                    colorData = colorData,
                    shData = shData,
                };
                return new CacheLoadResult { Status = LoadStatus.Hit, Result = result };
            }
            catch (Exception)
            {
                // meta 解析失败 / IO 异常 / Hash128 字符串非法 → 一律当损坏清理（重转兜底）
                return CorruptedAndClean(dir);
            }
        }

        static bool FileSizeMatches(string dir, string file, long expected)
        {
            if (expected < 0) return false;
            var fi = new FileInfo(Path.Combine(dir, file));
            return fi.Exists && fi.Length == expected;
        }

        // FNV-1a 64：轻量内容校验（非密码学，防意外损坏足够；270MB ~100ms）
        static ulong Fnv1a64(byte[] data)
        {
            ulong hash = 14695981039346656037UL;
            foreach (byte b in data)
            {
                hash ^= b;
                hash *= 1099511628211UL;
            }
            return hash;
        }

        static CacheLoadResult CorruptedAndClean(string dir)
        {
            Debug.LogWarning($"[SplatCache] 缓存损坏/不完整/版本不兼容，已清理目录并走重新转换: {dir}");
            try
            {
                if (Directory.Exists(dir))
                    Directory.Delete(dir, true); // 删不掉不阻塞：后续 Save 覆盖写
            }
            catch (Exception)
            {
            }
            return new CacheLoadResult { Status = LoadStatus.Corrupted };
        }
    }
}
