// SPDX-License-Identifier: MIT
// ============================================================================
// GaussianSplatPipeline.cs —— 运行时 PLY→byte[] 转换管线（阶段 1）
// 提取自包 Editor/GaussianSplatAssetCreator.cs（1133 行），对照源行号已注释。
// 提取规则（用户最高优先级要求）：核心计算逐行保留，不做任何顺手优化/重写。
// 仅 4 类工程改动：
//   ① 删 Editor 专属：EditorWindow 壳 / AssetDatabase 落盘 / DisplayProgressBar / BC7 分支
//   ② 改签名（5 处）：Create*Data 的 string filePath → 返回 byte[]（FileStream → ToArray）
//   ③ 进度：DisplayProgressBar → Action<float> 回调
//   ④ 命名空间：GaussianSplatting.Editor.Utils → QGStudio.SplatLoader（防与包 Editor 程序集 CS0433 冲突）
// 验证铁证：同 PLY 同质量档，本管线输出 vs Creator 产物逐字节一致。
// ============================================================================
using System;
using System.Collections.Generic;
using System.IO;
using GaussianSplatting.Runtime;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace QGStudio.SplatLoader
{
    public class GaussianSplatPipeline
    {
        const string kCamerasJson = "cameras.json"; // Creator:23

        // Creator:27-35（照搬）
        public enum DataQuality
        {
            VeryHigh,
            High,
            Medium,
            Low,
            VeryLow,
            Custom,
        }

        // 完整转换结果（5 通道 byte[] + 元数据；byte[] 即 Creator 落盘的 .bytes 文件内容）
        public struct ConvertResult
        {
            public int splatCount;
            public GaussianSplatAsset.VectorFormat formatPos;
            public GaussianSplatAsset.VectorFormat formatScale;
            public GaussianSplatAsset.ColorFormat formatColor;
            public GaussianSplatAsset.SHFormat formatSH;
            public Vector3 boundsMin;
            public Vector3 boundsMax;
            public Hash128 dataHash;
            public GaussianSplatAsset.CameraInfo[] cameras;
            public byte[] chunkData;
            public byte[] posData;
            public byte[] otherData;
            public byte[] colorData;
            public byte[] shData;
            public string errorMessage;
        }

        // —— 格式状态（Creator:44-47 的 m_Format*，由 ApplyQualityLevel 驱动）——
        GaussianSplatAsset.VectorFormat m_FormatPos;
        GaussianSplatAsset.VectorFormat m_FormatScale;
        GaussianSplatAsset.ColorFormat m_FormatColor;
        GaussianSplatAsset.SHFormat m_FormatSH;

        // 相机 json 是否导入（Creator:40 m_ImportCameras）
        public bool importCameras = true;

        // 当前质量档对应的 4 个格式（ApplyQualityLevel 设置；步骤式编排时 LoadingManager 需要读取）
        public GaussianSplatAsset.VectorFormat FormatPos => m_FormatPos;
        public GaussianSplatAsset.VectorFormat FormatScale => m_FormatScale;
        public GaussianSplatAsset.ColorFormat FormatColor => m_FormatColor;
        public GaussianSplatAsset.SHFormat FormatSH => m_FormatSH;

        // 进度回调（Creator 的 EditorUtility.DisplayProgressBar 换成它；数值语义照搬 Creator）
        Action<float> m_Progress;
        public void SetProgressCallback(Action<float> progress) => m_Progress = progress;

        // 最近一次错误（LoadInputFile 失败时设置；ConvertAll 同源写入 ConvertResult.errorMessage）
        public string LastErrorMessage { get; private set; }

        // Creator:54-58（照搬）
        public bool isUsingChunks =>
            m_FormatPos != GaussianSplatAsset.VectorFormat.Float32 ||
            m_FormatScale != GaussianSplatAsset.VectorFormat.Float32 ||
            m_FormatColor != GaussianSplatAsset.ColorFormat.Float32x4 ||
            m_FormatSH != GaussianSplatAsset.SHFormat.Float32;

        // Creator:189-228 ApplyQualityLevel（去 EditorPrefs，纯格式映射）
        public void ApplyQualityLevel(DataQuality quality)
        {
            switch (quality)
            {
                case DataQuality.Custom:
                    break;
                case DataQuality.VeryLow: // 18.62x smaller, 32.27 PSNR —— 运行时不支持（BC7 需 EditorUtility.CompressTexture）
                    throw new NotSupportedException("VeryLow(BC7) 质量档运行时不可用：BC7 压缩需要 Editor 的 CompressTexture");
                case DataQuality.Low: // 14.01x smaller, 35.17 PSNR
                    m_FormatPos = GaussianSplatAsset.VectorFormat.Norm11;
                    m_FormatScale = GaussianSplatAsset.VectorFormat.Norm6;
                    m_FormatColor = GaussianSplatAsset.ColorFormat.Norm8x4;
                    m_FormatSH = GaussianSplatAsset.SHFormat.Cluster16k;
                    break;
                case DataQuality.Medium: // 5.14x smaller, 47.46 PSNR
                    m_FormatPos = GaussianSplatAsset.VectorFormat.Norm11;
                    m_FormatScale = GaussianSplatAsset.VectorFormat.Norm11;
                    m_FormatColor = GaussianSplatAsset.ColorFormat.Norm8x4;
                    m_FormatSH = GaussianSplatAsset.SHFormat.Norm6;
                    break;
                case DataQuality.High: // 2.94x smaller, 57.77 PSNR
                    m_FormatPos = GaussianSplatAsset.VectorFormat.Norm16;
                    m_FormatScale = GaussianSplatAsset.VectorFormat.Norm16;
                    m_FormatColor = GaussianSplatAsset.ColorFormat.Float16x4;
                    m_FormatSH = GaussianSplatAsset.SHFormat.Norm11;
                    break;
                case DataQuality.VeryHigh: // 1.05x smaller
                    m_FormatPos = GaussianSplatAsset.VectorFormat.Float32;
                    m_FormatScale = GaussianSplatAsset.VectorFormat.Float32;
                    m_FormatColor = GaussianSplatAsset.ColorFormat.Float32x4;
                    m_FormatSH = GaussianSplatAsset.SHFormat.Float32;
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        // ================================================================
        // 步骤接口（LoadingManager 用 async/await 编排，每步之间让出主线程刷新 UI）
        // 每个方法内部与 Creator 对应代码逐行对齐
        // ================================================================

        // Creator:263-270 读数据 + 342-359 LoadInputSplatFile（去 ClearProgressBar）
        // ⚠️ 内部含 LinearizeDataJob（Job.Schedule），必须在主线程调用
        public NativeArray<InputSplatData> LoadInputFile(string filePath)
        {
            NativeArray<InputSplatData> data = default;
            if (!File.Exists(filePath))
            {
                LastErrorMessage = $"Did not find {filePath} file";
                Debug.LogError($"GS: {LastErrorMessage}");
                return data;
            }
            try
            {
                m_Progress?.Invoke(0.0f); // Creator:263 "Reading data files"
                GaussianFileReader.ReadFile(filePath, out data);
            }
            catch (Exception ex)
            {
                LastErrorMessage = ex.Message;
                Debug.LogError($"GS: {LastErrorMessage}");
            }
            return data;
        }

        // —— 异步解析分段（方案 B）：Step1 后台线程（IO+纯函数），Step2 主线程（LinearizeDataJob）——
        // 与 LoadInputFile 数据流等价：Step1 + Step2 合起来 = GaussianFileReader.ReadFile
        public NativeArray<InputSplatData> LoadInputFileStep1(string filePath)
        {
            NativeArray<InputSplatData> data = default;
            if (!File.Exists(filePath))
            {
                LastErrorMessage = $"Did not find {filePath} file";
                Debug.LogError($"GS: {LastErrorMessage}");
                return data;
            }
            try
            {
                GaussianFileReader.ReadFileStep1(filePath, out data);
            }
            catch (Exception ex)
            {
                LastErrorMessage = ex.Message;
                Debug.LogError($"GS: {LastErrorMessage}");
            }
            return data;
        }

        public void LoadInputFileStep2(NativeArray<InputSplatData> splats)
        {
            if (!splats.IsCreated || splats.Length == 0)
                return;
            GaussianFileReader.ReadFileStep2(splats);
        }

        // Creator:272-279 CalcBoundsJob 调度（Job 字段是 float3*，变量必须 float3；取地址需 unsafe，同 Creator 的 unsafe void CreateAsset）
        public unsafe (float3 min, float3 max) CalcBounds(NativeArray<InputSplatData> splats)
        {
            float3 boundsMin = default, boundsMax = default;
            var boundsJob = new CalcBoundsJob
            {
                m_BoundsMin = &boundsMin,
                m_BoundsMax = &boundsMax,
                m_SplatData = splats
            };
            boundsJob.Schedule().Complete();
            return (boundsMin, boundsMax);
        }

        // Creator:281-282 + 411-429 ReorderMorton
        public void ReorderMorton(NativeArray<InputSplatData> splats, float3 boundsMin, float3 boundsMax)
        {
            m_Progress?.Invoke(0.05f); // Creator:281 "Morton reordering"
            ReorderMortonInternal(splats, boundsMin, boundsMax);
        }

        // Creator:285-291 + 476-518 ClusterSHs（仅 m_FormatSH >= Cluster64k 时调用；否则返回未创建数组）
        public void ClusterSHs(NativeArray<InputSplatData> splats, out NativeArray<GaussianSplatAsset.SHTableItemFloat16> clusteredSHs, out NativeArray<int> splatSHIndices)
        {
            clusteredSHs = default;
            splatSHIndices = default;
            if (m_FormatSH >= GaussianSplatAsset.SHFormat.Cluster64k)
            {
                m_Progress?.Invoke(0.2f); // Creator:289 "Cluster SHs"
                s_ProgressForwarder = m_Progress; // 静态转发（ClusterSHsInternal 贴源码为 static）
                ClusterSHsInternal(splats, m_FormatSH, out clusteredSHs, out splatSHIndices);
                s_ProgressForwarder = null;
            }
        }

        // Creator:641-658 CreateChunkData（改签名：string filePath → byte[] 返回）
        public byte[] CreateChunkData(NativeArray<InputSplatData> splatData, ref Hash128 dataHash)
        {
            int chunkCount = (splatData.Length + GaussianSplatAsset.kChunkSize - 1) / GaussianSplatAsset.kChunkSize;
            CalcChunkDataJob job = new CalcChunkDataJob
            {
                splatData = splatData,
                chunks = new(chunkCount, Allocator.TempJob),
            };

            job.Schedule(chunkCount, 8).Complete();

            dataHash.Append(ref job.chunks);

            byte[] result = job.chunks.Reinterpret<byte>(UnsafeUtility.SizeOf<GaussianSplatAsset.ChunkInfo>()).ToArray(); // Creator:654-655 FileStream → ToArray

            job.chunks.Dispose();
            return result;
        }

        // Creator:812-833 CreatePositionsData（改签名）
        public byte[] CreatePositionsData(NativeArray<InputSplatData> inputSplats, ref Hash128 dataHash)
        {
            int dataLen = inputSplats.Length * GaussianSplatAsset.GetVectorSize(m_FormatPos);
            dataLen = NextMultipleOf(dataLen, 8); // serialized as ulong
            NativeArray<byte> data = new(dataLen, Allocator.TempJob);

            CreatePositionsDataJob job = new CreatePositionsDataJob
            {
                m_Input = inputSplats,
                m_Format = m_FormatPos,
                m_FormatSize = GaussianSplatAsset.GetVectorSize(m_FormatPos),
                m_Output = data
            };
            job.Schedule(inputSplats.Length, 8192).Complete();

            dataHash.Append(data);

            byte[] result = data.ToArray(); // Creator:829-830 FileStream → ToArray

            data.Dispose();
            return result;
        }

        // Creator:835-861 CreateOtherData（改签名）
        public byte[] CreateOtherData(NativeArray<InputSplatData> inputSplats, ref Hash128 dataHash, NativeArray<int> splatSHIndices)
        {
            int formatSize = GaussianSplatAsset.GetOtherSizeNoSHIndex(m_FormatScale);
            if (splatSHIndices.IsCreated)
                formatSize += 2;
            int dataLen = inputSplats.Length * formatSize;

            dataLen = NextMultipleOf(dataLen, 8); // serialized as ulong
            NativeArray<byte> data = new(dataLen, Allocator.TempJob);

            CreateOtherDataJob job = new CreateOtherDataJob
            {
                m_Input = inputSplats,
                m_SplatSHIndices = splatSHIndices,
                m_ScaleFormat = m_FormatScale,
                m_FormatSize = formatSize,
                m_Output = data
            };
            job.Schedule(inputSplats.Length, 8192).Complete();

            dataHash.Append(data);

            byte[] result = data.ToArray(); // Creator:857-858 FileStream → ToArray

            data.Dispose();
            return result;
        }

        // Creator:887-932 CreateColorData（改签名 + 删 BC7 压缩分支）
        public byte[] CreateColorData(NativeArray<InputSplatData> inputSplats, ref Hash128 dataHash)
        {
            var (width, height) = GaussianSplatAsset.CalcTextureSize(inputSplats.Length);
            NativeArray<float4> data = new(width * height, Allocator.TempJob);

            CreateColorDataJob job = new CreateColorDataJob();
            job.m_Input = inputSplats;
            job.m_Output = data;
            job.Schedule(inputSplats.Length, 8192).Complete();

            dataHash.Append(data);
            dataHash.Append((int)m_FormatColor);

            GraphicsFormat gfxFormat = GaussianSplatAsset.ColorFormatToGraphics(m_FormatColor);
            int dstSize = (int)GraphicsFormatUtility.ComputeMipmapSize(width, height, gfxFormat);

            if (GraphicsFormatUtility.IsCompressedFormat(gfxFormat))
            {
                // Creator:903-913 BC7 分支 —— 运行时无 EditorUtility.CompressTexture，防御性报错
                data.Dispose();
                throw new NotSupportedException($"GS: compressed color format {m_FormatColor} is not supported at runtime (VeryLow/BC7)");
            }

            ConvertColorJob jobConvert = new ConvertColorJob
            {
                width = width,
                height = height,
                inputData = data,
                format = m_FormatColor,
                outputData = new NativeArray<byte>(dstSize, Allocator.TempJob),
                formatBytesPerPixel = dstSize / width / height
            };
            jobConvert.Schedule(height, 1).Complete();
            byte[] result = jobConvert.outputData.ToArray(); // Creator:926-927 FileStream → ToArray
            jobConvert.outputData.Dispose();

            data.Dispose();
            return result;
        }

        // Creator:1039-1044 EmitSimpleDataFile + 1046-1066 CreateSHData（改签名）
        public byte[] CreateSHData(NativeArray<InputSplatData> inputSplats, ref Hash128 dataHash, NativeArray<GaussianSplatAsset.SHTableItemFloat16> clusteredSHs)
        {
            if (clusteredSHs.IsCreated)
            {
                return EmitSimpleDataFile(clusteredSHs, ref dataHash);
            }
            else
            {
                int dataLen = (int)GaussianSplatAsset.CalcSHDataSize(inputSplats.Length, m_FormatSH);
                NativeArray<byte> data = new(dataLen, Allocator.TempJob);
                CreateSHDataJob job = new CreateSHDataJob
                {
                    m_Input = inputSplats,
                    m_Format = m_FormatSH,
                    m_Output = data
                };
                job.Schedule(inputSplats.Length, 8192).Complete();
                byte[] result = EmitSimpleDataFile(data, ref dataHash);
                data.Dispose();
                return result;
            }
        }

        // Creator:1068-1118 LoadJsonCamerasFile（照搬，入口换成文件路径）
        public GaussianSplatAsset.CameraInfo[] LoadCameras(string curPath)
        {
            if (!importCameras)
                return null;

            string camerasPath;
            while (true)
            {
                var dir = Path.GetDirectoryName(curPath);
                if (!Directory.Exists(dir))
                    return null;
                camerasPath = $"{dir}/{kCamerasJson}";
                if (File.Exists(camerasPath))
                    break;
                curPath = dir;
            }

            if (!File.Exists(camerasPath))
                return null;

            string json = File.ReadAllText(camerasPath);
            var jsonCameras = JSONParser.FromJson<List<JsonCamera>>(json);
            if (jsonCameras == null || jsonCameras.Count == 0)
                return null;

            var result = new GaussianSplatAsset.CameraInfo[jsonCameras.Count];
            for (var camIndex = 0; camIndex < jsonCameras.Count; camIndex++)
            {
                var jsonCam = jsonCameras[camIndex];
                var pos = new Vector3(jsonCam.position[0], jsonCam.position[1], jsonCam.position[2]);
                // the matrix is a "view matrix", not "camera matrix" lol
                var axisx = new Vector3(jsonCam.rotation[0][0], jsonCam.rotation[1][0], jsonCam.rotation[2][0]);
                var axisy = new Vector3(jsonCam.rotation[0][1], jsonCam.rotation[1][1], jsonCam.rotation[2][1]);
                var axisz = new Vector3(jsonCam.rotation[0][2], jsonCam.rotation[1][2], jsonCam.rotation[2][2]);

                axisy *= -1;
                axisz *= -1;

                var cam = new GaussianSplatAsset.CameraInfo
                {
                    pos = pos,
                    axisX = axisx,
                    axisY = axisy,
                    axisZ = axisz,
                    fov = 25 //@TODO
                };
                result[camIndex] = cam;
            }

            return result;
        }

        // ================================================================
        // 同步完整转换（贴 CreateAsset 247-340 的编排；供验证脚本/一次性调用）
        // ⚠️ 全程 Job.Schedule 主线程；进度回调覆盖 0-0.99（Creator 数值语义）
        // ================================================================
        public ConvertResult ConvertAll(string inputFile, DataQuality quality)
        {
            ConvertResult result = default;
            try
            {
                ApplyQualityLevel(quality);
                result.formatPos = m_FormatPos;
                result.formatScale = m_FormatScale;
                result.formatColor = m_FormatColor;
                result.formatSH = m_FormatSH;
                result.cameras = LoadCameras(inputFile); // Creator:264

                NativeArray<InputSplatData> inputSplats = LoadInputFile(inputFile); // Creator:265
                if (inputSplats.Length == 0) // Creator:266-270
                {
                    result.errorMessage = "PLY 文件解析失败或点数为 0";
                    return result;
                }

                result.splatCount = inputSplats.Length;

                (float3 boundsMin, float3 boundsMax) = CalcBounds(inputSplats); // Creator:272-279
                result.boundsMin = (Vector3)boundsMin;
                result.boundsMax = (Vector3)boundsMax;

                ReorderMorton(inputSplats, boundsMin, boundsMax); // Creator:281-282

                NativeArray<int> splatSHIndices = default;
                NativeArray<GaussianSplatAsset.SHTableItemFloat16> clusteredSHs = default;
                ClusterSHs(inputSplats, out clusteredSHs, out splatSHIndices); // Creator:285-291

                // Creator:300 用 asset.formatVersion；Initialize 里赋 kCurrentVersion，数值等价
                var dataHash = new Hash128((uint)inputSplats.Length, (uint)GaussianSplatAsset.kCurrentVersion, 0, 0);
                bool useChunks = isUsingChunks;
                if (useChunks)
                    result.chunkData = CreateChunkData(inputSplats, ref dataHash); // Creator:310
                result.posData = CreatePositionsData(inputSplats, ref dataHash);   // Creator:311
                result.otherData = CreateOtherData(inputSplats, ref dataHash, splatSHIndices); // Creator:312
                result.colorData = CreateColorData(inputSplats, ref dataHash);     // Creator:313
                result.shData = CreateSHData(inputSplats, ref dataHash, clusteredSHs); // Creator:314
                result.dataHash = dataHash;

                splatSHIndices.Dispose(); // Creator:317-318
                clusteredSHs.Dispose();
                inputSplats.Dispose();

                m_Progress?.Invoke(0.99f);
            }
            catch (Exception ex)
            {
                result.errorMessage = ex.Message;
                Debug.LogError($"GS: ConvertAll failed: {ex}");
            }
            return result;
        }

        // ================================================================
        // 内部：Job 与辅助函数（以下全部照搬 Creator，行号已标注）
        // ================================================================

        // Creator:361-382
        [BurstCompile]
        struct CalcBoundsJob : IJob
        {
            [NativeDisableUnsafePtrRestriction] public unsafe float3* m_BoundsMin;
            [NativeDisableUnsafePtrRestriction] public unsafe float3* m_BoundsMax;
            [ReadOnly] public NativeArray<InputSplatData> m_SplatData;

            public unsafe void Execute()
            {
                float3 boundsMin = float.PositiveInfinity;
                float3 boundsMax = float.NegativeInfinity;

                for (int i = 0; i < m_SplatData.Length; ++i)
                {
                    float3 pos = m_SplatData[i].pos;
                    boundsMin = math.min(boundsMin, pos);
                    boundsMax = math.max(boundsMax, pos);
                }
                *m_BoundsMin = boundsMin;
                *m_BoundsMax = boundsMax;
            }
        }

        // Creator:384-400
        [BurstCompile]
        struct ReorderMortonJob : IJobParallelFor
        {
            const float kScaler = (float)((1 << 21) - 1);
            public float3 m_BoundsMin;
            public float3 m_InvBoundsSize;
            [ReadOnly] public NativeArray<InputSplatData> m_SplatData;
            public NativeArray<(ulong, int)> m_Order;

            public void Execute(int index)
            {
                float3 pos = ((float3)m_SplatData[index].pos - m_BoundsMin) * m_InvBoundsSize * kScaler;
                uint3 ipos = (uint3)pos;
                ulong code = GaussianUtils.MortonEncode3(ipos);
                m_Order[index] = (code, index);
            }
        }

        // Creator:402-409
        struct OrderComparer : IComparer<(ulong, int)>
        {
            public int Compare((ulong, int) a, (ulong, int) b)
            {
                if (a.Item1 < b.Item1) return -1;
                if (a.Item1 > b.Item1) return +1;
                return a.Item2 - b.Item2;
            }
        }

        // Creator:411-429（改名 ReorderMortonInternal：避免与 public 步骤方法重载混淆）
        static void ReorderMortonInternal(NativeArray<InputSplatData> splatData, float3 boundsMin, float3 boundsMax)
        {
            ReorderMortonJob order = new ReorderMortonJob
            {
                m_SplatData = splatData,
                m_BoundsMin = boundsMin,
                m_InvBoundsSize = 1.0f / (boundsMax - boundsMin),
                m_Order = new NativeArray<(ulong, int)>(splatData.Length, Allocator.TempJob)
            };
            order.Schedule(splatData.Length, 4096).Complete();
            order.m_Order.Sort(new OrderComparer());

            NativeArray<InputSplatData> copy = new(order.m_SplatData, Allocator.TempJob);
            for (int i = 0; i < copy.Length; ++i)
                order.m_SplatData[i] = copy[order.m_Order[i].Item2];
            copy.Dispose();

            order.m_Order.Dispose();
        }

        // Creator:431-440
        [BurstCompile]
        static unsafe void GatherSHs(int splatCount, InputSplatData* splatData, float* shData)
        {
            for (int i = 0; i < splatCount; ++i)
            {
                UnsafeUtility.MemCpy(shData, ((float*)splatData) + 9, 15 * 3 * sizeof(float));
                splatData++;
                shData += 15 * 3;
            }
        }

        // Creator:442-469
        [BurstCompile]
        struct ConvertSHClustersJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> m_Input;
            public NativeArray<GaussianSplatAsset.SHTableItemFloat16> m_Output;
            public void Execute(int index)
            {
                var addr = index * 15;
                GaussianSplatAsset.SHTableItemFloat16 res;
                res.sh1 = new half3(m_Input[addr + 0]);
                res.sh2 = new half3(m_Input[addr + 1]);
                res.sh3 = new half3(m_Input[addr + 2]);
                res.sh4 = new half3(m_Input[addr + 3]);
                res.sh5 = new half3(m_Input[addr + 4]);
                res.sh6 = new half3(m_Input[addr + 5]);
                res.sh7 = new half3(m_Input[addr + 6]);
                res.sh8 = new half3(m_Input[addr + 7]);
                res.sh9 = new half3(m_Input[addr + 8]);
                res.shA = new half3(m_Input[addr + 9]);
                res.shB = new half3(m_Input[addr + 10]);
                res.shC = new half3(m_Input[addr + 11]);
                res.shD = new half3(m_Input[addr + 12]);
                res.shE = new half3(m_Input[addr + 13]);
                res.shF = new half3(m_Input[addr + 14]);
                res.shPadding = default;
                m_Output[index] = res;
            }
        }

        // Creator:476-518
        static unsafe void ClusterSHsInternal(NativeArray<InputSplatData> splatData, GaussianSplatAsset.SHFormat format, out NativeArray<GaussianSplatAsset.SHTableItemFloat16> shs, out NativeArray<int> shIndices)
        {
            shs = default;
            shIndices = default;

            int shCount = GaussianSplatAsset.GetSHCount(format, splatData.Length);
            if (shCount >= splatData.Length) // no need to cluster, just use raw data
                return;

            const int kShDim = 15 * 3;
            const int kBatchSize = 2048;
            float passesOverData = format switch
            {
                GaussianSplatAsset.SHFormat.Cluster64k => 0.3f,
                GaussianSplatAsset.SHFormat.Cluster32k => 0.4f,
                GaussianSplatAsset.SHFormat.Cluster16k => 0.5f,
                GaussianSplatAsset.SHFormat.Cluster8k => 0.8f,
                GaussianSplatAsset.SHFormat.Cluster4k => 1.2f,
                _ => throw new ArgumentOutOfRangeException(nameof(format), format, null)
            };

            float t0 = Time.realtimeSinceStartup;
            NativeArray<float> shData = new(splatData.Length * kShDim, Allocator.Persistent);
            GatherSHs(splatData.Length, (InputSplatData*)splatData.GetUnsafeReadOnlyPtr(), (float*)shData.GetUnsafePtr());

            NativeArray<float> shMeans = new(shCount * kShDim, Allocator.Persistent);
            shIndices = new(splatData.Length, Allocator.Persistent);

            KMeansClustering.Calculate(kShDim, shData, kBatchSize, passesOverData, ClusterSHProgressStatic, shMeans, shIndices);
            shData.Dispose();

            shs = new NativeArray<GaussianSplatAsset.SHTableItemFloat16>(shCount, Allocator.Persistent);

            ConvertSHClustersJob job = new ConvertSHClustersJob
            {
                m_Input = shMeans.Reinterpret<float3>(4),
                m_Output = shs
            };
            job.Schedule(shCount, 256).Complete();
            shMeans.Dispose();
            float t1 = Time.realtimeSinceStartup;
            Debug.Log($"GS: clustered {splatData.Length / 1000000.0:F2}M SHs into {shCount / 1024}K ({passesOverData:F1}pass/{kBatchSize}batch) in {t1 - t0:F0}s");
        }

        // Creator:470-474 的静态版本签名（KMeansClustering.Calculate 需要 Func<float,bool>；转发到实例回调）
        static bool ClusterSHProgressStatic(float val)
        {
            if (s_ProgressForwarder != null)
                s_ProgressForwarder(val);
            return true;
        }
        static Action<float> s_ProgressForwarder; // 临时转发：ClusterSHsInternal 是 static（贴源码），实例进度经此到达

        // Creator:520-639（CalcChunkDataJob 照搬）
        [BurstCompile]
        struct CalcChunkDataJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction] public NativeArray<InputSplatData> splatData;
            public NativeArray<GaussianSplatAsset.ChunkInfo> chunks;

            public void Execute(int chunkIdx)
            {
                float3 chunkMinpos = float.PositiveInfinity;
                float3 chunkMinscl = float.PositiveInfinity;
                float4 chunkMincol = float.PositiveInfinity;
                float3 chunkMinshs = float.PositiveInfinity;
                float3 chunkMaxpos = float.NegativeInfinity;
                float3 chunkMaxscl = float.NegativeInfinity;
                float4 chunkMaxcol = float.NegativeInfinity;
                float3 chunkMaxshs = float.NegativeInfinity;

                int splatBegin = math.min(chunkIdx * GaussianSplatAsset.kChunkSize, splatData.Length);
                int splatEnd = math.min((chunkIdx + 1) * GaussianSplatAsset.kChunkSize, splatData.Length);

                // calculate data bounds inside the chunk
                for (int i = splatBegin; i < splatEnd; ++i)
                {
                    InputSplatData s = splatData[i];

                    // transform scale to be more uniformly distributed
                    s.scale = math.pow(s.scale, 1.0f / 8.0f);
                    // transform opacity to be more uniformly distributed
                    s.opacity = GaussianUtils.SquareCentered01(s.opacity);
                    splatData[i] = s;

                    chunkMinpos = math.min(chunkMinpos, s.pos);
                    chunkMinscl = math.min(chunkMinscl, s.scale);
                    chunkMincol = math.min(chunkMincol, new float4(s.dc0, s.opacity));
                    chunkMinshs = math.min(chunkMinshs, s.sh1);
                    chunkMinshs = math.min(chunkMinshs, s.sh2);
                    chunkMinshs = math.min(chunkMinshs, s.sh3);
                    chunkMinshs = math.min(chunkMinshs, s.sh4);
                    chunkMinshs = math.min(chunkMinshs, s.sh5);
                    chunkMinshs = math.min(chunkMinshs, s.sh6);
                    chunkMinshs = math.min(chunkMinshs, s.sh7);
                    chunkMinshs = math.min(chunkMinshs, s.sh8);
                    chunkMinshs = math.min(chunkMinshs, s.sh9);
                    chunkMinshs = math.min(chunkMinshs, s.shA);
                    chunkMinshs = math.min(chunkMinshs, s.shB);
                    chunkMinshs = math.min(chunkMinshs, s.shC);
                    chunkMinshs = math.min(chunkMinshs, s.shD);
                    chunkMinshs = math.min(chunkMinshs, s.shE);
                    chunkMinshs = math.min(chunkMinshs, s.shF);

                    chunkMaxpos = math.max(chunkMaxpos, s.pos);
                    chunkMaxscl = math.max(chunkMaxscl, s.scale);
                    chunkMaxcol = math.max(chunkMaxcol, new float4(s.dc0, s.opacity));
                    chunkMaxshs = math.max(chunkMaxshs, s.sh1);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh2);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh3);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh4);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh5);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh6);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh7);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh8);
                    chunkMaxshs = math.max(chunkMaxshs, s.sh9);
                    chunkMaxshs = math.max(chunkMaxshs, s.shA);
                    chunkMaxshs = math.max(chunkMaxshs, s.shB);
                    chunkMaxshs = math.max(chunkMaxshs, s.shC);
                    chunkMaxshs = math.max(chunkMaxshs, s.shD);
                    chunkMaxshs = math.max(chunkMaxshs, s.shE);
                    chunkMaxshs = math.max(chunkMaxshs, s.shF);
                }

                // make sure bounds are not zero
                chunkMaxpos = math.max(chunkMaxpos, chunkMinpos + 1.0e-5f);
                chunkMaxscl = math.max(chunkMaxscl, chunkMinscl + 1.0e-5f);
                chunkMaxcol = math.max(chunkMaxcol, chunkMincol + 1.0e-5f);
                chunkMaxshs = math.max(chunkMaxshs, chunkMinshs + 1.0e-5f);

                // store chunk info
                GaussianSplatAsset.ChunkInfo info = default;
                info.posX = new float2(chunkMinpos.x, chunkMaxpos.x);
                info.posY = new float2(chunkMinpos.y, chunkMaxpos.y);
                info.posZ = new float2(chunkMinpos.z, chunkMaxpos.z);
                info.sclX = math.f32tof16(chunkMinscl.x) | (math.f32tof16(chunkMaxscl.x) << 16);
                info.sclY = math.f32tof16(chunkMinscl.y) | (math.f32tof16(chunkMaxscl.y) << 16);
                info.sclZ = math.f32tof16(chunkMinscl.z) | (math.f32tof16(chunkMaxscl.z) << 16);
                info.colR = math.f32tof16(chunkMincol.x) | (math.f32tof16(chunkMaxcol.x) << 16);
                info.colG = math.f32tof16(chunkMincol.y) | (math.f32tof16(chunkMaxcol.y) << 16);
                info.colB = math.f32tof16(chunkMincol.z) | (math.f32tof16(chunkMaxcol.z) << 16);
                info.colA = math.f32tof16(chunkMincol.w) | (math.f32tof16(chunkMaxcol.w) << 16);
                info.shR = math.f32tof16(chunkMinshs.x) | (math.f32tof16(chunkMaxshs.x) << 16);
                info.shG = math.f32tof16(chunkMinshs.y) | (math.f32tof16(chunkMaxshs.y) << 16);
                info.shB = math.f32tof16(chunkMinshs.z) | (math.f32tof16(chunkMaxshs.z) << 16);
                chunks[chunkIdx] = info;

                // adjust data to be 0..1 within chunk bounds
                for (int i = splatBegin; i < splatEnd; ++i)
                {
                    InputSplatData s = splatData[i];
                    s.pos = ((float3)s.pos - chunkMinpos) / (chunkMaxpos - chunkMinpos);
                    s.scale = ((float3)s.scale - chunkMinscl) / (chunkMaxscl - chunkMinscl);
                    s.dc0 = ((float3)s.dc0 - chunkMincol.xyz) / (chunkMaxcol.xyz - chunkMincol.xyz);
                    s.opacity = (s.opacity - chunkMincol.w) / (chunkMaxcol.w - chunkMincol.w);
                    s.sh1 = ((float3)s.sh1 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh2 = ((float3)s.sh2 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh3 = ((float3)s.sh3 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh4 = ((float3)s.sh4 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh5 = ((float3)s.sh5 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh6 = ((float3)s.sh6 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh7 = ((float3)s.sh7 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh8 = ((float3)s.sh8 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.sh9 = ((float3)s.sh9 - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.shA = ((float3)s.shA - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.shB = ((float3)s.shB - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.shC = ((float3)s.shC - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.shD = ((float3)s.shD - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.shE = ((float3)s.shE - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    s.shF = ((float3)s.shF - chunkMinshs) / (chunkMaxshs - chunkMinshs);
                    splatData[i] = s;
                }
            }
        }

        // Creator:660-703 ConvertColorJob（照搬）
        [BurstCompile]
        struct ConvertColorJob : IJobParallelFor
        {
            public int width, height;
            [ReadOnly] public NativeArray<float4> inputData;
            [NativeDisableParallelForRestriction] public NativeArray<byte> outputData;
            public GaussianSplatAsset.ColorFormat format;
            public int formatBytesPerPixel;

            public unsafe void Execute(int y)
            {
                int srcIdx = y * width;
                byte* dstPtr = (byte*)outputData.GetUnsafePtr() + y * width * formatBytesPerPixel;
                for (int x = 0; x < width; ++x)
                {
                    float4 pix = inputData[srcIdx];

                    switch (format)
                    {
                        case GaussianSplatAsset.ColorFormat.Float32x4:
                        {
                            *(float4*)dstPtr = pix;
                        }
                            break;
                        case GaussianSplatAsset.ColorFormat.Float16x4:
                        {
                            half4 enc = new half4(pix);
                            *(half4*)dstPtr = enc;
                        }
                            break;
                        case GaussianSplatAsset.ColorFormat.Norm8x4:
                        {
                            pix = math.saturate(pix);
                            uint enc = (uint)(pix.x * 255.5f) | ((uint)(pix.y * 255.5f) << 8) | ((uint)(pix.z * 255.5f) << 16) | ((uint)(pix.w * 255.5f) << 24);
                            *(uint*)dstPtr = enc;
                        }
                            break;
                    }

                    srcIdx++;
                    dstPtr += formatBytesPerPixel;
                }
            }
        }

        // Creator:705-725（照搬）
        static ulong EncodeFloat3ToNorm16(float3 v) // 48 bits: 16.16.16
        {
            return (ulong)(v.x * 65535.5f) | ((ulong)(v.y * 65535.5f) << 16) | ((ulong)(v.z * 65535.5f) << 32);
        }
        static uint EncodeFloat3ToNorm11(float3 v) // 32 bits: 11.10.11
        {
            return (uint)(v.x * 2047.5f) | ((uint)(v.y * 1023.5f) << 11) | ((uint)(v.z * 2047.5f) << 21);
        }
        static ushort EncodeFloat3ToNorm655(float3 v) // 16 bits: 6.5.5
        {
            return (ushort)((uint)(v.x * 63.5f) | ((uint)(v.y * 31.5f) << 6) | ((uint)(v.z * 31.5f) << 11));
        }
        static ushort EncodeFloat3ToNorm565(float3 v) // 16 bits: 5.6.5
        {
            return (ushort)((uint)(v.x * 31.5f) | ((uint)(v.y * 63.5f) << 5) | ((uint)(v.z * 31.5f) << 11));
        }

        static uint EncodeQuatToNorm10(float4 v) // 32 bits: 10.10.10.2
        {
            return (uint)(v.x * 1023.5f) | ((uint)(v.y * 1023.5f) << 10) | ((uint)(v.z * 1023.5f) << 20) | ((uint)(v.w * 3.5f) << 30);
        }

        // Creator:727-758（照搬）
        static unsafe void EmitEncodedVector(float3 v, byte* outputPtr, GaussianSplatAsset.VectorFormat format)
        {
            switch (format)
            {
                case GaussianSplatAsset.VectorFormat.Float32:
                {
                    *(float*)outputPtr = v.x;
                    *(float*)(outputPtr + 4) = v.y;
                    *(float*)(outputPtr + 8) = v.z;
                }
                    break;
                case GaussianSplatAsset.VectorFormat.Norm16:
                {
                    ulong enc = EncodeFloat3ToNorm16(math.saturate(v));
                    *(uint*)outputPtr = (uint)enc;
                    *(ushort*)(outputPtr + 4) = (ushort)(enc >> 32);
                }
                    break;
                case GaussianSplatAsset.VectorFormat.Norm11:
                {
                    uint enc = EncodeFloat3ToNorm11(math.saturate(v));
                    *(uint*)outputPtr = enc;
                }
                    break;
                case GaussianSplatAsset.VectorFormat.Norm6:
                {
                    ushort enc = EncodeFloat3ToNorm655(math.saturate(v));
                    *(ushort*)outputPtr = enc;
                }
                    break;
            }
        }

        // Creator:760-773 CreatePositionsDataJob（照搬）
        [BurstCompile]
        struct CreatePositionsDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<InputSplatData> m_Input;
            public GaussianSplatAsset.VectorFormat m_Format;
            public int m_FormatSize;
            [NativeDisableParallelForRestriction] public NativeArray<byte> m_Output;

            public unsafe void Execute(int index)
            {
                byte* outputPtr = (byte*)m_Output.GetUnsafePtr() + index * m_FormatSize;
                EmitEncodedVector(m_Input[index].pos, outputPtr, m_Format);
            }
        }

        // Creator:775-805 CreateOtherDataJob（照搬）
        [BurstCompile]
        struct CreateOtherDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<InputSplatData> m_Input;
            [NativeDisableContainerSafetyRestriction] [ReadOnly] public NativeArray<int> m_SplatSHIndices;
            public GaussianSplatAsset.VectorFormat m_ScaleFormat;
            public int m_FormatSize;
            [NativeDisableParallelForRestriction] public NativeArray<byte> m_Output;

            public unsafe void Execute(int index)
            {
                byte* outputPtr = (byte*)m_Output.GetUnsafePtr() + index * m_FormatSize;

                // rotation: 4 bytes
                {
                    Quaternion rotQ = m_Input[index].rot;
                    float4 rot = new float4(rotQ.x, rotQ.y, rotQ.z, rotQ.w);
                    uint enc = EncodeQuatToNorm10(rot);
                    *(uint*)outputPtr = enc;
                    outputPtr += 4;
                }

                // scale: 6, 4 or 2 bytes
                EmitEncodedVector(m_Input[index].scale, outputPtr, m_ScaleFormat);
                outputPtr += GaussianSplatAsset.GetVectorSize(m_ScaleFormat);

                // SH index
                if (m_SplatSHIndices.IsCreated)
                    *(ushort*)outputPtr = (ushort)m_SplatSHIndices[index];
            }
        }

        // Creator:807-810（照搬）
        static int NextMultipleOf(int size, int multipleOf)
        {
            return (size + multipleOf - 1) / multipleOf * multipleOf;
        }

        // Creator:863-871（照搬）
        static int SplatIndexToTextureIndex(uint idx)
        {
            uint2 xy = GaussianUtils.DecodeMorton2D_16x16(idx);
            uint width = GaussianSplatAsset.kTextureWidth / 16;
            idx >>= 8;
            uint x = (idx % width) * 16 + xy.x;
            uint y = (idx / width) * 16 + xy.y;
            return (int)(y * GaussianSplatAsset.kTextureWidth + x);
        }

        // Creator:873-885 CreateColorDataJob（照搬）
        [BurstCompile]
        struct CreateColorDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<InputSplatData> m_Input;
            [NativeDisableParallelForRestriction] public NativeArray<float4> m_Output;

            public void Execute(int index)
            {
                var splat = m_Input[index];
                int i = SplatIndexToTextureIndex((uint)index);
                m_Output[i] = new float4(splat.dc0.x, splat.dc0.y, splat.dc0.z, splat.opacity);
            }
        }

        // Creator:934-1037 CreateSHDataJob（照搬）
        [BurstCompile]
        struct CreateSHDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<InputSplatData> m_Input;
            public GaussianSplatAsset.SHFormat m_Format;
            public NativeArray<byte> m_Output;
            public unsafe void Execute(int index)
            {
                var splat = m_Input[index];

                switch (m_Format)
                {
                    case GaussianSplatAsset.SHFormat.Float32:
                    {
                        GaussianSplatAsset.SHTableItemFloat32 res;
                        res.sh1 = splat.sh1;
                        res.sh2 = splat.sh2;
                        res.sh3 = splat.sh3;
                        res.sh4 = splat.sh4;
                        res.sh5 = splat.sh5;
                        res.sh6 = splat.sh6;
                        res.sh7 = splat.sh7;
                        res.sh8 = splat.sh8;
                        res.sh9 = splat.sh9;
                        res.shA = splat.shA;
                        res.shB = splat.shB;
                        res.shC = splat.shC;
                        res.shD = splat.shD;
                        res.shE = splat.shE;
                        res.shF = splat.shF;
                        res.shPadding = default;
                        ((GaussianSplatAsset.SHTableItemFloat32*)m_Output.GetUnsafePtr())[index] = res;
                    }
                        break;
                    case GaussianSplatAsset.SHFormat.Float16:
                    {
                        GaussianSplatAsset.SHTableItemFloat16 res;
                        res.sh1 = new half3(splat.sh1);
                        res.sh2 = new half3(splat.sh2);
                        res.sh3 = new half3(splat.sh3);
                        res.sh4 = new half3(splat.sh4);
                        res.sh5 = new half3(splat.sh5);
                        res.sh6 = new half3(splat.sh6);
                        res.sh7 = new half3(splat.sh7);
                        res.sh8 = new half3(splat.sh8);
                        res.sh9 = new half3(splat.sh9);
                        res.shA = new half3(splat.shA);
                        res.shB = new half3(splat.shB);
                        res.shC = new half3(splat.shC);
                        res.shD = new half3(splat.shD);
                        res.shE = new half3(splat.shE);
                        res.shF = new half3(splat.shF);
                        res.shPadding = default;
                        ((GaussianSplatAsset.SHTableItemFloat16*)m_Output.GetUnsafePtr())[index] = res;
                    }
                        break;
                    case GaussianSplatAsset.SHFormat.Norm11:
                    {
                        GaussianSplatAsset.SHTableItemNorm11 res;
                        res.sh1 = EncodeFloat3ToNorm11(splat.sh1);
                        res.sh2 = EncodeFloat3ToNorm11(splat.sh2);
                        res.sh3 = EncodeFloat3ToNorm11(splat.sh3);
                        res.sh4 = EncodeFloat3ToNorm11(splat.sh4);
                        res.sh5 = EncodeFloat3ToNorm11(splat.sh5);
                        res.sh6 = EncodeFloat3ToNorm11(splat.sh6);
                        res.sh7 = EncodeFloat3ToNorm11(splat.sh7);
                        res.sh8 = EncodeFloat3ToNorm11(splat.sh8);
                        res.sh9 = EncodeFloat3ToNorm11(splat.sh9);
                        res.shA = EncodeFloat3ToNorm11(splat.shA);
                        res.shB = EncodeFloat3ToNorm11(splat.shB);
                        res.shC = EncodeFloat3ToNorm11(splat.shC);
                        res.shD = EncodeFloat3ToNorm11(splat.shD);
                        res.shE = EncodeFloat3ToNorm11(splat.shE);
                        res.shF = EncodeFloat3ToNorm11(splat.shF);
                        ((GaussianSplatAsset.SHTableItemNorm11*)m_Output.GetUnsafePtr())[index] = res;
                    }
                        break;
                    case GaussianSplatAsset.SHFormat.Norm6:
                    {
                        GaussianSplatAsset.SHTableItemNorm6 res;
                        res.sh1 = EncodeFloat3ToNorm565(splat.sh1);
                        res.sh2 = EncodeFloat3ToNorm565(splat.sh2);
                        res.sh3 = EncodeFloat3ToNorm565(splat.sh3);
                        res.sh4 = EncodeFloat3ToNorm565(splat.sh4);
                        res.sh5 = EncodeFloat3ToNorm565(splat.sh5);
                        res.sh6 = EncodeFloat3ToNorm565(splat.sh6);
                        res.sh7 = EncodeFloat3ToNorm565(splat.sh7);
                        res.sh8 = EncodeFloat3ToNorm565(splat.sh8);
                        res.sh9 = EncodeFloat3ToNorm565(splat.sh9);
                        res.shA = EncodeFloat3ToNorm565(splat.shA);
                        res.shB = EncodeFloat3ToNorm565(splat.shB);
                        res.shC = EncodeFloat3ToNorm565(splat.shC);
                        res.shD = EncodeFloat3ToNorm565(splat.shD);
                        res.shE = EncodeFloat3ToNorm565(splat.shE);
                        res.shF = EncodeFloat3ToNorm565(splat.shF);
                        res.shPadding = default;
                        ((GaussianSplatAsset.SHTableItemNorm6*)m_Output.GetUnsafePtr())[index] = res;
                    }
                        break;
                    default:
                        break;
                }
            }
        }

        // Creator:1039-1044（改签名：filePath → byte[] 返回）
        static byte[] EmitSimpleDataFile<T>(NativeArray<T> data, ref Hash128 dataHash) where T : unmanaged
        {
            dataHash.Append(data);
            return data.Reinterpret<byte>(UnsafeUtility.SizeOf<T>()).ToArray(); // Creator:1042-1043 FileStream → ToArray
        }

        // Creator:1120-1131（照搬）
        [Serializable]
        public class JsonCamera
        {
            public int id;
            public string img_name;
            public int width;
            public int height;
            public float[] position;
            public float[][] rotation;
            public float fx;
            public float fy;
        }
    }
}
