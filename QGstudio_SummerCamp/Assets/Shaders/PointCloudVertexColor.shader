Shader "PointCloud/VertexColor"
{
    // 用途：点云冒烟渲染（MeshTopology.Points + SetColors 的顶点色）
    SubShader
    {
        Tags { "RenderPipeline" = "UniversalPipeline" "Queue" = "Geometry" }

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            
            struct Attributes
            {
                float4 positionOS : POSITION; 
                float4 color      : COLOR;      // 顶点色（RGBA，来自 Mesh 的 colors）
            };
            
            struct Varyings
            {
                float4 positionHCS : SV_POSITION; 
                float4 color       : COLOR;        
            };

            Varyings vert(Attributes input)
            {
                Varyings output;
                output.positionHCS = TransformObjectToHClip(input.positionOS.xyz);
                output.color = input.color;      
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                return input.color;              
            }
            ENDHLSL
        }
    }
}
