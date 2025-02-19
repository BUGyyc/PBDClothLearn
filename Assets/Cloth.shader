﻿Shader "Custom/Cloth"
{
    Properties
    {
    }
    SubShader
    {
        Tags { "RenderType"="Opaque"}

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            struct Attributes
            {
                float4 positionOS   : POSITION;
                float3 normalOS : NORMAL;
            };

            struct Varyings
            {
                float4 positionHCS  : SV_POSITION;
                half4 color           : COLOR;
            };

            Varyings vert(Attributes IN)
            {
                Varyings OUT;
                OUT.positionHCS = TransformObjectToHClip(IN.positionOS.xyz);
                OUT.color.xyz = IN.normalOS * 0.5 + 0.5; //把 -1到1 映射到 0到1
                OUT.color.w = 1.0;
                return OUT;
            }

            half4 frag(Varyings IN) : SV_Target
            {
                // return half4(0.3,0.9,0.5,1);
                return IN.color;
            }
            ENDHLSL
        }
    }
    FallBack "Diffuse"
}
