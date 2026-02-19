Shader "Custom/DistanceBasedColor"
{
    Properties
    {
        _PointSize("Point Size", Float) = 1.0
        _MinDistance("Min Distance", Float) = 0.0
        _MaxDistance("Max Distance", Float) = 10.0
        _NearColor("Near Color", Color) = (1, 0, 0, 1) // Rojo por defecto
        _FarColor("Far Color", Color) = (0, 0, 1, 1) // Azul por defecto
    }
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Opaque" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            // Uniforms (propiedades del material)
            float _PointSize;
            float _MinDistance;
            float _MaxDistance;
            float4 _NearColor;
            float4 _FarColor;

            // Vertex shader
            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float distance : TEXCOORD0;
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);

                // Calcular la distancia al origen (0, 0, 0)
                float3 worldPos = mul(unity_ObjectToWorld, v.vertex).xyz;
                o.distance = length(worldPos); // Distancia al origen

                return o;
            }

            // Fragment shader
            half4 frag(v2f i) : SV_Target
            {
                // Normalizar la distancia entre Min y Max
                float t = saturate((i.distance - _MinDistance) / (_MaxDistance - _MinDistance));

                // Interpolación de colores entre _NearColor y _FarColor
                half4 color = lerp(_NearColor, _FarColor, t);

                return color;
            }
            ENDCG
        }
    }
}

