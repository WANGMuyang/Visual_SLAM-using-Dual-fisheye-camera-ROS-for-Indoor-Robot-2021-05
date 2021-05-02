#version 450 core
#define PI 3.14159265358979
// #define _THETA_S_Y_SCALE	(640.0 / 720.0)
#define _THETA_S_Y_SCALE	(1920.0 / 960.0)

uniform vec2 resolution;        // resolution are given by opengl_ros_lib/simper_renderer
uniform sampler2D texture;      // texture are given by opengl_ros_lib/simper_renderer

uniform lowp float radius;
uniform lowp vec4 uvOffset;

in vec4 gl_FragCoord;
out vec4 gl_FragColor;

void main(void)
{   
    radius  = 0.445
    uvOffset = {0, 0, 0, 0};

    float u = gl_FragCoord.x / resolution.x;
    float v = gl_FragCoord.y / resolution.y;
    
    vec3 color = texture2D(texture, vec2(u, v)).xyz;

    lowp vec2 textureCoordinate = vec2(u,v);
    lowp vec2 revUV = vec2(u,v);

    if (textureCoordinate[0] <= 0.5) {
        revUV.x = revUV.x * 2.0;
    } else {
        revUV.x = (revUV.x - 0.5) * 2.0;
    }
    
    revUV *= PI;
    
    lowp vec3 p = vec3(cos(revUV.x), cos(revUV.y), sin(revUV.x));
    p.xz *= sqrt(1.0 - p.y * p.y);
    
    lowp float r = 1.0 - asin(p.z) / (PI / 2.0);
    lowp vec2 st = vec2(p.y, p.x);
    
    st *= r / sqrt(1.0 - p.z * p.z);
    st *= radius;
    st += 0.5;
    
    if (textureCoordinate.x <= 0.5) {
        st.x *= 0.5;
        st.x += 0.5;
        st.y = 1.0 - st.y;
        st.xy += uvOffset.wz;
    } else {
        st.x = 1.0 - st.x;
        st.x *= 0.5;
        st.xy += uvOffset.yx;
    }
    
    st.y = st.y * _THETA_S_Y_SCALE;
    
    gl_FragColor = texture2D(texture, st);
}