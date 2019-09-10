// const float PI = 3.1415926535;
// const float aPI = acos(-1.);

// float rand(float n){return fract(sin(n) * 43758.5453123);}

// // Polynomial smooth min (for copying and pasting into your shaders)
// float smin(float a, float b, float k) {
//     float h = clamp(0.5 + 0.5*(a-b)/k, 0.0, 1.0);
//     return mix(a, b, h) - k*h*(1.0-h);
// }

// float smax(float a,float b,float k){
//     return smin(a,b,-k);
// }


void mainImage( out vec4 fragColor, in vec2 fragCoord )
{


    // vec2 uv = (fragCoord.xy - .5 * iResolution.xy)/iResolution.x; // uv -.5 ~ .5  , x axis is scale t0 1.

    
    // // uv.y -= wordOffset;// whole screen offset ======================
    // // uv *= 0.5;// 0 ~ 1
    
    // vec2 st = vec2(atan(uv.x,uv.y),length(uv));
    // //st.x += PI;// 0 ~ 2PI on -y axis
    
    // float y = 0.0;
        

    vec3 col;

    col = vec3(1.0,0.0,0.0);

    // Output to screen
    fragColor = vec4(col,1.0);
}