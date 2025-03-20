#version 330 compatibility

in vec3 vN;
in vec2 vST;
in vec3 vE;

uniform sampler2D dayTexture;
uniform sampler2D nightTexture;
uniform sampler2D cloudTexture;
uniform sampler2D bumpTexture;
uniform float timer;

out vec4 FragColor;

void main() {
    vec3 lightPos = vec3(gl_LightSource[0].position);
    
    vec3 bump = normalize(texture(bumpTexture, vST).rgb * 2.0 - 1.0);
    vec3 N = normalize(vN + bump * 0.1);
    vec3 L = normalize(lightPos - vE);
    vec3 V = normalize(-vE);
    
    float NdotL = dot(N, L);
    
    // Base textures
    vec4 dayColor = texture(dayTexture, vST) * 0.9; 
    
    // Night lights
    vec4 nightTex = texture(nightTexture, vST);
    vec4 nightColor = pow(nightTex, vec4(0.4)) * 0.6; 
    
    // Animated cloud texture
    vec2 cloudST = vST;
    float rotationPeriod = 6.0 * 3600.0 * 1000.0;
    cloudST.x += (timer / rotationPeriod);
    vec4 cloudTex = texture(cloudTexture, cloudST);
    
    // Day/night transition
    float dayFactor = smoothstep(-0.15, 0.25, NdotL); 
    
    // Mix day and night textures
    vec4 darkNight = vec4(0.005, 0.005, 0.02, 1.0); 
    vec4 nightSide = mix(darkNight, nightColor, pow(nightTex.r, 0.25) * 1.8); 
    vec4 groundColor = mix(nightSide, dayColor, dayFactor);
    
    // Add clouds with lighting
    float cloudBrightness = mix(0.03, 0.95, dayFactor); 
    vec4 cloudColor = vec4(cloudBrightness);
    groundColor = mix(groundColor, cloudColor, cloudTex.r * 0.5);
    
    // Ambient light for night
    vec4 ambient = vec4(0.01, 0.01, 0.02, 1.0);
    
    FragColor = groundColor + ambient;
}