#version 330 compatibility

in vec3 vN;
in vec3 vE;
in float vHeight;

uniform float atmosphereStart;
uniform float atmosphereThickness;
uniform vec4 innerColor = vec4(0.4, 0.7, 1.0, 0.3);
uniform vec4 outerColor = vec4(0.1, 0.2, 0.8, 0.0);

out vec4 FragColor;

void main() {
    vec3 lightPos = vec3(gl_LightSource[0].position);
    vec3 V = normalize(-vE);
    vec3 N = normalize(vN);
    vec3 L = normalize(lightPos - vE);
    
    // Solar corona effect
    float sunAngle = dot(-V, L);  // Angle between view and light
    float coronaIntensity = smoothstep(0.95, 0.98, sunAngle); 
    vec4 coronaColor = vec4(1.0, 0.3, 0.1, 1.0) * coronaIntensity * 2.0;  
    
    float rim = 1.0 - max(dot(V, N), 0.0);
    rim = pow(rim, 1.5); 
    
    float diffuse = max(dot(N, L), 0.0);
    
    float heightFactor = vHeight / atmosphereThickness;
    
    heightFactor = smoothstep(-0.4, 1.4, heightFactor);
    
    // Multi-scatter approximation
    float forward = pow(max(dot(L, V), 0.0), 2.0); 
    float back = pow(max(dot(L, -V), 0.0), 1.5);    
    float scatter = mix(back, forward, 0.7);      
    
    float density = exp(-heightFactor * 1.5); 
    
    float lightFactor = smoothstep(-0.3, 0.3, dot(N, L)); 
    
    vec4 atmosColor = mix(innerColor, outerColor, pow(heightFactor, 0.7)); 
    
    // Combine effects
    float totalIntensity = mix(scatter, rim, 0.6) + 0.2;
    
    FragColor = mix(
        atmosColor * totalIntensity * density,
        coronaColor,
        coronaIntensity
    );
    
    float edgeFade = pow(1.0 - heightFactor, 3.0);
    
    // Apply a softer edge falloff
    FragColor.a *= lightFactor * edgeFade * (rim + scatter + coronaIntensity + 0.2);
    FragColor.a *= smoothstep(0.0, 0.4, density);
}