#version 330 compatibility

out vec3 vN;
out vec3 vE;
out float vHeight;

void main() {
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    vN = normalize(gl_NormalMatrix * gl_Normal);
    vE = vec3(gl_ModelViewMatrix * gl_Vertex);
    
    // Calculate true height above surface
    vHeight = length(gl_Vertex.xyz) - 1.0; // Assuming Earth radius is 1.0
}