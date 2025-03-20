#version 330 compatibility

out vec3 vN;  // normal vector
out vec2 vST; // (s,t) texture coordinates
out vec3 vE;  // vector from point to eye

void main() {
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    
    vN = normalize(gl_NormalMatrix * gl_Normal);
    
    vST = gl_MultiTexCoord0.st;
    vE = vec3(gl_ModelViewMatrix * gl_Vertex);
}