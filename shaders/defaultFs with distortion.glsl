#version 330

uniform sampler2D srcTex;
uniform vec2 LensCenter;
uniform vec2 ScreenCenter;
uniform vec2 Scale;
uniform vec2 ScaleIn;
uniform vec4 HmdWarpParam;
in vec2 texCoordFs;
out vec4 color;
    
vec2 HmdWarp(vec2 in_param) {
	vec2 theta = (in_param - LensCenter) * ScaleIn;
	float rSq = theta.x * theta.x * theta.y * theta.y;
	vec2 theta1 = theta * (HmdWarpParam.x + HmdWarpParam.y * rSq + HmdWarpParam.z * rSq * rSq + HmdWarpParam.w * rSq * rSq * rSq);
	
	return LensCenter + Scale * theta1;
}

void main() {
	vec2 tc = HmdWarp(texCoordFs);
	if(!all(equal(clamp(tc, ScreenCenter-vec2(0.25, 0.5), ScreenCenter + vec2(0.25, 0.5)), tc)))
		color = vec4(0.0, 0.0, 0.0, 0.0);
	else
		color = texture(srcTex, tc);
}

//#version 330
//uniform sampler2D srcTex;
//in vec2 texCoordFs;
//out vec4 color;
//void main() {
//	color = texture(srcTex, texCoordFs);
//}