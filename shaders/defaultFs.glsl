#version 330

uniform sampler2D srcTex;
uniform float k1;
uniform vec2 texScale;
in vec2 texCoordFs;
out vec4 color;

void main() {
	vec2 screenDimen = vec2(1, 1);
	
// Debug code: texture normally
//    gl_FragColor = texture2DRect(tex, gl_TexCoord[0].st);
//    return;

	// We are rendering onto a texture rectangle and texture
	// coordinates are [0,0] to (W,H) instead of [0,0] to (1,1).

	vec2 screenCenter = screenDimen/2.0;  /* Find the screen center */
	float norm = length(screenCenter);  /* Distance between corner and center */

	// get a vector from center to where we are at now (in screen space) and normalize it
	vec2 radial_vector = ( texCoordFs - screenCenter ) / norm;
	float radial_vector_len = length(radial_vector);
	vec2 radial_vector_unit = radial_vector / radial_vector_len;

/*
// Debug code:
gl_FragColor = vec4(ru, ru, ru, 1.0);
if(ru >= .95)
gl_FragColor = vec4(1.0, 0, 1.0, 1.0);
return;
*/

	// Compute the new distance from the screen center.
	float new_dist = radial_vector_len + k1 * pow(radial_vector_len,3.0);

	/* Now, compute texture coordinate we want to lookup. */

	// Find the coordinate we want to lookup
	vec2 warp_coord = radial_vector_unit * (new_dist * norm);

	// Scale the image vertically and horizontally to get it to fill the screen
	warp_coord = warp_coord * texScale;

	// Translate the coordinte such that the (0,0) is back at the screen center
	warp_coord = warp_coord + screenCenter;


	/* If we lookup a texture coordinate that is not on the texture, return a solid color */
	if ((warp_coord.s > float(screenDimen.x)  || warp_coord.s < 0.0) ||
	    (warp_coord.t > float(screenDimen.y) || warp_coord.t < 0.0))
		color = vec4(0,0,0,1); // black
	else
		color = texture(srcTex, warp_coord);  // lookup into the texture
}