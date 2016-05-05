// uniform float pNear;
// uniform float pFar;

varying float depth;

//vec4 packFloatToVec4i(const float value)
//{
// const vec4 bitSh = vec4(256.0*256.0*256.0, 256.0*256.0, 256.0, 1.0);
//  const vec4 bitMsk = vec4(0.0, 1.0/256.0, 1.0/256.0, 1.0/256.0);
//  vec4 res = fract(value * bitSh);
//  res -= res.xxyz * bitMsk;
//  return res;
// }
// vec4 pack (float d) {
//      unsigned int32 mask;
//      mask = 255;
//      unsigned int8 d0;
//      unsigned int8 d1;
//      unsigned int8 d2;
//      unsigned int8 d3;
//      d0 =  mask &  ((d<<24)>>24);
//      d1 = mask &  ((d<<16)>>24);
//      d2 = mask &  ((d<<8)>>24);
//      d3 = mask &  ((d<<0)>>24);
//      return vec4(d0,d1,d2,d3);
// }

vec3 pack3(float d)
{
	float d1;
	float d2;
	float d3;
	d1 = floor(d/(255.0*255.0));
	d = d-d1*(255.0*255.0);
	d2 = floor(d/255.0);
	d3 = d - d2*255.0;
	return vec3(d1/255.0, d2/255.0, d3/255.0);
}

void main()
{
  // This normalizes the depth value
  //gl_FragColor = vec4(vec3(depth / (pFar - pNear)), 1.0);
 
  // This returns the world position 
  //gl_FragColor = vec4(vec3(depth/10000), 1.0);
  float depth2;
  depth2 = .5;
  //gl_FragColor = packFloatToVec4i(depth2);

  gl_FragColor = vec4(pack3(10.0*depth), 1.0);
}
