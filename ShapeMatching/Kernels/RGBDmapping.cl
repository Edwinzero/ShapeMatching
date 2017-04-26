// float3 is considered as float4 by OpenCL
// alignment can also be enforced by using __attribute__ ((aligned (16)));
#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics: enable
#pragma OPENCL EXTENSION cl_khr_local_int32_base_atomics : enable

#ifdef DOUBLE_SUPPORT
#ifdef cl_amd_fp64
#pragma OPENCL EXTENSION cl_amd_fp64:enable
#elif defined (cl_khr_fp64)
#pragma OPENCL EXTENSION cl_khr_fp64:enable
#endif
#endif

#define MAX_DEPTH (10000.0f)

// stage 1: backproject points (for display)
__kernel void depth_to_point(__global unsigned short* depth, __global float3 *points, int2 dim, float4 IK)
{
	int x = get_global_id(0);
	int y = get_global_id(1);
	if(x >= dim.x || y >= dim.y){ return; }
	int id = y * dim.x + x;

	// get uv, d
	int2 uv = (int2)(x, y);
	uv = clamp(uv, (int2)(0), dim-1);
	float d = convert_float_rtz(depth[y * dim.x + x]) * 0.001;

#if 0
	if (d < 0.025 || d > 5.0) {
		d = 0.0;
	}

	if (uv.x < 10 || uv.x > dim.x - 10 || uv.y < 10 || uv.y > dim.y - 10) {
		d = 0.0;
	}
#endif

	float3 p = (float3)(uv.x, uv.y, d);
	p.xy -= IK.hi;
	p.xy /= IK.lo;
	p.xy *= d;

	points[id] = p;
}

__kernel void point_to_world(__global float3 *points, __global float3 *worlds, int2 dim, float16 EM)
{
	int x = get_global_id(0);
	int y = get_global_id(1);
	if(x >= dim.x || y >= dim.y){ return; }
	int id = y * dim.x + x;

	// EM * points = worlds
	float3 v = points[id];
	// EM here is the inverse of Extrinsic matrix (points -> world)
	worlds[id] = EM.lo.lo.xyz * v.x + EM.lo.hi.xyz * v.y + EM.hi.lo.xyz * v.z + EM.hi.hi.xyz;
}

__kernel void point_to_normal(__global float3 *worlds, __global float3 *normals, int2 dim)
{
	int x = get_global_id(0);
	int y = get_global_id(1);
	if(x >= dim.x || y >= dim.y){ return; }
	int id = y * dim.x + x;

	x = clamp(x, 1, dim.x-2);
	y = clamp(y, 1, dim.y-2);
	int k = y * dim.x + x;
	float3 s = worlds[k+1] - worlds[k-1];
	float3 t = worlds[k+dim.x] - worlds[k-dim.x];

	normals[id] = normalize(-cross(s, t));
}

// stage 2: fuse points into TSDF field (for reconstruction)
#define TRUNC		(3.0f)  // 3mm
#define MAX_WEIGHT	(2.0f)
float get_depth_16u(__global unsigned short *depth_map, int2 dim, float2 v) {
	int2 p = clamp(convert_int2_rtn(v), (int2)(0), dim - 1);
	float d = convert_float_rtn(depth_map[p.x + p.y * dim.x]);

#if 1
	if (d < 25.0 || d > 5000.0) {
		return MAX_DEPTH;
	}

	if (p.x < 10 || p.x > dim.x - 10 || p.y < 10 || p.y > dim.y - 10) {
		return MAX_DEPTH;
	}
#endif
	return d * 0.001f;
}


float3 get_normal( __global float3 *normal_map, int2 dim, float2 v ) {
	int2 p = clamp( convert_int2_rtn( v ), (int2)(0), dim - 1 );
	return normal_map[ p.x + p.y * dim.x ];
}



__kernel void depth_to_color(__global float3 *points, __global uchar3 *color, __global uchar3 *mapbuf,
							 int2 ddim, int2 cdim, 
							 float4 Dintr, float4 Cintr, float16 M)
{
	int x = get_global_id(0);
	int y = get_global_id(1);
	if(x >= ddim.x || y >= ddim.y){ return; }
	int id = y * ddim.x + x;

	float3 p = points[id];
	// transform point from world space to color space
	float3 c_p = M.lo.lo.xyz * p.x + M.lo.hi.xyz * p.y + M.hi.lo.xyz * p.z + M.hi.hi.xyz;

	float2 xy = (float2)(c_p.x / c_p.z , c_p.y / c_p.z);
	
	int2 uv = convert_int2_rtp(xy * Cintr.lo + Cintr.hi - (float2)(0.5f, 0.5f));

	uchar3 co = (uchar3)(0, 0, 0);
	if(uv.x < 0 || uv.x >= cdim.x || uv.y < 0 || uv.y >= cdim.y){
	//if(cdim.x == 1920 && cdim.y == 1080){
		mapbuf[id] = co;
		return;
	}
	int cid = uv.y * cdim.x + uv.x;
	co = color[cid];
	mapbuf[id] = co;
}

__kernel void feature_matching()
{
	/* code */
}

