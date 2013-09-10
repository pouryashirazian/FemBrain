// TODO: Add OpenCL kernel code here.
#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}
#define XPLUS {1.0f, 0.0f, 0.0f, 0.0f}
#define XMINUS {-1.0f, 0.0f, 0.0f, 0.0f}

#define YPLUS {0.0f, 1.0f, 0.0f, 0.0f}
#define YMINUS {0.0f, -1.0f, 0.0f, 0.0f}

#define ZPLUS {0.0f, 0.0f, 1.0f, 0.0f}
#define ZMINUS {0.0f, 0.0f, -1.0f, 0.0f}
#define RED {1.0f, 0.0f, 0.0f, 1.0f}

//Kernel Function to compute distances to sphere at origin 
__kernel void particleMove(__global float4* arrInOutPosition,
						   __global float4* arrInOutVelocity,
						   __global float4* arrInMeshColor,
						   const unsigned int count,
						   const float deltaT)
{
	int idx = get_global_id(0);
	if(idx >= count)
		return;
	
	float4 p = arrInOutPosition[idx];
	float4 v = arrInOutVelocity[idx];
	p = p + deltaT * v;
	p.w = 1.0f;
	
	if(isgreater(p.x, 1.0f))
		v = v * -1.0f;
	else if(isless(p.x, -1.0f))
		v = v * -1.0f;
		
	if(isgreater(p.y, 1.0f))
		v = v * -1.0f;
	else if(isless(p.y, -1.0f))
		v = v * -1.0f;

	if(isgreater(p.z, 1.0f))
		v = v * -1.0f;
	else if(isless(p.z, -1.0f))
		v = v * -1.0f;
	v.w = 1.0f;
			
	arrInOutPosition[idx] = p;
	arrInOutVelocity[idx] = v;
}