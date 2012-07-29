#define ZERO4 {0.0f, 0.0f, 0.0f, 0.0f}

//Kernel Function to compute distances to sphere at origin 
__kernel void distSphere(__global float* arrPosX, 
						 __global float* arrPosY, 
						 __global float* arrPosZ, 
						 __global float* arrDist,
						 const unsigned int count)
{
	float4 v = ZERO4;
	int idx = get_global_id(0);
	if(idx < count)
	{	
		v.x = arrPosX[idx];
		v.y = arrPosY[idx];
		v.z = arrPosZ[idx];		
		arrDist[idx] = dot(v, v);
		//arrDist[idx] = arrPosX[idx] * arrPosX[idx] + arrPosY[idx] * arrPosY[idx] + arrPosZ[idx] * arrPosZ[idx];
	}	
};