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
__kernel void particleMove(__global float4* arrMeshVertex,
						   __global float4* arrMeshColor,
						   __global float4* arrPos,
						   __global float4* arrVel,
						   __global float4* arrColor,
						   const unsigned int count,
						   const float deltaT)
{
	int idx = get_global_id(0);
	if(idx < count)
	{			
		float4 v = arrVel[idx];
		float4 p = arrPos[idx] + deltaT * v;

		if(p.x > 1.0f)
		{
			float4 xMinus = XMINUS;
			v = v + 2 * dot(-v, xMinus);
		}
		else if(p.x < -1.0f)
		{
			float4 xPlus = XPLUS;
			v = v + 2 * dot(-v, xPlus);
		}
	
		if(p.y > 1.0f)
		{
			float4 yMinus = YMINUS;
			v = v + 2 * dot(-v, yMinus);
		}
		else if(p.y < -1.0f)
		{
			float4 yPlus = YPLUS;
			v = v + 2 * dot(-v, yPlus);
		}

		if(p.z > 1.0f)
		{	
			float4 zMinus = ZMINUS;
			v = v + 2 * dot(-v, zMinus);
		}
		else if(p.z < -1.0f)
		{
			float4 zPlus = ZPLUS;
			v = v + 2 * dot(-v, zPlus);
		}

		float4 red = RED;
		arrVel[idx] = v;
		arrColor[idx] = red;
		arrMeshVertex[idx] = arrPos[idx];
		arrMeshColor[idx] = arrColor[idx];
	}	
};