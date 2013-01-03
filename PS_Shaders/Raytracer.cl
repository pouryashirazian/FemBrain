/*!
*	RayTracer Kernel
*
*/
__kernel void RayTrace(__write_only image2d_t texOutRenderBuffer)
{
	int2 dim = (int2)(128, 128);
	int2 idx = (int2)(get_global_id(0), get_global_id(1));
		
	if((idx.x > dim.x)||(idx.y > dim.y))
		return;

	float3 lower = (float3)(-2.0f, -2.0f, -2.0f);
	float3 upper = (float3)(2.0f, 2.0f, 2.0f);
	
	//Ray Origin
	float3 e = (float3)(0.0f, 0.0f, -10.0f);
	
	float3 s;
	s.x = lower.x + ((upper.x - lower.x) * ((float)idx.x + 0.5f)) / (float)dim.x;
	s.y = lower.y + ((upper.y - lower.y) * ((float)idx.y + 0.5f)) / (float)dim.y;
	s.z = lower.z;
	
	//printf("%.3f, %.3f, %.3f \n", s.y, s.y, s.y); 
	
	
	//Ray Direction
	float3 d = normalize(s - e);
	
	//circle center
	float3 c = (float3)(0.0f, 0.0f, 0.0f);
	float radius = 1.0f;
	
	float A = dot(d, d);
	float B = 2.0f * dot(d, e - c);
	float C = dot(e - c, e - c) - radius * radius;
	float delta = B*B - 4.0f * A * C;
	
	float4 clFinal = (float4)(0.5f, 0.5f, 0.5f, 1.0f);
	
	printf("DELTA = %.f \n", delta);
	
	if(delta >= 0.0f)
	{
		float tt = ( - B + sqrt(delta) ) / (2.0f * A);
		if(tt >= lower.z && tt <= upper.z)
			clFinal = (float4)(1.0f, 0.0f, 0.0f, 1.0f);
		else
		{
			tt = (- B - sqrt(delta)) / (2.0f * A);
			if(tt >= lower.z && tt <= upper.z)
				clFinal = (float4)(1.0f, 0.0f, 0.0f, 1.0f);
		}
	}
	
	
	write_imagef(texOutRenderBuffer, idx, clFinal); 
	
}
