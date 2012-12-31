#ifndef __CLPP_CONTEXT_H__
#define __CLPP_CONTEXT_H__

#if defined (__APPLE__) || defined(MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif

enum clppVendor { Vendor_Unknown, Vendor_NVidia, Vendor_AMD, Vendor_Intel };

class clppContext
{
public:
	cl_context clContext;			// OpenCL context
	cl_platform_id clPlatform;		// OpenCL Platform
	cl_device_id clDevice;			// OpenCL Device
	cl_command_queue clQueue;		// OpenCL command queue 

	// Default setup : use the default platform and default device
	void setup();

	// Setup with a specific platform and device
	void setup(unsigned int platformId, unsigned int deviceId);

	// Returns the SIMT Capability of the device.
	int GetSIMTCapability();

	// Informations
	bool isGPU;
	bool isCPU;
	clppVendor Vendor;

private:
	// Case-insensitive strstr() work-alike.
	static char* stristr(const char *String, const char *Pattern);
};

#endif
