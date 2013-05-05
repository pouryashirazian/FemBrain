#include "clpp/clppSort_Blelloch.h"

#define TRANSPOSE // transpose the initial vector (faster memory access)

#pragma region Constructor

clppSort_Blelloch::clppSort_Blelloch(clppContext* context, unsigned int maxElements)
{
	nkeys = _N;
	nkeys_rounded = _N;

	if (!compile(context, "clppSort_Blelloch.cl")){
		return;
	}

	//---- Prepare all the kernels
	cl_int clStatus;

	_kernel_Histogram = clCreateKernel(_clProgram, "kernel_Histogram", &clStatus);
	checkCLStatus(clStatus);
	
	_kernel_ScanHistogram = clCreateKernel(_clProgram, "kernel_ScanHistograms", &clStatus);
	checkCLStatus(clStatus);

	_kernel_PasteHistogram = clCreateKernel(_clProgram, "kernel_PasteHistograms", &clStatus);
	checkCLStatus(clStatus);

	_kernel_Reorder = clCreateKernel(_clProgram, "kernel_Reorder", &clStatus);
	checkCLStatus(clStatus);

	_kernel_Transpose = clCreateKernel(_clProgram, "kernel_Transpose", &clStatus);
	checkCLStatus(clStatus);
}

string clppSort_Blelloch::compilePreprocess(string kernel)
{
	//---- Read the source code
	string definitions = "";
	char buffer[500];

	sprintf(buffer, "#define _ITEMS %d\n", _ITEMS);
	definitions += buffer;

	sprintf(buffer, "#define _GROUPS %d\n", _GROUPS);
	definitions += buffer;

	sprintf(buffer, "#define _BITS %d\n", _BITS);
	definitions += buffer;

#ifdef TRANSPOSE
	definitions += "#define TRANSPOSE\n";
#endif

#ifdef PERMUT
	definitions += "#define PERMUT\n";
#endif

	sprintf(buffer, "#define _RADIX %d\n", _RADIX);
	definitions += buffer;

	return definitions + kernel;
}

#pragma endregion

#pragma region sort

void clppSort_Blelloch::sort()
{
    //assert(nkeys_rounded <= _N);
    //assert(nkeys <= nkeys_rounded);

	//resize(1000);

    int nbcol = nkeys_rounded / (_GROUPS * _ITEMS);
    int nbrow = _GROUPS * _ITEMS;

#ifdef TRANSPOSE
		transpose(nbrow, nbcol);
#endif

	    for(unsigned int pass = 0; pass < _PASS; pass++)
    {
        histogram(pass);

        scanHistogram();
        reorder(pass);
    }

#ifdef TRANSPOSE
	transpose(nbcol, nbrow);
#endif

    _timerSort = _timerHisto + _timerScan + _timerReorder + _timerTranspose;
 }

#pragma endregion

#pragma region resize

// resize the sorted vector
void clppSort_Blelloch::resize(int nn)
{
    nkeys = nn;
    // length of the vector has to be divisible by (_GROUPS * _ITEMS)
    int remainder = nkeys % (_GROUPS * _ITEMS);
    nkeys_rounded = nkeys;
    cl_int clStatus;
    unsigned int pad[_GROUPS * _ITEMS];
    for (int ii = 0; ii < _GROUPS * _ITEMS; ii++)
        pad[ii] = _MAXINT - (unsigned int)1;

    if (remainder != 0)
    {
        nkeys_rounded = nkeys - remainder + (_GROUPS * _ITEMS);
        // pad the vector with big values
        assert(nkeys_rounded <= _N);
        clStatus = clEnqueueWriteBuffer(_context->clQueue, _clBuffer_dataSet, CL_TRUE, sizeof(int) * nkeys, sizeof(int) * (_GROUPS * _ITEMS - remainder), pad, 0, NULL, NULL);
        
        checkCLStatus(clStatus);
    }
}

#pragma endregion

#pragma region transpose

void clppSort_Blelloch::transpose(int nbrow,int nbcol)
{
    cl_int clStatus;

    clStatus  = clSetKernelArg(_kernel_Transpose, 0, sizeof(cl_mem), &_clBuffer_dataSet);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Transpose, 1, sizeof(cl_mem), &_clBuffer_outKeys);
    checkCLStatus(clStatus);

    clStatus = clSetKernelArg(_kernel_Transpose, 2, sizeof(int), &nbcol);
    checkCLStatus(clStatus);

    clStatus = clSetKernelArg(_kernel_Transpose, 3, sizeof(int), &nbrow);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Transpose, 4, sizeof(cl_mem), &_clBuffer_inPermutations);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Transpose, 5, sizeof(cl_mem), &_clBuffer_outPermutations);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Transpose, 6, sizeof(int)*_GROUPS*_GROUPS, NULL);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Transpose, 7, sizeof(int)*_GROUPS*_GROUPS, NULL);
    checkCLStatus(clStatus);

    cl_event eve;
    size_t global_work_size[2];
    size_t local_work_size[2];

    assert(nbrow%_GROUPS == 0);
    assert(nbcol%_GROUPS == 0);

    global_work_size[0]=nbrow/_GROUPS;
    global_work_size[1]=nbcol;

    local_work_size[0]=1;
    local_work_size[1]=_GROUPS;

	// two dimensions: rows and columns
	clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_Transpose, 2, NULL, global_work_size, local_work_size, 0, NULL, &eve);
	checkCLStatus(clStatus);

    // exchange the pointers

    // swap the old and new vectors of keys
    cl_mem _clBuffer_temp;
    _clBuffer_temp = _clBuffer_dataSet;
    _clBuffer_dataSet = _clBuffer_outKeys;
    _clBuffer_outKeys = _clBuffer_temp;

    // swap the old and new permutations
    _clBuffer_temp = _clBuffer_inPermutations;
    _clBuffer_inPermutations = _clBuffer_outPermutations;
    _clBuffer_outPermutations = _clBuffer_temp;

    // timing
    clFinish(_context->clQueue);

    cl_ulong beginning, end;

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), (void*)&beginning, NULL);
    checkCLStatus(clStatus);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), (void*)&end, NULL);
    checkCLStatus(clStatus);

	_timerTranspose += (float)(end-beginning)/1e9;
}

#pragma endregion

#pragma region histogram

void clppSort_Blelloch::histogram(int pass)
{
    cl_int clStatus;

    size_t nblocitems=_ITEMS;
    size_t nbitems=_GROUPS*_ITEMS;

    assert(_RADIX == pow(2.f,_BITS));

    clStatus  = clSetKernelArg(_kernel_Histogram, 0, sizeof(cl_mem), &_clBuffer_dataSet);
    checkCLStatus(clStatus);

    clStatus = clSetKernelArg(_kernel_Histogram, 2, sizeof(int), &pass);
    checkCLStatus(clStatus);

    assert(nkeys_rounded%(_GROUPS * _ITEMS) == 0);
    assert(nkeys_rounded <= _N);

    clStatus = clSetKernelArg(_kernel_Histogram, 4, sizeof(int), &nkeys_rounded);
    checkCLStatus(clStatus);

    cl_event eve;

    clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_Histogram, 1, NULL, &nbitems, &nblocitems, 0, NULL, &eve);

    //cout << clStatus<<" , "<<CL_OUT_OF_RESOURCES<<endl;
    checkCLStatus(clStatus);

    clFinish(_context->clQueue);

    cl_ulong beginning,end;

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), (void*) &beginning, NULL);
    checkCLStatus(clStatus);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), (void*) &end, NULL);
    checkCLStatus(clStatus);

    _timerHisto += (float)(end-beginning)/1e9;
}

#pragma endregion

#pragma region scanHistogram

void clppSort_Blelloch::scanHistogram()
{
    cl_int clStatus;

    // numbers of processors for the local scan
    // half the size of the local histograms
    size_t nbitems = _RADIX* _GROUPS*_ITEMS / 2;

    size_t nblocitems = nbitems/_HISTOSPLIT ;

    int maxmemcache=max(_HISTOSPLIT,_ITEMS * _GROUPS * _RADIX / _HISTOSPLIT);

    // scan locally the histogram (the histogram is split into several
    // parts that fit into the local memory)

    clStatus = clSetKernelArg(_kernel_ScanHistogram, 0, sizeof(cl_mem), &_clBuffer_Histograms);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_ScanHistogram, 1, sizeof(int)* maxmemcache , NULL); // mem cache

    clStatus = clSetKernelArg(_kernel_ScanHistogram, 2, sizeof(cl_mem), &_clBuffer_globsum);
    checkCLStatus(clStatus);

    cl_event eve;
    clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_ScanHistogram, 1, NULL, &nbitems, &nblocitems, 0, NULL, &eve);

    // cout << clStatus<<","<< CL_INVALID_WORK_ITEM_SIZE<< " "<<nbitems<<" "<<nblocitems<<endl;
    // cout <<CL_DEVICE_MAX_WORK_ITEM_SIZES<<endl;
    checkCLStatus(clStatus);
    clFinish(_context->clQueue);

    cl_ulong beginning,end;

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), (void*)&beginning, NULL);
    checkCLStatus(clStatus);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), (void*)&end, NULL);
    checkCLStatus(clStatus);

    _timerScan += (float)(end-beginning)/1e9;

    // second scan for the globsum
    clStatus = clSetKernelArg(_kernel_ScanHistogram, 0, sizeof(cl_mem), &_clBuffer_globsum);
    checkCLStatus(clStatus);

    clStatus = clSetKernelArg(_kernel_ScanHistogram, 2, sizeof(cl_mem), &_clBuffer_temp);
    checkCLStatus(clStatus);

    nbitems= _HISTOSPLIT / 2;
    nblocitems=nbitems;

    clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_ScanHistogram, 1, NULL, &nbitems, &nblocitems, 0, NULL, &eve);

    checkCLStatus(clStatus);
    clFinish(_context->clQueue);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), (void*) &beginning, NULL);
    checkCLStatus(clStatus);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), (void*)&end, NULL);
    checkCLStatus(clStatus);

    _timerScan += (float)(end-beginning)/1e9;

	// loops again in order to paste together the local histograms
    nbitems = _RADIX* _GROUPS*_ITEMS/2;
    nblocitems=nbitems/_HISTOSPLIT;

    clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_PasteHistogram, 1, NULL, &nbitems, &nblocitems, 0, NULL, &eve);

    checkCLStatus(clStatus);
    clFinish(_context->clQueue);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), (void*)&beginning, NULL);
    checkCLStatus(clStatus);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), (void*)&end, NULL);
    checkCLStatus(clStatus);

    _timerScan += (float)(end-beginning)/1e9;
}

#pragma endregion

#pragma region reorder

void clppSort_Blelloch::reorder(int pass)
{
    cl_int clStatus;

    size_t nblocitems=_ITEMS;
    size_t nbitems=_GROUPS*_ITEMS;


    clFinish(_context->clQueue);

    clStatus  = clSetKernelArg(_kernel_Reorder, 0, sizeof(cl_mem), &_clBuffer_dataSet);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Reorder, 1, sizeof(cl_mem), &_clBuffer_outKeys);
    checkCLStatus(clStatus);

    clStatus = clSetKernelArg(_kernel_Reorder, 3, sizeof(int), &pass);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Reorder, 4, sizeof(cl_mem), &_clBuffer_inPermutations);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Reorder, 5, sizeof(cl_mem), &_clBuffer_outPermutations);
    checkCLStatus(clStatus);

    clStatus  = clSetKernelArg(_kernel_Reorder, 6, sizeof(int)* _RADIX * _ITEMS , NULL); // mem cache
    checkCLStatus(clStatus);

    assert(nkeys_rounded%(_GROUPS * _ITEMS) == 0);

    clStatus = clSetKernelArg(_kernel_Reorder, 7, sizeof(int), &nkeys_rounded);
    checkCLStatus(clStatus);

    assert(_RADIX == pow(2.f, _BITS));

    cl_event eve;

    clStatus = clEnqueueNDRangeKernel(_context->clQueue, _kernel_Reorder, 1, NULL, &nbitems, &nblocitems, 0, NULL, &eve);

    checkCLStatus(clStatus);
    clFinish(_context->clQueue);

    cl_ulong beginning,end;

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_QUEUED, sizeof(cl_ulong), (void*) &beginning, NULL);
    checkCLStatus(clStatus);

    clStatus = clGetEventProfilingInfo(eve, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), (void*) &end, NULL);
    checkCLStatus(clStatus);

    _timerReorder += (float)(end-beginning)/1e9;

    // swap the old and new vectors of keys
    cl_mem _clBuffer_temp;
    _clBuffer_temp = _clBuffer_dataSet;
    _clBuffer_dataSet = _clBuffer_outKeys;
    _clBuffer_outKeys = _clBuffer_temp;

    // swap the old and new permutations
    _clBuffer_temp = _clBuffer_inPermutations;
    _clBuffer_inPermutations = _clBuffer_outPermutations;
    _clBuffer_outPermutations = _clBuffer_temp;
}

#pragma endregion

#pragma region pushDatas

void clppSort_Blelloch::pushCLDatas(cl_mem clBuffer_dataSet, size_t datasetSize)
{
	cl_int clStatus;

	//---- Construction of the initial permutation
	for(size_t i = 0; i < _N; i++)
		_permutations[i] = i;

	//---- Create the buffers
	_clBuffer_outKeys  = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * _N, NULL, &clStatus);
	checkCLStatus(clStatus);

	_clBuffer_inPermutations = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * _N, NULL, &clStatus);
	checkCLStatus(clStatus);

	_clBuffer_outPermutations = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * _N, NULL, &clStatus);
	checkCLStatus(clStatus);

	// copy on the device
	_clBuffer_Histograms  = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * _RADIX * _GROUPS * _ITEMS, NULL, &clStatus);
	checkCLStatus(clStatus);

	// copy on the device
	_clBuffer_globsum  = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * _HISTOSPLIT, NULL, &clStatus);
	checkCLStatus(clStatus);

	// temporary vector when the sum is not needed
	_clBuffer_temp  = clCreateBuffer(_context->clContext, CL_MEM_READ_WRITE, sizeof(int) * _HISTOSPLIT, NULL, &clStatus);
	checkCLStatus(clStatus);

	
	resize(datasetSize);
	//cout <<"nkeys="<<nkeys<<" "<<nkeys_rounded<<endl;

	//---- Send the data
	if (_dataSet != 0)
	{
		clStatus = clEnqueueWriteBuffer(_context->clQueue, clBuffer_dataSet, CL_FALSE, 0, sizeof(int) * _N, _dataSet, 0, NULL, NULL);
		checkCLStatus(clStatus);
	}

	clStatus = clEnqueueWriteBuffer(_context->clQueue, _clBuffer_inPermutations, CL_FALSE, 0, sizeof(int) * _N, _permutations, 0, NULL, NULL);
	checkCLStatus(clStatus);

	//---- We set here the fixed arguments of the OpenCL kernels
	clStatus = clSetKernelArg(_kernel_Histogram, 1, sizeof(cl_mem), &_clBuffer_Histograms);
	checkCLStatus(clStatus);

	clStatus = clSetKernelArg(_kernel_Histogram, 3, sizeof(int)*_RADIX*_ITEMS, NULL);
	checkCLStatus(clStatus);

	clStatus = clSetKernelArg(_kernel_PasteHistogram, 0, sizeof(cl_mem), &_clBuffer_Histograms);
	checkCLStatus(clStatus);

	clStatus = clSetKernelArg(_kernel_PasteHistogram, 1, sizeof(cl_mem), &_clBuffer_globsum);
	checkCLStatus(clStatus);

	clStatus = clSetKernelArg(_kernel_Reorder, 2, sizeof(cl_mem), &_clBuffer_Histograms);
	checkCLStatus(clStatus);

	clStatus  = clSetKernelArg(_kernel_Reorder, 6, sizeof(int) * _RADIX * _ITEMS , NULL); // mem cache
	checkCLStatus(clStatus);
}

#pragma endregion

#pragma region popDatas

void clppSort_Blelloch::popDatas()
{
    cl_int clStatus;

    clFinish(_context->clQueue);     // wait end of read

	clStatus = clEnqueueReadBuffer(_context->clQueue, _clBuffer_dataSet, CL_FALSE, 0, sizeof(int) * _N, _dataSet, 0, NULL, NULL);
	checkCLStatus(clStatus);

	/*
    clFinish(_context->clQueue);     // wait end of read

    status = clEnqueueReadBuffer(_context->clQueue, _clBuffer_inPermutations, CL_TRUE, 0, sizeof(int) * _N, h_Permut, 0, NULL, NULL);
	checkCLStatus(clStatus);

    clFinish(_context->clQueue);     // wait end of read

    status = clEnqueueReadBuffer(_context->clQueue, _clBuffer_Histograms, CL_TRUE, 0, sizeof(int) * _RADIX * _GROUPS * _ITEMS, h_Histograms, 0, NULL, NULL);
    checkCLStatus(clStatus);

    status = clEnqueueReadBuffer(_context->clQueue, _clBuffer_globsum, CL_TRUE, 0, sizeof(int) * _HISTOSPLIT, h_globsum, 0, NULL, NULL);
    checkCLStatus(clStatus);
	*/

    clFinish(_context->clQueue);     // wait end of read
}

#pragma endregion
