#include "clpp/clppSort_CPU.h"

#include <algorithm>

#pragma region Construsctor

clppSort_CPU::clppSort_CPU(clppContext* context)
{
}

clppSort_CPU::~clppSort_CPU()
{
}

#pragma endregion

#pragma region sort

void clppSort_CPU::sort()
{
	//std::sort((char*)_keys, (char*)_keys + _datasetSize * (_keyBits/8));
	std::sort((int*)_dataSet, ((int*)_dataSet) + _datasetSize);
}

#pragma endregion

#pragma region pushDatas

void clppSort_CPU::pushDatas(void* dataSet, size_t datasetSize)
{
	_keySize = _valueSize = 4;
	_dataSet = dataSet;
	_datasetSize = datasetSize;
}

void clppSort_CPU::pushCLDatas(cl_mem clBuffer_dataSet, size_t datasetSize)
{
	// Unsupported of course !
}

#pragma endregion

#pragma region popDatas

void clppSort_CPU::popDatas()
{
}

#pragma endregion