/*
 * PS_PolygonizerScalarImpl.cpp
 *
 *  Created on: 2011-11-30
 *      Author: pourya
 */
#include "PS_Polygonizer.h"
#include "_CellConfigTable.h"

namespace PS{
namespace SIMDPOLY{
/*
void CMPUProcessor::process_cells_continuation(const FieldComputer& fc, MPU& mpu) const
{
	//Discard checks weather this partition is empty or not.
	if(this->discard(fc, mpu))
		return;

	//Create Field-Value Cache and init it
	//const size_t m = GRID_DIM * GRID_DIM * PS_SIMD_BLOCKS(GRID_DIM);
	const int cellsPerSide = GRID_DIM - 1;
	const float mpuSide = cellsPerSide * m_cellsize;
	const float halfSide = 0.5f * mpuSide;
	//const U32 m = GRID_DIM * GRID_DIM * GRID_DIM;
	//float PS_SIMD_ALIGN(fvCache[m]);

	//Cells Processed and storage data
	const int ctCells = cellsPerSide * cellsPerSide * cellsPerSide;
	U8 arrProcessed[cellsPerSide][cellsPerSide][cellsPerSide];
	memset(arrProcessed, 0, ctCells);
	GENERIC_POLYMORPHIC_STACK<svec3i, ctCells> stkCells;
	GENERIC_POLYMORPHIC_STACK<svec3f, ctCells> stkSeeds;

	//EdgeTable
	EDGETABLE edgeTable;
	memset(&edgeTable, 0, sizeof(EDGETABLE));
	mpu.ctFieldEvals = 0;
	mpu.ctVertices = 0;
	mpu.ctTriangles = 0;

	//Find a seed point
	svec3f lo = mpu.bboxLo;
	svec3f hi = vadd3f(lo, svec3f(mpuSide, mpuSide, mpuSide));
	svec3f seed;

	//Try Shooting rays if no seeds found
	float PS_SIMD_ALIGN(arrX[8]);
	float PS_SIMD_ALIGN(arrY[8]);
	float PS_SIMD_ALIGN(arrZ[8]);
	float PS_SIMD_ALIGN(arrS[8]);
	for(int i=0; i<8; i++)
	{
		arrX[i] = (float)((0xF0 >> i) & 1);
		arrY[i] = (float)((0xCC >> i) & 1);
		arrZ[i] = (float)((0xAA >> i) & 1);
		arrS[i] = i;
	}


	//Shoot Rays
	{
	#ifdef SIMD_USE_M128
		//My SIMD VARS
		svec3f c = vadd3f(lo, svec3f(halfSide, halfSide, halfSide));
		Float_ rootResolution = Float_(arrS) * Float_(1.0f / 3.0f);
		Float_ cX(c.x);
		Float_ cY(c.y);
		Float_ cZ(c.z);
		Float_ hs(halfSide);

		//Shoot to MPU corners [8]
		for(int i=0; i<8; i++)
		{
			float fx = halfSide * arrX[i] - halfSide * (1 - arrX[i]);
			float fy = halfSide * arrY[i] - halfSide * (1 - arrY[i]);
			float fz = halfSide * arrZ[i] - halfSide * (1 - arrZ[i]);
			{
				Float_ posX = cX + Float_(fx) * rootResolution;
				Float_ posY = cY + Float_(fy) * rootResolution;
				Float_ posZ = cZ + Float_(fz) * rootResolution;

				if(fc.computeRootSimd(posX, posY, posZ, seed))
				{
					//bGotSeed = true;
					stkSeeds.push(seed);
					//break;
				}
			}
		}

		//Shoot to MPU FACE-MIDS [6]
		{
			//if(!bGotSeed) //X
			{
				Float_ posXP = cX + hs * rootResolution;
				Float_ posXN = cX - hs * rootResolution;
				if(fc.computeRootSimd(posXP, cY, cZ, seed))
					stkSeeds.push(seed);
				if(fc.computeRootSimd(posXN, cY, cZ, seed))
					stkSeeds.push(seed);
			}
			//if(!bGotSeed) //Y
			{
				Float_ posYP = cY + hs * rootResolution;
				Float_ posYN = cY - hs * rootResolution;
				if(fc.computeRootSimd(cX, posYP, cZ, seed))
					stkSeeds.push(seed);
				if(fc.computeRootSimd(cX, posYN, cZ, seed))
					stkSeeds.push(seed);
			}
			//if(!bGotSeed) //Z
			{
				Float_ posZP = cZ + hs * rootResolution;
				Float_ posZN = cZ - hs * rootResolution;
				if(fc.computeRootSimd(cX, cY, posZP, seed))
					stkSeeds.push(seed);
				if(fc.computeRootSimd(cX, cY, posZN, seed))
					stkSeeds.push(seed);
			}

			//Search Primitive Seeds
			if(stkSeeds.empty())
			{
				//If number of primitives are really large this linear search will take too long
				//Use Binary Search instead
				U32 ctPrims = fc.m_blobPrims.count;
				for(U32 i =0; i < ctPrims; i++)
				{
					seed = svec3f(fc.m_blobPrims.seedX[i], fc.m_blobPrims.seedY[i], fc.m_blobPrims.seedZ[i]);

					if((seed.x >= lo.x)&&(seed.x <= hi.x)&&
					   (seed.y >= lo.y)&&(seed.y <= hi.y)&&
					   (seed.z >= lo.z)&&(seed.z <= hi.z))
					{
						//bGotSeed = true;
						stkSeeds.push(seed);
						break;
					}
				}
			}
		}

	#elif defined(SIMD_USE_M256)
		Float_ X(&arrX[0]);
		Float_ Y(&arrY[0]);
		Float_ Z(&arrZ[0]);
		Float_ zero(0.0f);
		Float_ one(1.0f);
		Float_ side(mpuSide);
	#endif

	}

	//Use all the seeds we found
	while(!stkSeeds.empty())
	{
		seed = stkSeeds.top();
		stkSeeds.pop();

		svec3i idx;
		idx.x = (int)floor((seed.x - lo.x) / m_cellsize);
		idx.y = (int)floor((seed.y - lo.y) / m_cellsize);
		idx.z = (int)floor((seed.z - lo.z) / m_cellsize);

		if((idx.x < 0)||(idx.x > (GRID_DIM -2)))
			continue;
		if((idx.y < 0)||(idx.y > (GRID_DIM -2)))
			continue;
		if((idx.z < 0)||(idx.z > (GRID_DIM -2)))
			continue;

		//Push onto stack
		stkCells.push(idx);

		float PS_SIMD_ALIGN(arrPowerMask[8]);
		arrPowerMask[0] = 1.0f;
		for(int iSimd=1; iSimd<8; iSimd++)
			arrPowerMask[iSimd] = 2.0f * arrPowerMask[iSimd - 1];
		int arrConfig[16];
		int idxCellConfig = 0;

		Float_ arrIsoVal(ISO_VALUE);
		Float_ one(1.0f);
		Float_ cs(m_cellsize);

		float PS_SIMD_ALIGN(arrFields[8]);

	#ifdef SIMD_USE_M128
		Float_ maskPowerLeft(&arrPowerMask[0]);
		Float_ maskPowerRight(&arrPowerMask[4]);
		Float_ rootResolution = Float_(arrS) * Float_(1.0f / 3.0f);
	#elif defined(SIMD_USE_M256)
		Float_ maskPower(&arrPowerMask[0]);
		Float_ rootResolution = Float_(arrS) * Float_(1.0f / 7.0f);
	#endif

		//While there is a cell to process
		while(!stkCells.empty())
		{
			//Fetch and Mark
			svec3i idxCell = stkCells.top();
			svec3f cellLo = vadd3f(mpu.bboxLo, svec3f(idxCell.x * m_cellsize, idxCell.y * m_cellsize, idxCell.z * m_cellsize));

			arrProcessed[idxCell.x][idxCell.y][idxCell.z] = 1;
			stkCells.pop();

		#ifdef SIMD_USE_M128
			//Compute Cell Fields
			Float_ X1 = Float_(cellLo.x) + Float_(&arrX[0]) * cs;
			Float_ X2 = Float_(cellLo.x) + Float_(&arrX[4]) * cs;
			Float_ Y  = Float_(cellLo.y) + Float_(&arrY[0]) * cs;
			Float_ Z = Float_(cellLo.z) + Float_(&arrZ[0]) * cs;
			Float_ F1;
			Float_ F2;

			fc.fieldValue(X1, Y, Z, F1);
			fc.fieldValue(X2, Y, Z, F2);

			//Store
			F1.store(&arrFields[0]);
			F2.store(&arrFields[4]);

			//Compute configuration
			Float_ Left = SimdAnd(SimdGTE(F1, arrIsoVal), one) * maskPowerLeft;
			Float_ Right = SimdAnd(SimdGTE(F2, arrIsoVal), one) * maskPowerRight;

			//3 Levels of HADDS Needed
			Left = SimdhAdd(Left, Right);
			Left = SimdhAdd(Left, Left);
			Left = SimdhAdd(Left, Left);
			idxCellConfig = static_cast<int>(Left[0]);
		#elif defined(SIMD_USE_M256)
			Float_ X = Float_(cellLo.x) + Float_(&arrX[0]) * cs;
			Float_ Y = Float_(cellLo.y) + Float_(&arrY[0]) * cs;
			Float_ Z = Float_(cellLo.z) + Float_(&arrZ[0]) * cs;
			Float_ F;

			//Compute Field for cell
			fc.fieldValue(X, Y, Z, F);

			//Store
			F.store(&arrFields[0]);

			Float_ Left = SimdAnd(SimdGTE(F, arrIsoVal), one) * maskPower;
			Left = SimdhAdd(Left, Left);
			Left = SimdhAdd(Left, Left);
			idxCellConfig = static_cast<int>(Left[0] + Left[4]);
		#endif
			if(idxCellConfig == 0 || idxCellConfig == 255)
				continue;

			int arrFaces[6];
			int axis;
			fc.getCrossedFaces(arrFields, arrFaces);
			for(int i=0; i<6; i++)
			{
				if(arrFaces[i])
				{
					axis = i / 2;
					svec3i nxtAdr = idxCell;
					int v = velement3i(nxtAdr, axis);

					//if negative
					if(i % 2)
					{
						if(v == 0)
						  continue;
						vsetElement3i(nxtAdr, axis, v - 1);
					}
					else
					{
						if(v == GRID_DIM - 2)
						  continue;
						vsetElement3i(nxtAdr, axis, velement3i(nxtAdr, axis) + 1);
					}

					if(!arrProcessed[nxtAdr.x][nxtAdr.y][nxtAdr.z])
						stkCells.push(nxtAdr);
				}
			}


			//Fetch cell configuration
			memcpy(arrConfig, &g_triTableCache[idxCellConfig][0], 16 * sizeof(int));
			//Increment number of processed cells
			//Compute surface vertex, normal and material on each crossing edge
			int idxMeshVertex[16];
			//Read case
			int ctTotalPolygons = 0;
			int ctEdges = 0;

			for(int icase=0; icase<16; icase++)
			{
				int candidate = arrConfig[icase];
				if(candidate != -1)
				{
					int idxEdgeStart = corner1[candidate];
					int idxEdgeAxis  = edgeaxis[candidate];

					//2 - idxEdgeAxis
					//Compute indices
					int sx = idxCell.x + ((idxEdgeStart & 4) >> 2);
					int sy = idxCell.y + ((idxEdgeStart & 2) >> 1);
					int sz = idxCell.z + (idxEdgeStart & 1);

					idxMeshVertex[icase] = getEdge(edgeTable, sx, sy, sz, idxEdgeAxis);

					//See if the vertex exist in edge table. If it doesn't exist compute and add it to edge table
					if(idxMeshVertex[icase] == -1)
					{
						//Reduced operations for edge processing
						svec3f e1 = vadd3f(mpu.bboxLo, svec3f(m_cellsize * sx, m_cellsize * sy, m_cellsize * sz));
						svec3f e2 = e1;
						vsetElement3f(e2, idxEdgeAxis, velement3f(e2, idxEdgeAxis) + m_cellsize);

						Float_ rootFields;
						Float_ posX(e1.x);
						Float_ posY(e1.y);
						Float_ posZ(e1.z);
						{
							Float_ displaceX(e2.x - e1.x);
							Float_ displaceY(e2.y - e1.y);
							Float_ displaceZ(e2.z - e1.z);

							posX = posX + displaceX * rootResolution;
							posY = posY + displaceY * rootResolution;
							posZ = posZ + displaceZ * rootResolution;
						}

						svec3f p;
						//Compute SIMD root
						fc.computeRootSimd(posX, posY, posZ, p);

						Float_ cellCornerPosX_(p.x);
						Float_ cellCornerPosY_(p.y);
						Float_ cellCornerPosZ_(p.z);
						Float_ vtxField;
						Float_ outColorX;
						Float_ outColorY;
						Float_ outColorZ;

						Float_ outNormalX;
						Float_ outNormalY;
						Float_ outNormalZ;

						//Compute Field and Color
						fc.fieldValueAndColor(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_,
											  vtxField, outColorX, outColorY, outColorZ);

						//Use computed field to get the normal
						fc.normal(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_, vtxField,
								  outNormalX, outNormalY, outNormalZ, NORMAL_DELTA);


						U16 idxVertex = mpu.ctVertices;
						U16 idxVertex_X = idxVertex * 3;
						U16 idxVertex_Y = idxVertex * 3 + 1;
						U16 idxVertex_Z = idxVertex * 3 + 2;

						mpu.vPos[idxVertex_X]   = p.x;
						mpu.vPos[idxVertex_Y]   = p.y;
						mpu.vPos[idxVertex_Z]   = p.z;

						mpu.vNorm[idxVertex_X]  = outNormalX[0];
						mpu.vNorm[idxVertex_Y]  = outNormalY[0];
						mpu.vNorm[idxVertex_Z]  = outNormalZ[0];

						mpu.vColor[idxVertex_X] = outColorX[0];
						mpu.vColor[idxVertex_Y] = outColorY[0];
						mpu.vColor[idxVertex_Z] = outColorZ[0];

						mpu.ctVertices++;


						//Get vertex v index from list. It is the last one
						idxMeshVertex[icase] = idxVertex;
						setEdge(edgeTable, sx, sy, sz, idxEdgeAxis, idxVertex);
					}

					ctEdges++;
				}
				else
					break;
			}//End icase

			//Number of polygons
			ctTotalPolygons = ctEdges / 3;
			for(int icase = 0; icase < ctTotalPolygons; icase++)
			{
				int idxTriangle = mpu.ctTriangles;
				mpu.triangles[idxTriangle * 3 + 0] = idxMeshVertex[icase*3 + 0];
				mpu.triangles[idxTriangle * 3 + 1] = idxMeshVertex[icase*3 + 1];
				mpu.triangles[idxTriangle * 3 + 2] = idxMeshVertex[icase*3 + 2];
				mpu.ctTriangles++;
			}
		}//WHILE STKCELLS
	}//WHILE STKSEEDS
}


void CMPUProcessor::process_cells_fieldsimd_cellparallel_oldedgeAccess(const FieldComputer& fc, MPU& mpu, tbb::tick_count& tickFieldEvals) const
{
	//Create Field-Value Cache and init it
	//const size_t m = GRID_DIM * GRID_DIM * PS_SIMD_BLOCKS(GRID_DIM);
	const size_t m = GRID_DIM * GRID_DIM * GRID_DIM;
	float PS_SIMD_ALIGN(fvCache[m]);

	//EdgeTable
	EDGETABLE edgeTable;
	memset(&edgeTable, 0, sizeof(EDGETABLE));
	int idxCellConfig = 0;

	mpu.ctFieldEvals = 0;
	mpu.ctVertices = 0;
	mpu.ctTriangles = 0;

	{
		Float_ arrInside(0.0f);
		Float_ arrOutside(0.0f);
		Float_ arrIsoVal(ISO_VALUE);
		Float_ one(1.0f);

		//Cache all field-values with in this MPU
		int kMax = PS_SIMD_BLOCKS(GRID_DIM);
		for(int i=0; i<GRID_DIM; i++)
		{
			for(int j=0; j<GRID_DIM; j++)
			{
				for(int k=0; k<kMax; k++)
				{
					Float_ cellCornerPOSX_(mpu.bboxLo.x + i*m_cellsize);
					Float_ cellCornerPOSY_(mpu.bboxLo.y + j*m_cellsize);

					float PS_SIMD_ALIGN(arrPosZ[PS_SIMD_FLEN]);
					for(int iSimd=0; iSimd<PS_SIMD_FLEN; iSimd++)
						arrPosZ[iSimd] = mpu.bboxLo.z + ((k*PS_SIMD_FLEN) + iSimd)*m_cellsize;

					Float_ cellCornerPOSZ_(arrPosZ);
					Float_ arrField;

					//FieldValue computed using SIMD
					fc.fieldValue(cellCornerPOSX_, cellCornerPOSY_, cellCornerPOSZ_, arrField);

					//arrField.store(&fvCache[CELLID_FROM_IDX(i,j, k*PS_SIMD_FLEN)]);
					for(int iSimd=0; iSimd<PS_SIMD_FLEN; iSimd++)
					{
						fvCache[CELLID_FROM_IDX(i,j, (k*PS_SIMD_FLEN) + iSimd)] = arrField[iSimd];
					}


					//Keep stats
					mpu.ctFieldEvals++;
					cellCornerPOSX_ = SimdAnd(SimdGTE(arrField, arrIsoVal), one);
					arrInside = arrInside + cellCornerPOSX_;
					arrOutside = arrOutside + one - cellCornerPOSX_;
				}
			}
		}

		//Early Discard Shaved 2 seconds off
		VecNMask mask1(arrInside.v);
		if(mask1 == PS_SIMD_ALLZERO)
			return;

		mask1 = VecNMask(arrOutside.v);
		if(mask1 == PS_SIMD_ALLZERO)
			return;

	}

	//Get Tick Count for FieldEvals
	tickFieldEvals = tbb::tick_count::now();

	/////////////////////////////////////////////////////////////////////////
	//Process all cells in this MPU
	for(int i=0; i<GRID_DIM-1; i++)
	{
		for(int j=0; j<GRID_DIM-1; j++)
		{
			for(int k=0; k<GRID_DIM-1; k++)
			{
				int	cellCornerKey[8];
				svec3i cellCornerIDX[8];
				float cellCornerField[8];

				cellCornerKey[0] = CELLID_FROM_IDX(i, j, k);
				cellCornerIDX[0] = svec3i(i, j, k);

				cellCornerKey[1] = CELLID_FROM_IDX(i, j, k+1);
				cellCornerIDX[1] = svec3i(i, j, k+1);

				cellCornerKey[2] = CELLID_FROM_IDX(i, j+1, k);
				cellCornerIDX[2] = svec3i(i, j+1, k);

				cellCornerKey[3] = CELLID_FROM_IDX(i, j+1, k+1);
				cellCornerIDX[3] = svec3i(i, j+1, k+1);

				cellCornerKey[4] = CELLID_FROM_IDX(i+1, j, k);
				cellCornerIDX[4] = svec3i(i+1, j, k);

				cellCornerKey[5] = CELLID_FROM_IDX(i+1, j, k+1);
				cellCornerIDX[5] = svec3i(i+1, j, k+1);

				cellCornerKey[6] = CELLID_FROM_IDX(i+1, j+1, k);
				cellCornerIDX[6] = svec3i(i+1, j+1, k);

				cellCornerKey[7] = CELLID_FROM_IDX(i+1, j+1, k+1);
				cellCornerIDX[7] = svec3i(i+1, j+1, k+1);

				//Compute Cell Config
				idxCellConfig = 0;
				for(int icase=0; icase<8; icase++)
				{
					cellCornerField[icase] = fvCache[cellCornerKey[icase]];
					if(cellCornerField[icase] > ISO_VALUE)
						idxCellConfig += (1 << icase);
				}

				//Cell Configuration
				if((idxCellConfig != 0)&&(idxCellConfig != 255))
				{
					//Increment number of processed cells
					//ctIntersectedCells++;
					//Compute surface vertex, normal and material on each crossing edge
					int idxMeshVertex[16];
					svec3i idxMissingEdge[16];

					//Read case
					int ctTotalPolygons = 0;
					int ctEdges = 0;
					int ctMissing = 0;

					for(int icase=0; icase<16; icase++)
					{
						int candidate = g_triTableCache[idxCellConfig][icase];
						if(candidate != -1)
						{
							int idxEdgeStart = corner1[candidate];
							int idxEdgeEnd 	 = corner2[candidate];
							idxMeshVertex[icase] = getEdge(edgeTable,
									cellCornerIDX[idxEdgeStart],
									cellCornerIDX[idxEdgeEnd]);

							//See if the vertex exist in edge table. If it doesn't exist compute and add it to edge table
							if(idxMeshVertex[icase] == -1)
							{
								//Start + End + Position in array
								idxMissingEdge[ctMissing] = svec3i(idxEdgeStart, idxEdgeEnd, icase);
								ctMissing++;
							}
							ctEdges++;
						}
						else
							break;
					}
					ctTotalPolygons = ctEdges / 3;


					//Get all missing edge vertices in one array
					float PS_SIMD_ALIGN(arrPosX[PS_SIMD_FLEN]);
					float PS_SIMD_ALIGN(arrPosY[PS_SIMD_FLEN]);
					float PS_SIMD_ALIGN(arrPosZ[PS_SIMD_FLEN]);

					int szEdgeBlocks = PS_SIMD_BLOCKS(ctMissing);

					for(int iEdgeBlk = 0; iEdgeBlk < szEdgeBlocks; iEdgeBlk++)
					{
						int idxSIMDLine = 0;
						int idxEdge = iEdgeBlk * PS_SIMD_FLEN;

						//Count edges that are processed in SIMD (Must be less then SIMD_FLEN)
						int ctProcessedEdges = 0;

						while(idxSIMDLine < PS_SIMD_FLEN)
						{
							if(idxEdge < ctMissing)
							{
								U8 idxEdgeStart  = idxMissingEdge[idxEdge].x;
								U8 idxEdgeEnd 	 = idxMissingEdge[idxEdge].y;

								//Current Root Finding method is interpolation
								//TODO: change this to Newton-Raphson for better quality root that are marched towards the surface
								//float scale = (ISO_VALUE - cellCornerField[idxEdgeStart])/(cellCornerField[idxEdgeEnd] - cellCornerField[idxEdgeStart]);
								svec3f e1 = vadd3f(mpu.bboxLo, svec3f(m_cellsize * cellCornerIDX[idxEdgeStart].x, m_cellsize * cellCornerIDX[idxEdgeStart].y, m_cellsize * cellCornerIDX[idxEdgeStart].z));
								svec3f e2 = vadd3f(mpu.bboxLo, svec3f(m_cellsize * cellCornerIDX[idxEdgeEnd].x, m_cellsize * cellCornerIDX[idxEdgeEnd].y, m_cellsize * cellCornerIDX[idxEdgeEnd].z));
								//svec3f p = vadd3f(e1, vscale3f(scale, vsub3f(e2, e1)));
								svec3f p;
								float fp;
								fc.computeRootNewtonRaphson(e1, e2, cellCornerField[idxEdgeStart], cellCornerField[idxEdgeEnd], p, fp, ISO_VALUE, 5);

								arrPosX[idxSIMDLine] = p.x;
								arrPosY[idxSIMDLine] = p.y;
								arrPosZ[idxSIMDLine] = p.z;
								ctProcessedEdges++;
							}
							else
							{
								arrPosX[idxSIMDLine] = 0.0f;
								arrPosY[idxSIMDLine] = 0.0f;
								arrPosZ[idxSIMDLine] = 0.0f;
							}

							idxSIMDLine++;
							idxEdge++;
						}

						//Setup mesh
						mpu.ctFieldEvals += 4;

						Float_ cellCornerPosX_(arrPosX);
						Float_ cellCornerPosY_(arrPosY);
						Float_ cellCornerPosZ_(arrPosZ);
						Float_ vtxField;
						Float_ outColorX;
						Float_ outColorY;
						Float_ outColorZ;

						Float_ outNormalX;
						Float_ outNormalY;
						Float_ outNormalZ;

						//Compute Field and Color
						fc.fieldValueAndColor(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_,
								vtxField, outColorX, outColorY, outColorZ);

						//Use computed field to get the normal
						fc.normal(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_, vtxField,
								outNormalX, outNormalY, outNormalZ, NORMAL_DELTA);

						//Save PolyVertex
						for(int i=0; i<ctProcessedEdges; i++)
						{
							int idxEdge = iEdgeBlk * PS_SIMD_FLEN + i;

							U16 idxVertex = mpu.ctVertices;
							U16 idxVertex_X = idxVertex * 3;
							U16 idxVertex_Y = idxVertex * 3 + 1;
							U16 idxVertex_Z = idxVertex * 3 + 2;

							mpu.vPos[idxVertex_X]   = arrPosX[i];
							mpu.vPos[idxVertex_Y]   = arrPosY[i];
							mpu.vPos[idxVertex_Z]   = arrPosZ[i];

							mpu.vNorm[idxVertex_X]  = outNormalX[i];
							mpu.vNorm[idxVertex_Y]  = outNormalY[i];
							mpu.vNorm[idxVertex_Z]  = outNormalZ[i];

							mpu.vColor[idxVertex_X] = outColorX[i];
							mpu.vColor[idxVertex_Y] = outColorY[i];
							mpu.vColor[idxVertex_Z] = outColorZ[i];

							mpu.ctVertices++;


							//Get vertex v index from list. It is the last one
							idxMeshVertex[idxMissingEdge[idxEdge].z] = idxVertex;
							setEdge(edgeTable, cellCornerIDX[idxMissingEdge[idxEdge].x],
									cellCornerIDX[idxMissingEdge[idxEdge].y], idxVertex);
						}

					} //Process EdgeBlock

					for(int icase = 0; icase < ctTotalPolygons; icase++)
					{
						int idxTriangle = mpu.ctTriangles;
						mpu.triangles[idxTriangle * 3 + 0] = idxMeshVertex[icase*3 + 0];
						mpu.triangles[idxTriangle * 3 + 1] = idxMeshVertex[icase*3 + 1];
						mpu.triangles[idxTriangle * 3 + 2] = idxMeshVertex[icase*3 + 2];
						mpu.ctTriangles++;
					}

				}//End Process intersected cell
			}
		}
	}

	/*
	Float_ BL, BR, TL, TR;
	Float_ arrIsoVal(ISO_VALUE);
	Float_ one(1.0f);

	const int kMax = PS_SIMD_BLOCKS(GRID_DIM);
	const int szOneRow = PS_SIMD_FLEN * kMax;
	const int szOneGrid = szOneRow * GRID_DIM;

	float PS_SIMD_ALIGN(maskOddBL[PS_SIMD_FLEN]);
	float PS_SIMD_ALIGN(maskEvenBL[PS_SIMD_FLEN]);

	for(int iSimd=0; iSimd<PS_SIMD_FLEN; iSimd++)
	{
		if(iSimd % 2 == 0)
		{
			maskOddBL[iSimd] = 1;
			maskEvenBL[iSimd] = 2;
		}
		else
		{
			maskOddBL[iSimd] = 2;
			maskEvenBL[iSimd] = 1;
		}
	}

	Float_ maskOddBL_, maskOddBR_, maskOddTL_, maskOddTR_;
	Float_ maskEvenBL_, maskEvenBR_, maskEvenTL_, maskEvenTR_;

	maskOddBL_ = Float_(maskOddBL);
	maskOddBR_ = maskOddBL_ * Float_(16);
	maskOddTL_ = maskOddBL_ * Float_(4);
	maskOddTR_ = maskOddBL_ * Float_(64);

	maskEvenBL_ = Float_(maskEvenBL);
	maskEvenBR_ = maskEvenBL_ * Float_(16);
	maskEvenTL_ = maskEvenBL_ * Float_(4);
	maskEvenTR_ = maskEvenBL_ * Float_(64);

	for(int i=0; i<GRID_DIM-1; i++)
	{
		for(int j=0; j<GRID_DIM-1; j++)
		{
			for(int k=0; k<kMax; k++)
			{
				BL = Float_(&fvCache[ i * szOneGrid + j * szOneRow + k * PS_SIMD_FLEN]);
				BR = Float_(&fvCache[ (i+1) * szOneGrid + j * szOneRow + k * PS_SIMD_FLEN]);
				TL = Float_(&fvCache[ i * szOneGrid + (j+1) * szOneRow + k * PS_SIMD_FLEN]);
				TR = Float_(&fvCache[ (i+1) * szOneGrid + (j+1) * szOneRow + k * PS_SIMD_FLEN]);

				//Test

				//float PS_SIMD_ALIGN(test[PS_SIMD_FLEN]) = {0.1,0.1, 0.1, 0.0};
				//BL = Float_(test);

				//test = {0, 0, 0 , 0.6};
				//BR = Float_(test);

				//test = {0, 0.7, 0.7, 0.6};
				//TL = Float_(test);

				//test = {0, 0.8, 0.9 , 0.6};
				//TR = Float_(test);

				//Extract HOT/COLD
				BL = SimdAnd(SimdGTE(BL, arrIsoVal), one);
				BR = SimdAnd(SimdGTE(BR, arrIsoVal), one);
				TL = SimdAnd(SimdGTE(TL, arrIsoVal), one);
				TR = SimdAnd(SimdGTE(TR, arrIsoVal), one);

				Float_ ccOdd  = BL * maskOddBL + BR * maskOddBR_ + TL * maskOddTL_ + TR * maskOddTR_;
				Float_ ccEven = BL * maskEvenBL + BR * maskEvenBR_ + TL * maskEvenTL_ + TR * maskEvenTR_;

				//Evens should be shifted left
#ifdef SIMD_USE_M128
				ccEven = _mm_shuffle_ps(ccEven.v, ccEven.v, _MM_SHUFFLE(0, 0, 2, 1));
#else
				ccEven = _mm256_shuffle_ps(ccEven.v, ccEven.v, _MM_SHUFFLE(6, 5, 4, 3, 2, 1, 0, 0));
#endif
				ccEven = SimdhAdd(ccEven, ccEven);
				ccOdd = SimdhAdd(ccOdd, ccOdd);

				//Now we got configurations for 3 cells when SIMD=4F or
				//7 cells when SIMD=8F
			}
		}
	}
*/

//}

/*
void CMPUProcessor::process_cells_fieldsimd_cellserial_newedgeAccess(const FieldComputer& fc, MPU& mpu, tbb::tick_count& tickFieldEvals) const
{
	//Discard MPU is doesnot contain field
	if(this->discard(fc, mpu))
		return;

	//Create Field-Value Cache and init it
	const U32 m = GRID_DIM * GRID_DIM * GRID_DIM;
	float PS_SIMD_ALIGN(fvCache[m]);

	//Init
	EDGETABLE edgeTable;
	memset(&edgeTable, 0, sizeof(EDGETABLE));
	mpu.ctFieldEvals = 0;
	mpu.ctVertices = 0;
	mpu.ctTriangles = 0;

	//Compute steps
	const int kMax = PS_SIMD_BLOCKS(GRID_DIM);
	const int szOneNeedle = PS_SIMD_FLEN * kMax;
	const int szOneSlab = szOneNeedle * GRID_DIM;

	//Temporary goodies
	Float_ arrIsoVal(ISO_VALUE);
	Float_ one(1.0f);
	float PS_SIMD_ALIGN(scaleZ[PS_SIMD_FLEN]);
	for(int iSimd=0; iSimd<PS_SIMD_FLEN; iSimd++)
		scaleZ[iSimd] = iSimd;

	//Compute field at grid positions
	{
		//Temp variables
		Float_ arrInside(0.0f);
		Float_ arrOutside(0.0f);

		Float_ arrCellSize(m_cellsize);
		Float_ arrScaleZ(scaleZ);

		//Cache all field-values with in this MPU
		for(int i=0; i<GRID_DIM; i++)
		{
			for(int j=0; j<GRID_DIM; j++)
			{
				for(int k=0; k<kMax; k++)
				{
					Float_ cellCornerPOSX_(mpu.bboxLo.x + i*m_cellsize);
					Float_ cellCornerPOSY_(mpu.bboxLo.y + j*m_cellsize);
					Float_ cellCornerPOSZ_(mpu.bboxLo.z);
					Float_ k_(k * PS_SIMD_FLEN);

					cellCornerPOSZ_ = cellCornerPOSZ_ + (k_ + arrScaleZ) * arrCellSize;

					Float_ arrField;

					//FieldValue computed using SIMD
					fc.fieldValue(cellCornerPOSX_, cellCornerPOSY_, cellCornerPOSZ_, arrField);

					//Store Fields Row By Row
					arrField.store(&fvCache[ i * szOneNeedle + j * szOneSlab + k * PS_SIMD_FLEN]);

					//Keep stats
					mpu.ctFieldEvals++;
					cellCornerPOSX_ = SimdAnd(SimdGTE(arrField, arrIsoVal), one);
					arrInside = arrInside + cellCornerPOSX_;
					arrOutside = arrOutside + one - cellCornerPOSX_;
				}
			}
		}

		//Early Discard Shaved 2 seconds off
		VecNMask mask1(arrInside.v);
		if(mask1 == PS_SIMD_ALLZERO)
			return;

		mask1 = VecNMask(arrOutside.v);
		if(mask1 == PS_SIMD_ALLZERO)
			return;
	}

	//Get Tick Count for FieldEvals
	tickFieldEvals = tbb::tick_count::now();

	/////////////////////////////////////////////////////////
	//Process Multiple cells- Column Swap Method
	//For the 8 corners of each cell
	float PS_SIMD_ALIGN(arrPowerMask[8]);
	float PS_SIMD_ALIGN(arrFields[8]);

	arrPowerMask[0] = 1.0f;
	for(int iSimd=1; iSimd<8; iSimd++)
		arrPowerMask[iSimd] = 2.0f * arrPowerMask[iSimd - 1];

	int arrConfig[16];
	int idxCellConfig = 0;


#ifdef SIMD_USE_M128
	Float_ maskPowerLeft = Float_(&arrPowerMask[0]);
	Float_ maskPowerRight = Float_(&arrPowerMask[4]);
	Float_ rootResolution;
	{
		Float_ scale(scaleZ);
		Float_ oneThird(1.0f / 3.0f);
		rootResolution = scale * oneThird;
	}

#elif defined(SIMD_USE_M256)
	Float_ maskPower = Float_(&arrPowerMask[0]);
	Float_ rootResolution;
	{
		Float_ scale(scaleZ);
		Float_ oneSeventh(1.0f / 7.0f);
		rootResolution = scale * oneSeventh;
	}
#endif

	for(int i=0; i<GRID_DIM-1; i++)
	{
		//Compute once use multiple of i
		U32 ip0 = i * szOneNeedle;
		U32 ip1 = ip0 + szOneNeedle;

		for(int j=0; j<GRID_DIM-1; j++)
		{
			//Compute once use multiple of j
			U32 jp0 = j * szOneSlab;
			U32 jp1 = jp0 + szOneSlab;

			for(int k=0; k<GRID_DIM-1; k++)
			{
				//GATHER FIELDS
				U32 idx = ip0 + jp0 + k;
				arrFields[0] = fvCache[ idx];
				arrFields[1] = fvCache[ idx + 1];

				idx = ip0 + jp1 + k;
				arrFields[2] = fvCache[ idx];
				arrFields[3] = fvCache[ idx + 1];

				idx = ip1 + jp0 + k;
				arrFields[4] = fvCache[ idx];
				arrFields[5] = fvCache[ idx + 1];

				idx = ip1 + jp1 + k;
				arrFields[6] = fvCache[ idx];
				arrFields[7] = fvCache[ idx + 1];
#ifdef SIMD_USE_M128
				//Compute configuration
				Float_ Left = SimdAnd(SimdGTE(Float_(&arrFields[0]), arrIsoVal), one) * maskPowerLeft;
				Float_ Right = SimdAnd(SimdGTE(Float_(&arrFields[4]), arrIsoVal), one) * maskPowerRight;

				//3 Levels of HADDS Needed
				Left = SimdhAdd(Left, Right);
				Left = SimdhAdd(Left, Left);
				Left = SimdhAdd(Left, Left);
				idxCellConfig = static_cast<int>(Left[0]);
#elif defined(SIMD_USE_M256)
				Float_ Left = SimdAnd(SimdGTE(Float_(&arrFields[0]), arrIsoVal), one) * maskPower;
				Left = SimdhAdd(Left, Left);
				Left = SimdhAdd(Left, Left);
				idxCellConfig = static_cast<int>(Left[0] + Left[4]);
#endif
				if(idxCellConfig == 0 || idxCellConfig == 255)
					continue;

				//Fetch cell configuration
				memcpy(arrConfig, &g_triTableCache[idxCellConfig][0], 16 * sizeof(int));
				//Increment number of processed cells
				//Compute surface vertex, normal and material on each crossing edge
				int idxMeshVertex[16];
				Left = SimdhAdd(Left, Left);
				//Read case
				int ctTotalPolygons = 0;
				int ctEdges = 0;

				for(int icase=0; icase<16; icase++)
				{
					int candidate = arrConfig[icase];
					if(candidate == -1)
						break;

					//New Edge
					ctEdges++;
					int idxEdgeStart = corner1[candidate];
					int idxEdgeAxis  = edgeaxis[candidate];

					//Compute indices
					int sx = i + ((idxEdgeStart & 4) >> 2);
					int sy = j + ((idxEdgeStart & 2) >> 1);
					int sz = k + (idxEdgeStart & 1);

					idxMeshVertex[icase] = getEdge(edgeTable, sx, sy, sz, idxEdgeAxis);

					//See if the vertex exist in edge table. If it doesn't exist compute and add it to edge table
					if(idxMeshVertex[icase] == -1)
					{
						//Reduced operations for edge processing
						svec3f e1 = vadd3f(mpu.bboxLo, svec3f(m_cellsize * sx, m_cellsize * sy, m_cellsize * sz));
						svec3f e2 = e1;
						vsetElement3f(e2, idxEdgeAxis, velement3f(e2, idxEdgeAxis) + m_cellsize);


						//int idxEdgeStart = corner2[candidate];
						Float_ rootFields;
						Float_ posX(e1.x);
						Float_ posY(e1.y);
						Float_ posZ(e1.z);
						{
							Float_ displaceX(e2.x - e1.x);
							Float_ displaceY(e2.y - e1.y);
							Float_ displaceZ(e2.z - e1.z);

							posX = posX + displaceX * rootResolution;
							posY = posY + displaceY * rootResolution;
							posZ = posZ + displaceZ * rootResolution;

							fc.fieldValue(posX, posY, posZ, rootFields);
						}

						Float_ inside = SimdAnd(SimdGTE(rootFields, arrIsoVal), one);

						int interval = 0;
						//Compute Interval
						{
#if defined(SIMD_USE_M128)
							bool bRight = (static_cast<int>(inside[0]) == 0);
							inside = SimdhAdd(inside, inside);
							inside = SimdhAdd(inside, inside);
							int ii = static_cast<int>(inside[0]);
							interval = bRight * (PS_SIMD_FLEN - ii) + !bRight * ii;
#elif defined(SIMD_USE_M256)
							bool bRight = (static_cast<int>(inside[0]) == 0);
							inside = SimdhAdd(inside, inside);
							inside = SimdhAdd(inside, inside);
							int ii = static_cast<int>(inside[0] + inside[4]);
							interval = bRight * (PS_SIMD_FLEN - ii) + !bRight * ii;
#endif

						}

						//Update e1 and e2 for the high resolution field
						e1 = svec3f(posX[interval - 1], posY[interval - 1], posZ[interval - 1]);
						e2 = svec3f(posX[interval], posY[interval], posZ[interval]);

						float scale = (ISO_VALUE - rootFields[interval - 1])/(rootFields[interval] - rootFields[interval-1]);
						svec3f p = vadd3f(e1, vscale3f(scale, vsub3f(e2, e1)));

						Float_ cellCornerPosX_(p.x);
						Float_ cellCornerPosY_(p.y);
						Float_ cellCornerPosZ_(p.z);
						Float_ vtxField;
						Float_ outColorX;
						Float_ outColorY;
						Float_ outColorZ;

						Float_ outNormalX;
						Float_ outNormalY;
						Float_ outNormalZ;

						//Compute Field and Color
						fc.fieldValueAndColor(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_,
								vtxField, outColorX, outColorY, outColorZ);

						//Use computed field to get the normal
						fc.normal(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_, vtxField,
								outNormalX, outNormalY, outNormalZ, NORMAL_DELTA);


						U16 idxVertex = mpu.ctVertices;
						U16 idxVertex_X = idxVertex * 3;
						U16 idxVertex_Y = idxVertex * 3 + 1;
						U16 idxVertex_Z = idxVertex * 3 + 2;

						mpu.vPos[idxVertex_X]   = p.x;
						mpu.vPos[idxVertex_Y]   = p.y;
						mpu.vPos[idxVertex_Z]   = p.z;

						mpu.vNorm[idxVertex_X]  = outNormalX[0];
						mpu.vNorm[idxVertex_Y]  = outNormalY[0];
						mpu.vNorm[idxVertex_Z]  = outNormalZ[0];

						mpu.vColor[idxVertex_X] = outColorX[0];
						mpu.vColor[idxVertex_Y] = outColorY[0];
						mpu.vColor[idxVertex_Z] = outColorZ[0];

						mpu.ctVertices++;


						//Get vertex v index from list. It is the last one
						idxMeshVertex[icase] = idxVertex;
						setEdge(edgeTable, sx, sy, sz, idxEdgeAxis, idxVertex);
					}
				}//End icase

				//Number of polygons
				ctTotalPolygons = ctEdges / 3;
				for(int icase = 0; icase < ctTotalPolygons; icase++)
				{
					int idxTriangle = mpu.ctTriangles * 3;
					mpu.triangles[idxTriangle] = idxMeshVertex[icase*3];
					mpu.triangles[idxTriangle + 1] = idxMeshVertex[icase*3 + 1];
					mpu.triangles[idxTriangle + 2] = idxMeshVertex[icase*3 + 2];
					mpu.ctTriangles++;
				}
			}//Processed One Cell
		}
	}

}
*/

void CMPUProcessor::process_cells_scalar(const FieldComputer& fc, MPU& mpu, MPUGLOBALMESH& globalMesh, tbb::tick_count& tickFieldEvals) const
{
	//Create Field-Value Cache and init it
	//const size_t m = GRID_DIM * GRID_DIM * PS_SIMD_BLOCKS(GRID_DIM);
	const size_t m = GRID_DIM * GRID_DIM * GRID_DIM;
	float fvCache[m];

	//EdgeTable
	EDGETABLE edgeTable;
	memset(&edgeTable, 0, sizeof(EDGETABLE));
	int idxCellConfig = 0;

	mpu.ctFieldEvals = 0;
	mpu.ctVertices = 0;
	mpu.ctTriangles = 0;

	{
		int ctInside = 0;
		int ctOutside = 0;

		//Cache all field-values with in this MPU
		for(int i=0; i<GRID_DIM; i++)
		{
			for(int j=0; j<GRID_DIM; j++)
			{
				for(int k=0; k<GRID_DIM; k++)
				{
					Float_ cellCornerPOSX_(mpu.bboxLo.x + i*m_cellsize);
					Float_ cellCornerPOSY_(mpu.bboxLo.y + j*m_cellsize);
					Float_ cellCornerPOSZ_(mpu.bboxLo.z + k*m_cellsize);

					Float_ arrField;

					//FieldValue computed using SIMD
					fc.fieldValue(cellCornerPOSX_, cellCornerPOSY_, cellCornerPOSZ_, arrField);

					fvCache[CELLID_FROM_IDX(i,j,k)] = arrField[0];

					//Keep stats
					mpu.ctFieldEvals++;

					if(arrField[0] >= ISO_VALUE)
						ctInside++;
					else
						ctOutside++;
				}
			}
		}

		//Early Discard Shaved 2 seconds off
		if(ctInside == 0)
			return;

		if(ctOutside == 0)
			return;

	}
	//Get Tick Count for FieldEvals
	tickFieldEvals = tbb::tick_count::now();


	//
	U16* lpMeshTriangles = &globalMesh.vTriangles[mpu.idxGlobalID * MPU_MESHPART_TRIANGLE_STRIDE];
	U32 szVertexPartOffset = mpu.idxGlobalID * MPU_MESHPART_VERTEX_STRIDE;
	float* lpMeshVertices = &globalMesh.vPos[szVertexPartOffset];
	float* lpMeshNormals = &globalMesh.vNorm[szVertexPartOffset];
	float* lpMeshColors = &globalMesh.vColor[szVertexPartOffset];

	/////////////////////////////////////////////////////////////////////////
	//Process all cells in this MPU
	for(int i=0; i<GRID_DIM-1; i++)
	{
		for(int j=0; j<GRID_DIM-1; j++)
		{
			for(int k=0; k<GRID_DIM-1; k++)
			{
				int	cellCornerKey[8];
				svec3i cellCornerIDX[8];
				float cellCornerField[8];

				cellCornerKey[0] = CELLID_FROM_IDX(i, j, k);
				cellCornerIDX[0] = svec3i(i, j, k);

				cellCornerKey[1] = CELLID_FROM_IDX(i, j, k+1);
				cellCornerIDX[1] = svec3i(i, j, k+1);

				cellCornerKey[2] = CELLID_FROM_IDX(i, j+1, k);
				cellCornerIDX[2] = svec3i(i, j+1, k);

				cellCornerKey[3] = CELLID_FROM_IDX(i, j+1, k+1);
				cellCornerIDX[3] = svec3i(i, j+1, k+1);

				cellCornerKey[4] = CELLID_FROM_IDX(i+1, j, k);
				cellCornerIDX[4] = svec3i(i+1, j, k);

				cellCornerKey[5] = CELLID_FROM_IDX(i+1, j, k+1);
				cellCornerIDX[5] = svec3i(i+1, j, k+1);

				cellCornerKey[6] = CELLID_FROM_IDX(i+1, j+1, k);
				cellCornerIDX[6] = svec3i(i+1, j+1, k);

				cellCornerKey[7] = CELLID_FROM_IDX(i+1, j+1, k+1);
				cellCornerIDX[7] = svec3i(i+1, j+1, k+1);

				//Compute Cell Config
				idxCellConfig = 0;
				for(int icase=0; icase<8; icase++)
				{
					cellCornerField[icase] = fvCache[cellCornerKey[icase]];
					if(cellCornerField[icase] > ISO_VALUE)
						idxCellConfig += (1 << icase);
				}

				//Cell Configuration
				if((idxCellConfig != 0)&&(idxCellConfig != 255))
				{
					//Increment number of processed cells
					//ctIntersectedCells++;
					//Compute surface vertex, normal and material on each crossing edge
					int idxMeshVertex[16];

					//Read case
					int ctTotalPolygons = 0;
					int ctEdges = 0;

					for(int icase=0; icase<16; icase++)
					{
						int candidate = g_triTableCache[idxCellConfig][icase];
						if(candidate != -1)
						{
							int idxEdgeStart = corner1[candidate];
							int idxEdgeEnd 	 = corner2[candidate];
							idxMeshVertex[icase] = getEdge(edgeTable,
									cellCornerIDX[idxEdgeStart],
									cellCornerIDX[idxEdgeEnd]);

							//See if the vertex exist in edge table. If it doesn't exist compute and add it to edge table
							if(idxMeshVertex[icase] == -1)
							{
								float scale = (ISO_VALUE - cellCornerField[idxEdgeStart])/(cellCornerField[idxEdgeEnd] - cellCornerField[idxEdgeStart]);
								svec3f e1 = vadd3f(mpu.bboxLo, svec3f(m_cellsize * cellCornerIDX[idxEdgeStart].x, m_cellsize * cellCornerIDX[idxEdgeStart].y, m_cellsize * cellCornerIDX[idxEdgeStart].z));
								svec3f e2 = vadd3f(mpu.bboxLo, svec3f(m_cellsize * cellCornerIDX[idxEdgeEnd].x, m_cellsize * cellCornerIDX[idxEdgeEnd].y, m_cellsize * cellCornerIDX[idxEdgeEnd].z));
								svec3f p = vadd3f(e1, vscale3f(scale, vsub3f(e2, e1)));

								Float_ cellCornerPosX_(p.x);
								Float_ cellCornerPosY_(p.y);
								Float_ cellCornerPosZ_(p.z);
								Float_ vtxField;
								Float_ outColorX;
								Float_ outColorY;
								Float_ outColorZ;

								Float_ outNormalX;
								Float_ outNormalY;
								Float_ outNormalZ;

								//Compute Field and Color
								fc.fieldValueAndColor(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_,
										vtxField, outColorX, outColorY, outColorZ);

								//Use computed field to get the normal
								fc.normal(cellCornerPosX_, cellCornerPosY_, cellCornerPosZ_, vtxField,
										outNormalX, outNormalY, outNormalZ, NORMAL_DELTA);

								U16 idxVertex = mpu.ctVertices;
								U16 idxVertex_X = idxVertex * 3;
								U16 idxVertex_Y = idxVertex * 3 + 1;
								U16 idxVertex_Z = idxVertex * 3 + 2;

								lpMeshVertices[idxVertex_X]   = cellCornerPosX_[0];
								lpMeshVertices[idxVertex_Y]   = cellCornerPosY_[0];
								lpMeshVertices[idxVertex_Z]   = cellCornerPosZ_[0];

								lpMeshNormals[idxVertex_X]  = outNormalX[0];
								lpMeshNormals[idxVertex_Y]  = outNormalY[0];
								lpMeshNormals[idxVertex_Z]  = outNormalZ[0];

								lpMeshColors[idxVertex_X] = outColorX[0];
								lpMeshColors[idxVertex_Y] = outColorY[0];
								lpMeshColors[idxVertex_Z] = outColorZ[0];

								mpu.ctVertices++;


								//Get vertex v index from list. It is the last one
								idxMeshVertex[icase] = idxVertex;
								setEdge(edgeTable, cellCornerIDX[idxEdgeStart], cellCornerIDX[idxEdgeEnd], idxVertex);
							}
							ctEdges++;
						}
						else
							break;
					}
					ctTotalPolygons = ctEdges / 3;
					for(int icase = 0; icase < ctTotalPolygons; icase++)
					{
						int idxTriangle = mpu.ctTriangles;
						lpMeshTriangles[idxTriangle * 3 + 0] = idxMeshVertex[icase*3 + 0];
						lpMeshTriangles[idxTriangle * 3 + 1] = idxMeshVertex[icase*3 + 1];
						lpMeshTriangles[idxTriangle * 3 + 2] = idxMeshVertex[icase*3 + 2];
						mpu.ctTriangles++;
					}

				}//End Process intersected cell
			}
		}
	}
}

}
}
