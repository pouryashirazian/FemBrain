//Tetrahedralization
//If the cell config is non-zero then it will be exploded to 6 tets
__kernel void TetMeshCells(__global U8* arrInCellConfig,
						    __constant struct CellParam* inCellParam,
						    __constant struct GridParam* inGridParam,
						    __global U32* arrOutIncludedCells,
						    __global U32* arrOutIncludedVertices) {
	
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inCellParam->ctNeededCells[0])||(idY >= inCellParam->ctNeededCells[1])||(idZ >= inCellParam->ctNeededCells[2]))
		return;
	U32 idxCell = idZ * (inCellParam->ctNeededCells[0] * inCellParam->ctNeededCells[1]) + idY * inCellParam->ctNeededCells[0] + idX;
	if(idxCell >= inCellParam->ctTotalCells)
		return;
	
	//In the cell is inside or crossing the surface then count it
	arrOutIncludedCells[idxCell] = (arrInCellConfig[idxCell] != 0);
	
	if(arrOutIncludedCells[idxCell]) {
		U32 dx = inGridParam->ctGridPoints[0];	
		U32 dxdy = inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1];
		
		//Compute Configuration index
	    arrOutIncludedVertices[idZ * dxdy + idY * dx + idX] = 1;
	    arrOutIncludedVertices[(idZ+1) * dxdy + idY * dx + idX] = 1;
	    arrOutIncludedVertices[idZ * dxdy + (idY+1) * dx + idX] = 1;
	    arrOutIncludedVertices[(idZ+1) * dxdy + (idY+1) * dx + idX] = 1;
	    arrOutIncludedVertices[idZ * dxdy + idY * dx + idX + 1] = 1;
	    arrOutIncludedVertices[(idZ+1) * dxdy + idY * dx + idX + 1] = 1;
	    arrOutIncludedVertices[idZ * dxdy + (idY+1) * dx + idX + 1] = 1;
	    arrOutIncludedVertices[(idZ+1) * dxdy + (idY+1) * dx + idX + 1] = 1;
	}
}


//Compute mesh vertices
__kernel void TetMeshVertices(__global float4* arrInFields,
							  __global U32* arrInVerticesIncluded,
							  __global U32* arrInVerticesOffset,
							  __constant struct GridParam* inGridParam,
							  __global float* arrOutTetMeshVertices)
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inGridParam->ctGridPoints[0])||(idY >= inGridParam->ctGridPoints[1])||(idZ >= inGridParam->ctGridPoints[2]))
		return;
	U32 idxVertex = idZ * (inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1]) + idY * inGridParam->ctGridPoints[0] + idX;
	if(idxVertex >= inGridParam->ctTotalPoints)
		return;
		
	//IF This vertex has no hot edges then return
	if(arrInVerticesIncluded[idxVertex] == 0)
		return;
	
	float4 v = arrInFields[idxVertex];
	U32 idxStore = arrInVerticesOffset[idxVertex] * 3;
	
	arrOutTetMeshVertices[idxStore] = v.x;
	arrOutTetMeshVertices[idxStore + 1] = v.y;
	arrOutTetMeshVertices[idxStore + 2] = v.z;		
}

//Explode each crossing or inside cell into 6 tetrahedra
__kernel void TetMeshElements(__global U32* arrInVerticesOffset,
							  __global U32* arrInCellTetOffset,   	  	  	  	  	  
							  __global U32* arrInCellsIncluded,							  
							  __constant struct GridParam* inGridParam,
							  __constant struct CellParam* inCellParam,
							  __global U32* arrOutTetMeshIndices) 
{
	int idX = get_global_id(0);
	int idY = get_global_id(1);
	int idZ = get_global_id(2);
	if((idX >= inCellParam->ctNeededCells[0])||(idY >= inCellParam->ctNeededCells[1])||(idZ >= inCellParam->ctNeededCells[2]))
		return;
	U32 idxCell = idZ * (inCellParam->ctNeededCells[0] * inCellParam->ctNeededCells[1]) + idY * inCellParam->ctNeededCells[0] + idX;
	if(idxCell >= inCellParam->ctTotalCells)
		return;
	if(arrInCellsIncluded[idxCell] == 0)
		return;

	U32 dx = inGridParam->ctGridPoints[0];	
	U32 dxdy = inGridParam->ctGridPoints[0] * inGridParam->ctGridPoints[1];

	//Where to store the indices
    U32 offsetTet = arrInCellTetOffset[idxCell] * 24;
	
	//Compute Configuration index
    U32 idxCorners[8];
    idxCorners[0] = arrInVerticesOffset[idZ * dxdy + idY * dx + idX];
    idxCorners[1] = arrInVerticesOffset[(idZ+1) * dxdy + idY * dx + idX];
    idxCorners[2] = arrInVerticesOffset[idZ * dxdy + (idY+1) * dx + idX];
    idxCorners[3] = arrInVerticesOffset[(idZ+1) * dxdy + (idY+1) * dx + idX];
    idxCorners[4] = arrInVerticesOffset[idZ * dxdy + idY * dx + idX + 1];
    idxCorners[5] = arrInVerticesOffset[(idZ+1) * dxdy + idY * dx + idX + 1];
    idxCorners[6] = arrInVerticesOffset[idZ * dxdy + (idY+1) * dx + idX + 1];
    idxCorners[7] = arrInVerticesOffset[(idZ+1) * dxdy + (idY+1) * dx + idX + 1];

    
	arrOutTetMeshIndices[offsetTet + 0] = idxCorners[LBN];
	arrOutTetMeshIndices[offsetTet + 1] = idxCorners[LTN];
	arrOutTetMeshIndices[offsetTet + 2] = idxCorners[RBN];
	arrOutTetMeshIndices[offsetTet + 3] = idxCorners[LBF];
	
	arrOutTetMeshIndices[offsetTet + 4] = idxCorners[RTN];
	arrOutTetMeshIndices[offsetTet + 5] = idxCorners[LTN];
	arrOutTetMeshIndices[offsetTet + 6] = idxCorners[LBF];
	arrOutTetMeshIndices[offsetTet + 7] = idxCorners[RBN];

	arrOutTetMeshIndices[offsetTet + 8] = idxCorners[RTN];
	arrOutTetMeshIndices[offsetTet + 9] = idxCorners[LTN];
	arrOutTetMeshIndices[offsetTet + 10] = idxCorners[LTF];
	arrOutTetMeshIndices[offsetTet + 11] = idxCorners[LBF];

	arrOutTetMeshIndices[offsetTet + 12] = idxCorners[RTN];
	arrOutTetMeshIndices[offsetTet + 13] = idxCorners[RBN];
	arrOutTetMeshIndices[offsetTet + 14] = idxCorners[LBF];
	arrOutTetMeshIndices[offsetTet + 15] = idxCorners[RBF];

	arrOutTetMeshIndices[offsetTet + 16] = idxCorners[RTN];
	arrOutTetMeshIndices[offsetTet + 17] = idxCorners[LBF];
	arrOutTetMeshIndices[offsetTet + 18] = idxCorners[LTF];
	arrOutTetMeshIndices[offsetTet + 19] = idxCorners[RBF];

	arrOutTetMeshIndices[offsetTet + 20] = idxCorners[RTN];
	arrOutTetMeshIndices[offsetTet + 21] = idxCorners[LTF];
	arrOutTetMeshIndices[offsetTet + 22] = idxCorners[RTF];
	arrOutTetMeshIndices[offsetTet + 23] = idxCorners[RBF];
}




