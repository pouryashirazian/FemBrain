// References :
//
// http://hal.archives-ouvertes.fr/docs/00/59/67/30/PDF/ocl-radix-sort.pdf

// change of changeTranspositionIndex for the transposition
inline int changeTranspositionIndex(int i,int n)
{
    int ip;

#ifdef TRANSPOSE
    int k, l;
    k = i / (n/_GROUPS/_ITEMS);
    l = i % (n/_GROUPS/_ITEMS);
    ip = l * (_GROUPS*_ITEMS) + k;
#else
    ip = i;
#endif

    return ip;
}

//------------------------------------------------------------
// kernel_Histogram
//
// Purpose : compute the histogram for each radix and each virtual processor for the pass
//------------------------------------------------------------

__kernel
void kernel_Histogram(
	const __global int* d_Keys,
	__global int* d_Histograms,
	const int pass,
	__local int* loc_histo,
	const int n
	)
{
    size_t it = get_local_id(0);
    size_t ig = get_global_id(0);

    size_t gr = get_group_id(0);

    size_t groups=get_num_groups(0);
    size_t items=get_local_size(0);

    // set the local histograms to zero
    for (int ir=0;ir<_RADIX;ir++)
    {
        //d_Histograms[ir * groups * items + items * gr + it]=0;
        loc_histo[ir * items + it] = 0;
    }

    barrier(CLK_LOCAL_MEM_FENCE);


    // range of keys that are analyzed by the work item
    //int start= gr * n/groups + it * n/groups/items;
    int size = n / groups / items;
    int start= ig * size;

  int key,shortkey,k;

  // compute the index
  // the computation depends on the transposition
  for(int j= 0; j< size;j++){
#ifdef TRANSPOSE
    k= groups * items * j + ig;
#else
    k=j+start;
#endif
      
    key=d_Keys[k];   

    // extract the group of _BITS bits of the pass
    // the result is in the range 0.._RADIX-1
    shortkey=(( key >> (pass * _BITS)) & (_RADIX-1));  

    // increment the local histogram
    loc_histo[shortkey *  items + it ]++;
  }

    barrier(CLK_LOCAL_MEM_FENCE);

    // copy the local histogram to the global one
    for (int ir = 0; ir < _RADIX; ir++)
        d_Histograms[ir * groups * items + items * gr + it]=loc_histo[ir * items + it];

    barrier(CLK_GLOBAL_MEM_FENCE);
}

//------------------------------------------------------------
// kernel_Transpose
//
// Purpose : Transpose a set of int
//------------------------------------------------------------

__kernel
void kernel_Transpose(
	const __global int* invect,
	__global int* outvect,
	const int nbcol,
	const int nbrow,
	const __global int* inperm,
	__global int* outperm,
	__local int* blockmat,
	__local int* blockperm
	)
{
    size_t i0 = get_global_id(0) * _GROUPS;  // first row changeTranspositionIndex
    size_t j = get_global_id(1);  // column changeTranspositionIndex

    int iloc;  // first local row changeTranspositionIndex
    size_t jloc = get_local_id(1);  // local column changeTranspositionIndex

    // Fill the cache
    for(iloc = 0; iloc < _GROUPS;iloc++)
    {
        int k = (i0+iloc)*nbcol+j;  // position in the matrix
        blockmat[iloc*_GROUPS+jloc] = invect[k];
#ifdef PERMUT
        blockperm[iloc*_GROUPS+jloc] = inperm[k];
#endif
    }

    barrier(CLK_LOCAL_MEM_FENCE);

    // first row changeTranspositionIndex in the transpose
    int j0 = get_group_id(1)*_GROUPS;

    // put the cache at the good place
    // loop on the rows
    for (iloc = 0; iloc < _GROUPS; iloc++)
    {
        int kt=(j0+iloc)*nbrow+i0+jloc;  // position in the transpose
        outvect[kt]=blockmat[jloc*_GROUPS+iloc];
#ifdef PERMUT
        outperm[kt]=blockperm[jloc*_GROUPS+iloc];
#endif
    }

}

//------------------------------------------------------------
// kernel_Reorder
//
// Purpose : Each virtual processor reorders its data using the scanned histogram
//------------------------------------------------------------

__kernel
void kernel_Reorder(
	const __global int* d_inKeys,
	__global int* d_outKeys,
	__global int* d_Histograms,
	const int pass,
	__global int* d_inPermut,
	__global int* d_outPermut,
	__local int* loc_histo,
	const int n
	)
{
    size_t it = get_local_id(0);
    size_t ig = get_global_id(0);

    size_t gr = get_group_id(0);
    size_t groups = get_num_groups(0);
    size_t items = get_local_size(0);

    //int start= gr * n/groups + it * n/groups/items;
    int start = ig *(n/groups/items);
    int size = n/groups/items;

    // take the histograms in the cache
    for (int ir=0;ir<_RADIX;ir++)
        loc_histo[ir * items + it] = d_Histograms[ir * groups * items + items * gr + it];
    barrier(CLK_LOCAL_MEM_FENCE);


  int newpos,key,shortkey,k,newpost;

  for(int j= 0; j< size;j++){
#ifdef TRANSPOSE
      k= groups * items * j + ig;
#else
      k=j+start;
#endif
    key = d_inKeys[k];   
    shortkey=((key >> (pass * _BITS)) & (_RADIX-1)); 

    newpos=loc_histo[shortkey * items + it];


#ifdef TRANSPOSE
    int ignew,jnew;
    ignew= newpos/(n/groups/items);
    jnew = newpos%(n/groups/items);
    newpost = jnew * (groups*items) + ignew;
#else
    newpost=newpos;
#endif

    d_outKeys[newpost]= key;  // killing line !!!

#ifdef PERMUT 
      d_outPermut[newpost]=d_inPermut[k]; 
#endif

    newpos++;
    loc_histo[shortkey * items + it]=newpos;

    }
}

//------------------------------------------------------------
// kernel_ScanHistograms
//
// Purpose :
// perform a parallel prefix sum (a scan) on the local histograms
// (see Blelloch 1990) each workitem worries about two memories
// see also http://http.developer.nvidia.com/GPUGems3/gpugems3_ch39.html
//------------------------------------------------------------

__kernel
void kernel_ScanHistograms(
	__global int* histo,
	__local int* temp,
	__global int* globsum
	)
{
    size_t it = get_local_id(0);
    size_t ig = get_global_id(0);
    int decale = 1;
    size_t n = get_local_size(0) * 2 ;
    size_t gr = get_group_id(0);

    // load input into local memory
    // up sweep phase
    temp[2*it] = histo[2*ig];
    temp[2*it+1] = histo[2*ig+1];

    // parallel prefix sum (algorithm of Blelloch 1990)
    for(int d = n>>1; d > 0; d >>= 1)
    {
        barrier(CLK_LOCAL_MEM_FENCE);
        if (it < d)
        {
            int ai = decale*(2*it+1)-1;
            int bi = decale*(2*it+2)-1;
            temp[bi] += temp[ai];
        }
        decale *= 2;
    }

    // store the last element in the global sum vector
    // (maybe used in the next step for constructing the global scan)
    // clear the last element
    if (it == 0)
    {
        globsum[gr]=temp[n-1];
        temp[n - 1] = 0;
    }

    // down sweep phase
    for(int d = 1; d < n; d *= 2)
    {
        decale >>= 1;
        barrier(CLK_LOCAL_MEM_FENCE);

        if (it < d)
        {
            int ai = decale*(2*it+1)-1;
            int bi = decale*(2*it+2)-1;

            int t = temp[ai];
            temp[ai] = temp[bi];
            temp[bi] += t;
        }

    }
    barrier(CLK_LOCAL_MEM_FENCE);

    // write results to device memory

    histo[2*ig] = temp[2*it];
    histo[2*ig+1] = temp[2*it+1];

    barrier(CLK_GLOBAL_MEM_FENCE);
}

//------------------------------------------------------------
// kernel_PasteHistograms
//
// Purpose :
// use the global sum for updating the local histograms
// each work item updates two values
//------------------------------------------------------------

__kernel
void kernel_PasteHistograms(__global int* histo, __global int* globsum)
{
    size_t ig = get_global_id(0);
    size_t gr = get_group_id(0);

    int s;

    s = globsum[gr];

    // write results to device memory
    histo[2*ig] += s;
    histo[2*ig+1] += s;

    barrier(CLK_GLOBAL_MEM_FENCE);
}
