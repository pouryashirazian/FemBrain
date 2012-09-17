#ifndef PS_PIXELMAP_H
#define PS_PIXELMAP_H

#include <iostream>
#include <fstream>
#include <math.h>
#include "../PS_Base/PS_MathBase.h"

using namespace std;

namespace PS{
namespace RASTER{

//**************************************************************
//CPixelMap manages a bitmap, load/save from/to PPM files.
//Put and Get Pixel Values
//Add and Subtract from pixel values
//Get the pointer to the buffer of pixels
//Get and Set the dimensions of bitmap
//**************************************************************
class CPixelMap
{
public:
	CPixelMap( const char* );
	CPixelMap( FILE* fil);
	CPixelMap( int, int );
	CPixelMap( const CPixelMap* );
	~CPixelMap();

	bool load( const char* );
	bool load( FILE* fil );
	bool save(const char * fname);

	inline void putPixel ( int x, int y, U8 r, U8 g, U8 b )
	{
		if (x>=0 && x<m_w && y>=0 && y<m_h)
			map[y*m_w*3+x*3] = r,map[y*m_w*3+x*3+1]=g,map[y*m_w*3+x*3+2]=b;
	}

	inline void addpixel ( int x, int y, U8 r, U8 g, U8 b )
	{
		if (x>=0 && x<m_w && y>=0 && y<m_h)
			map[y*m_w*3+x*3] += r,map[y*m_w*3+x*3+1]+=g,map[y*m_w*3+x*3+2]+=b;
	}

	inline void addipixel ( int x, int y, int r, int g, int b ) 
	{
		int ctPixel = 0;
		if (x>=0 && x<m_w && y>=0 && y<m_h)
		{
			ctPixel = y*m_w*3+x*3;
			map[ctPixel + 0] =((map[ctPixel + 0]+r)<0) ? 0 : map[ctPixel + 0]+r;
			map[ctPixel + 1] =((map[ctPixel + 1]+g)<0) ? 0 : map[ctPixel + 1]+g;
			map[ctPixel + 2] =((map[ctPixel + 2]+b)<0) ? 0 : map[ctPixel + 2]+b;
		}
	}

	inline void subpixel ( int x, int y, int r, int g, int b ) 
	{
		if (x>=0 && x<m_w && y>=0 && y<m_h)
		{
			map[y*m_w*3+x*3+0]=((map[y*m_w*3+x*3+0]-r)<0) ? 0 : map[y*m_w*3+x*3+0]-r;
			map[y*m_w*3+x*3+1]=((map[y*m_w*3+x*3+1]-g)<0) ? 0 : map[y*m_w*3+x*3+1]-g;
			map[y*m_w*3+x*3+2]=((map[y*m_w*3+x*3+2]-b)<0) ? 0 : map[y*m_w*3+x*3+2]-b;
		}
	}

	const U8* getPixel ( int, int ) const;
	U8 red ( int, int ) const;  
	U8 green ( int, int ) const;  
	U8 blue ( int, int ) const;  
	U8 component ( int, int,  int ) const;  

	int width() const { return m_w; }
	int height() const { return m_h; }

	const U8* const_buffer() const{ return map; }
	U8* buffer() {return map;}

	void reset( int, int );
	void checkers( U8 = 0, U8 = 0, U8 = 0);
	void backGround(U8 cr, U8 cg, U8 cb);

	//  void stripes( U8 = 0, U8 = 0, U8 = 0);  
	//  const CPixelMap& difference(const CPixelMap&);

	friend ostream& operator << ( ostream&, const CPixelMap&);


private:
	int readHeader ( FILE* );
	void whiteSpace( FILE* );

private:
	U8* map;
	int m_w, m_h;
};

}
}
#endif
