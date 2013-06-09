#ifndef PS_PIXELMAP_H
#define PS_PIXELMAP_H

#include <iostream>
#include <fstream>
#include <math.h>
#include "../PS_Base/PS_MathBase.h"
#include "PS_Vector.h"

using namespace std;
using namespace PS;
using namespace PS::MATH;

#define DEFAULT_PIXMAP_DEPTH 4
typedef vec4u8 rgba8;

namespace PS{
namespace RASTER{

//**************************************************************
//Pixmap is a grid of RGBA values
//Supports fast pixel operations.
//Access the pointer to the buffer of pixels
//Save and Load to png
//**************************************************************
class Pixmap
{
public:
	Pixmap();
	Pixmap( const char* chrFilePath);
	Pixmap(int w, int h);
	Pixmap(const Pixmap& other);
	virtual ~Pixmap();

	//Dims
	int width() const { return m_width; }
	int height() const { return m_height; }
	int depth() const {return m_depth;}

	//Data Pointers
	const U8* const_data() const { return &m_bitmap[0]; }
	U8* data() {return &m_bitmap[0];}

	//Pixel Ops
	inline void putPixel(int x, int y, const rgba8& px) {
		assert(x >= 0 && x < m_width && y >= 0 && y < m_height);
		px.store(&m_bitmap[y * m_width * m_depth + x * m_depth]);
	}

	inline rgba8 getPixel(int x, int y) const {
		Clamp<int>(x, 0, m_width - 1);
		Clamp<int>(y, 0, m_height - 1);
		return rgba8(&m_bitmap[y * m_width * m_depth + x * m_depth]);
	}

	//Global Changes
	void reset( int w, int h);
	void checkers(const rgba8& color);
	void fill(const rgba8& color);

	//Serialize
	bool load(const char* chrFilePath);
	bool save(const char* chrFilePath);


private:
	U8* m_bitmap;
	int m_width;
	int m_height;
	int m_depth;
};

}
}
#endif
