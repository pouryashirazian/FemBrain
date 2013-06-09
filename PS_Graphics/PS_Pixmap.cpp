#include <string.h>
#include "PS_Pixmap.h"
#include "../PS_Base/PS_FileDirectory.h"
#include "lodepng.h"


namespace PS{
	namespace RASTER{
		Pixmap::Pixmap():m_width(0), m_height(0), m_depth(DEFAULT_PIXMAP_DEPTH), m_bitmap(NULL) {
		}

		Pixmap::Pixmap(const char* chrFilePath)
		{
			m_width = m_height;
			m_depth = DEFAULT_PIXMAP_DEPTH;
			m_bitmap = NULL;
			load(chrFilePath);
		}

		Pixmap::Pixmap(int w, int h)
		{
			m_bitmap = NULL;
			reset(w, h);
		}

		Pixmap::Pixmap(const Pixmap& other)
		{
			m_width= other.width();
			m_height = other.height();
			m_depth = other.depth();
			m_bitmap = new U8[m_width * m_height * m_depth];
			memcpy(m_bitmap, other.const_data(), m_width * m_height * m_depth);
		}

		Pixmap::~Pixmap()
		{
			SAFE_DELETE_ARRAY(m_bitmap);
		}


		void Pixmap::reset(int w, int h)
		{
			SAFE_DELETE_ARRAY(m_bitmap);
			m_width = w, m_height = h, m_depth = DEFAULT_PIXMAP_DEPTH;
			m_bitmap = new U8[m_width * m_height * m_depth];
			memset(m_bitmap, 0, m_width * m_height * m_depth);
		}

		void Pixmap::checkers(const rgba8& color) {
			//const int D = min(height(), width()) / 16;
			int mask;
			rgba8 colorMasked = rgba8(color.x ^ 255, color.y ^ 255, color.z ^ 255, color.w);
			for (int x = 0; x < width(); x++) {
				for (int y = 0; y < height(); y++) {
					mask = ((x & 16) == 0) ^ ((y & 16) == 0);
					if(mask)
						putPixel(x, y, colorMasked);
					else
						putPixel(x, y, color);
				}
			}
		}

		void Pixmap::fill(const rgba8& color)
		{
			for (int x=0; x<width(); x++)
				for (int y=0; y<height(); y++)
					putPixel(x, y, color);
		}

		bool Pixmap::load(const char* chrFilePath)
		{
			DAnsiStr strExt = PS::FILESTRINGUTILS::ExtractFileExt(DAnsiStr(chrFilePath));
			if(strExt.toUpper() == "PNG") {
				std::vector<U8> image;
				U32 width, height;

				//decode
				U32 error = lodepng::decode(image, width, height, chrFilePath);
				if(error) {
					printf("decoder error %d : %s", error, lodepng_error_text(error));
					return false;
				}
				else {
					reset(width, height);
					memcpy(m_bitmap, &image[0], m_width * m_height * 4);
					return true;
				}
			}
			return false;
		}


		bool Pixmap::save(const char * chrFilePath)
		{
			DAnsiStr strExt = PS::FILESTRINGUTILS::ExtractFileExt(DAnsiStr(chrFilePath));
			if(strExt.toUpper() == DAnsiStr("PNG")) {
				std::vector<U8> data;
				data.assign(m_bitmap, m_bitmap + m_width *  m_height * m_depth);

				U32 err = lodepng::encode(chrFilePath, data, m_width, m_height);
				if(err)
					printf("Error while encoding: %s", lodepng_error_text(err));
				else
					return true;
			}

			return false;
		}



	}
}
