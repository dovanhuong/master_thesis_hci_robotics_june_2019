/******************************************************************************************
*                                                                                        *
*    Optimal Path Planner to Separate Different-colored Beans with Two Agents            *
*    Version 1.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2012  Soonkyum Kim                                                    *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: kim.soonkyum@gmail.com                                                     *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
#ifndef _MY_PNGFILE_H_
#define _MY_PNGFILE_H_

#include <stdio.h>
#include <vector>
#include <png.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

class myPngfile
{
public:
	myPngfile (const char *filename) {
		valid = false;
    	png_ptr = NULL;
    	info_ptr = NULL;
	    row_pointers = NULL;
    	
	    fp = fopen (filename, "wb");
	    if (!fp) { png_destroy_write_struct(&png_ptr, &info_ptr); fclose (fp); return; }

	    png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	    if (png_ptr == NULL) { fclose (fp); return; }
    
	    info_ptr = png_create_info_struct (png_ptr);
	    if (info_ptr == NULL) { png_destroy_write_struct(&png_ptr, &info_ptr); fclose (fp); return; }
    
	    if (setjmp (png_jmpbuf (png_ptr))) { png_destroy_write_struct(&png_ptr, &info_ptr); fclose (fp); return; }
	    valid = true;
	}

	~myPngfile () {
	    for (size_t y = 0; y < m_maxY; y++) {
	        png_free(png_ptr, row_pointers[y]);
	    }
	    png_free(png_ptr, row_pointers);

		png_destroy_write_struct (&png_ptr, &info_ptr);
	    fclose (fp);
	}
	
    bool setSize(const uint& _x, const uint& _y, const double _minX, const double _minY, const double _maxX, const double _maxY)
    {
    	m_maxX = _x;
    	m_maxY = _y;
	    png_set_IHDR (png_ptr,
    	              info_ptr,
    	              m_maxX,
    	              m_maxY,
	                  8,
	                  PNG_COLOR_TYPE_RGB,
	                  PNG_INTERLACE_NONE,
	                  PNG_COMPRESSION_TYPE_DEFAULT,
	                  PNG_FILTER_TYPE_DEFAULT);

	    row_pointers = (png_byte**)png_malloc(png_ptr, m_maxY*sizeof(png_byte *));
	    for (size_t y = 0; y < m_maxY; ++y) {
       		png_byte *row = (png_byte*)png_malloc (png_ptr, 3*sizeof(uint8_t)*m_maxX);
        	row_pointers[m_maxY-y-1] = row;
        }
        // ste step size of the png
        m_minX = _minX;
        m_minY = _minY;
        m_stepX = (_maxX - m_minX)/((double)(m_maxX));
        m_stepY = (_maxY - m_minY)/((double)(m_maxY));

       	return (true);
    }
    
    bool setBackGroundColor(const char& _color)
    {
    	setColor(_color);
	    for (size_t y = 0; y < m_maxY; ++y) {
		    for (size_t x = 0; x < m_maxX; ++x) {
		    	row_pointers[y][3*x] = r;
		    	row_pointers[y][3*x+1] = g;
		    	row_pointers[y][3*x+2] = b;
    		}
    	}
    	return (true);
    }
        
    bool addCircle(const double& xc, const double& yc, const double& rc, double w/* = 0.0*/, const char& _color/* = 'k'*/)
    {
    	setColor(_color);

		w = MAX(w,MAX(m_stepX,m_stepY));
    	double xd, yd, d;
    	size_t minx, miny, maxx, maxy;
    	minx = (size_t)floor(MAX(0.0,(xc-rc-m_minX-w-m_stepX)/m_stepX));
    	maxx = MIN(m_maxX,(size_t)ceil((xc+rc-m_minX+w+m_stepX)/m_stepX));
    	miny = (size_t)floor(MAX(0.0,(yc-rc-m_minY-w-m_stepY)/m_stepY));
    	maxy = MIN(m_maxY,(size_t)ceil((yc+rc-m_minY+w+m_stepY)/m_stepY));
	    for (size_t y = miny; y < maxy; ++y)
		{
	    	yd = ((double)y)*m_stepY + m_minY;
		    for (size_t x = minx; x < maxx; ++x) {
		    	xd = ((double)x)*m_stepX + m_minX;
				d = sqrt((xd-xc)*(xd-xc)+(yd-yc)*(yd-yc));
		    	if( fabs(rc-d) < MAX(m_stepX,m_stepY) )
				{
			    	row_pointers[m_maxY-y-1][3*x] = r;
			    	row_pointers[m_maxY-y-1][3*x+1] = g;
			    	row_pointers[m_maxY-y-1][3*x+2] = b;
			    }
    		}
    	}
    	return (true);
    }

    bool save()
    {
	    png_init_io (png_ptr, fp);
	    png_set_rows (png_ptr, info_ptr, row_pointers);
	    png_write_png (png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
	    return (true);
    }

private:
	bool valid;
    FILE *fp;
    png_structp png_ptr;
    png_infop info_ptr;
    png_byte **row_pointers;

	uint8_t r, g, b;
	uint m_maxX, m_maxY;
	double m_minX, m_minY, m_stepX, m_stepY;

	void setColor(const char& _c)
	{
		switch(_c) {
			case 'r':
			case 'R':
				r = 255; g = 0; b = 0;
				break;
			case 'b':
			case 'B':
				r = 0; g = 0; b = 255;
				break;
			case 'g':
			case 'G':
				r = 0; g = 255; b = 0;
				break;
			case 'y':
			case 'Y':
				r = 255; g = 255; b = 0;
				break;
			case 'k':
			case 'K':
				r = 0; g = 0; b = 0;
				break;
			case 'w':
			case 'W':
			default:
				r = 255; g = 255; b = 255;
				break;
		}
	}
};

#endif // _MY_PNGFILE_H_
