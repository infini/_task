//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <string.h>
#include "DebugDraw.h"
#include "DetourMath.h"
#include "DetourCoordinates.h"


duDebugDraw::~duDebugDraw()
{
	// Empty
}
	

inline int bit(int a, int b)
{
	return (a & (1 << b)) >> b;
}

unsigned int duIntToCol(int i, int a)
{
	int	r = bit(i, 1) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 2) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 0) + bit(i, 5) * 2 + 1;
	return duRGBA(r*63,g*63,b*63,a);
}

void duIntToCol(int i, dtCoordinates& col)
{
	int	r = bit(i, 0) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 1) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 2) + bit(i, 5) * 2 + 1;
	col.SetX( 1 - r*63.0f/255.0f );
	col.SetY( 1 - g*63.0f/255.0f );
	col.SetZ( 1 - b*63.0f/255.0f );
}

void duCalcBoxColors(unsigned int* colors, unsigned int colTop, unsigned int colSide)
{
	if (!colors) return;
	
	colors[0] = duMultCol(colTop, 250);
	colors[1] = duMultCol(colSide, 140);
	colors[2] = duMultCol(colSide, 165);
	colors[3] = duMultCol(colSide, 217);
	colors[4] = duMultCol(colSide, 165);
	colors[5] = duMultCol(colSide, 217);
}

void duDebugDrawCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendCylinderWire(dd, minx,miny,minz, maxx,maxy,maxz, col);
	dd->end();
}

void duDebugDrawBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						float maxx, float maxy, float maxz, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendBoxWire(dd, minx,miny,minz, maxx,maxy,maxz, col);
	dd->end();
}

void duDebugDrawArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					const float x1, const float y1, const float z1, const float h,
					const float as0, const float as1, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendArc(dd, x0,y0,z0, x1,y1,z1, h, as0, as1, col);
	dd->end();
}

void duDebugDrawArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					  const float x1, const float y1, const float z1,
					  const float as0, const float as1, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendArrow(dd, x0,y0,z0, x1,y1,z1, as0, as1, col);
	dd->end();
}

void duDebugDrawCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					   const float r, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendCircle(dd, x,y,z, r, col);
	dd->end();
}

void duDebugDrawCross(struct duDebugDraw* dd, const float x, const float y, const float z,
					  const float size, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendCross(dd, x,y,z, size, col);
	dd->end();
}

void duDebugDrawBox(struct duDebugDraw* dd, float minx, float miny, float minz,
					float maxx, float maxy, float maxz, const unsigned int* fcol)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_QUADS);
	duAppendBox(dd, minx,miny,minz, maxx,maxy,maxz, fcol);
	dd->end();
}

void duDebugDrawCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
						 float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_TRIS);
	duAppendCylinder(dd, minx,miny,minz, maxx,maxy,maxz, col);
	dd->end();
}

void duDebugDrawGridXZ(struct duDebugDraw* dd, const float ox, const float oy, const float oz,
					   const int w, const int h, const float size,
					   const unsigned int col, const float lineWidth)
{
	if (!dd) return;

	dd->begin(DU_DRAW_LINES, lineWidth);
	for (int i = 0; i <= h; ++i)
	{
		dd->vertex(ox,oy,oz+i*size, col);
		dd->vertex(ox+w*size,oy,oz+i*size, col);
	}
	for (int i = 0; i <= w; ++i)
	{
		dd->vertex(ox+i*size,oy,oz, col);
		dd->vertex(ox+i*size,oy,oz+h*size, col);
	}
	dd->end();
}
		 

void duAppendCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						  float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;

	static const int NUM_SEG = 16;
	static float dir[NUM_SEG*2];
	static bool init = false;
	if (!init)
	{
		init = true;
		for (int i = 0; i < NUM_SEG; ++i)
		{
			const float a = (float)i/(float)NUM_SEG*DU_PI*2;
			dir[i*2] = dtMathCosf(a);
			dir[i*2+1] = dtMathSinf(a);
		}
	}
	
	const float cx = (maxx + minx)/2;
	const float cz = (maxz + minz)/2;
	const float rx = (maxx - minx)/2;
	const float rz = (maxz - minz)/2;
	
	for (int i = 0, j = NUM_SEG-1; i < NUM_SEG; j = i++)
	{
		dd->vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
	}
	for (int i = 0; i < NUM_SEG; i += NUM_SEG/4)
	{
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
	}
}

void duAppendBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
					 float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	// Top
	dd->vertex(minx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, minz, col);
	
	// bottom
	dd->vertex(minx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, minz, col);
	
	// Sides
	dd->vertex(minx, miny, minz, col);
	dd->vertex(minx, maxy, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
}

void duAppendBoxPoints(struct duDebugDraw* dd, float minx, float miny, float minz,
					   float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	// Top
	dd->vertex(minx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, minz, col);
	
	// bottom
	dd->vertex(minx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, minz, col);
}

void duAppendBox(struct duDebugDraw* dd, float minx, float miny, float minz,
				 float maxx, float maxy, float maxz, const unsigned int* fcol)
{
	if (!dd) return;
	dtCoordinates verts[8];
	verts[0] = dtCoordinates( minx, miny, minz );
	verts[1] = dtCoordinates( maxx, miny, minz );
	verts[2] = dtCoordinates( maxx, miny, maxz );
	verts[3] = dtCoordinates( minx, miny, maxz );
	verts[4] = dtCoordinates( minx, maxy, minz );
	verts[5] = dtCoordinates( maxx, maxy, minz );
	verts[6] = dtCoordinates( maxx, maxy, maxz );
	verts[7] = dtCoordinates( minx, maxy, maxz );
	static const unsigned char inds[6*4] =
	{
		7, 6, 5, 4,
		0, 1, 2, 3,
		1, 5, 6, 2,
		3, 7, 4, 0,
		2, 6, 7, 3,
		0, 4, 5, 1,
	};
	
	const unsigned char* in = inds;
	for (int i = 0; i < 6; ++i)
	{
		dd->vertex(verts[*in], fcol[i]); in++;
		dd->vertex(verts[*in], fcol[i]); in++;
		dd->vertex(verts[*in], fcol[i]); in++;
		dd->vertex(verts[*in], fcol[i]); in++;
	}
}

void duAppendCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
					  float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	
	static const int NUM_SEG = 16;
	static float dir[NUM_SEG*2];
	static bool init = false;
	if (!init)
	{
		init = true;
		for (int i = 0; i < NUM_SEG; ++i)
		{
			const float a = (float)i/(float)NUM_SEG*DU_PI*2;
			dir[i*2] = cosf(a);
			dir[i*2+1] = sinf(a);
		}
	}
	
	unsigned int col2 = duMultCol(col, 160);
	
	const float cx = (maxx + minx)/2;
	const float cz = (maxz + minz)/2;
	const float rx = (maxx - minx)/2;
	const float rz = (maxz - minz)/2;

	for (int i = 2; i < NUM_SEG; ++i)
	{
		const int a = 0, b = i-1, c = i;
		dd->vertex(cx+dir[a*2+0]*rx, miny, cz+dir[a*2+1]*rz, col2);
		dd->vertex(cx+dir[b*2+0]*rx, miny, cz+dir[b*2+1]*rz, col2);
		dd->vertex(cx+dir[c*2+0]*rx, miny, cz+dir[c*2+1]*rz, col2);
	}
	for (int i = 2; i < NUM_SEG; ++i)
	{
		const int a = 0, b = i, c = i-1;
		dd->vertex(cx+dir[a*2+0]*rx, maxy, cz+dir[a*2+1]*rz, col);
		dd->vertex(cx+dir[b*2+0]*rx, maxy, cz+dir[b*2+1]*rz, col);
		dd->vertex(cx+dir[c*2+0]*rx, maxy, cz+dir[c*2+1]*rz, col);
	}
	for (int i = 0, j = NUM_SEG-1; i < NUM_SEG; j = i++)
	{
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2);
		dd->vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col2);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);

		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
	}
}


inline void evalArc(const float x0, const float y0, const float z0,
					const float dx, const float dy, const float dz,
					const float h, const float u, dtCoordinates& res)
{
	res.SetX( x0 + dx * u );
	res.SetY( y0 + dy * u + h * (1-(u*2-1)*(u*2-1)) );
	res.SetZ( z0 + dz * u );
}


inline void vcross(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2)
{
	dest.SetX( v1.Y()*v2.Z() - v1.Z()*v2.Y() );
	dest.SetY( v1.Z()*v2.X() - v1.X()*v2.Z() );
	dest.SetZ( v1.X()*v2.Y() - v1.Y()*v2.Z()); 
}

inline void vnormalize(dtCoordinates& v)
{
	float d = 1.0f / sqrtf(v.X()*v.X() + v.Y()*v.Y() + v.Z()*v.Z());
	v.SetX( v.X() * d );
	v.SetY( v.Y() * d );
	v.SetZ( v.Z() * d );
}

inline void vsub(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2)
{
	dest.SetX( v1.X()-v2.X() );
	dest.SetY( v1.Y()-v2.Y() );
	dest.SetZ( v1.Z()-v2.Z() );
}

inline float vdistSqr(const dtCoordinates& v1, const dtCoordinates& v2)
{
	const float x = v1.X()-v2.X();
	const float y = v1.Y()-v2.Y();
	const float z = v1.Z()-v2.Z();
	return x*x + y*y + z*z;
}


void appendArrowHead(struct duDebugDraw* dd, const dtCoordinates& p, const dtCoordinates& q,
					 const float s, unsigned int col)
{
	const float eps = 0.001f;
	if (!dd) return;
	if (vdistSqr(p,q) < eps*eps) return;
	dtCoordinates ax, ay( 0,1,0 ), az;
	vsub(az, q, p);
	vnormalize(az);
	vcross(ax, ay, az);
	vcross(ay, az, ax);
	vnormalize(ay);

	dd->vertex(p, col);
//	dd->vertex(p[0]+az[0]*s+ay[0]*s/2, p[1]+az[1]*s+ay[1]*s/2, p[2]+az[2]*s+ay[2]*s/2, col);
	dd->vertex(p.X()+az.X()*s+ax.X()*s/3, p.Y()+az.Y()*s+ax.Y()*s/3, p.Z()+az.Z()*s+ax.Z()*s/3, col);

	dd->vertex(p, col);
//	dd->vertex(p[0]+az[0]*s-ay[0]*s/2, p[1]+az[1]*s-ay[1]*s/2, p[2]+az[2]*s-ay[2]*s/2, col);
	dd->vertex(p.X()+az.X()*s-ax.X()*s/3, p.Y()+az.Y()*s-ax.Y()*s/3, p.Z()+az.Z()*s-ax.Z()*s/3, col);
	
}

void duAppendArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				 const float x1, const float y1, const float z1, const float h,
				 const float as0, const float as1, unsigned int col)
{
	if (!dd) return;
	static const int NUM_ARC_PTS = 8;
	static const float PAD = 0.05f;
	static const float ARC_PTS_SCALE = (1.0f-PAD*2) / (float)NUM_ARC_PTS;
	const float dx = x1 - x0;
	const float dy = y1 - y0;
	const float dz = z1 - z0;
	const float len = sqrtf(dx*dx + dy*dy + dz*dz);
	dtCoordinates prev;
	evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD, prev);
	for (int i = 1; i <= NUM_ARC_PTS; ++i)
	{
		const float u = PAD + i * ARC_PTS_SCALE;
		dtCoordinates pt;
		evalArc(x0,y0,z0, dx,dy,dz, len*h, u, pt);
		dd->vertex(prev.X(),prev.Y(),prev.Z(), col);
		dd->vertex(pt.X(),pt.Y(),pt.Z(), col);
		prev.SetX( pt.X() ); prev.SetY( pt.Y() ); prev.SetZ( pt.Z() );
	}
	
	// End arrows
	if (as0 > 0.001f)
	{
		dtCoordinates p, q;
		evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD, p);
		evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD+0.05f, q);
		appendArrowHead(dd, p, q, as0, col);
	}

	if (as1 > 0.001f)
	{
		dtCoordinates p, q;
		evalArc(x0,y0,z0, dx,dy,dz, len*h, 1-PAD, p);
		evalArc(x0,y0,z0, dx,dy,dz, len*h, 1-(PAD+0.05f), q);
		appendArrowHead(dd, p, q, as1, col);
	}
}

void duAppendArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				   const float x1, const float y1, const float z1,
				   const float as0, const float as1, unsigned int col)
{
	if (!dd) return;

	dd->vertex(x0,y0,z0, col);
	dd->vertex(x1,y1,z1, col);
	
	// End arrows
	const dtCoordinates p( x0,y0,z0 ), q( x1,y1,z1 );
	if (as0 > 0.001f)
		appendArrowHead(dd, p, q, as0, col);
	if (as1 > 0.001f)
		appendArrowHead(dd, q, p, as1, col);
}

void duAppendCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					const float r, unsigned int col)
{
	if (!dd) return;
	static const int NUM_SEG = 40;
	static float dir[40*2];
	static bool init = false;
	if (!init)
	{
		init = true;
		for (int i = 0; i < NUM_SEG; ++i)
		{
			const float a = (float)i/(float)NUM_SEG*DU_PI*2;
			dir[i*2] = cosf(a);
			dir[i*2+1] = sinf(a);
		}
	}
	
	for (int i = 0, j = NUM_SEG-1; i < NUM_SEG; j = i++)
	{
		dd->vertex(x+dir[j*2+0]*r, y, z+dir[j*2+1]*r, col);
		dd->vertex(x+dir[i*2+0]*r, y, z+dir[i*2+1]*r, col);
	}
}

void duAppendCross(struct duDebugDraw* dd, const float x, const float y, const float z,
				   const float s, unsigned int col)
{
	if (!dd) return;
	dd->vertex(x-s,y,z, col);
	dd->vertex(x+s,y,z, col);
	dd->vertex(x,y-s,z, col);
	dd->vertex(x,y+s,z, col);
	dd->vertex(x,y,z-s, col);
	dd->vertex(x,y,z+s, col);
}

duDisplayList::duDisplayList(int cap) :
	m_pos(0),
	m_color(0),
	m_size(0),
	m_cap(0),
	m_depthMask(true),
	m_prim(DU_DRAW_LINES),
	m_primSize(1.0f)
{
	if (cap < 8)
		cap = 8;
	resize(cap);
}

duDisplayList::~duDisplayList()
{
	delete [] m_pos;
	delete [] m_color;
}

void duDisplayList::resize(int cap)
{
	dtCoordinates* newPos = new dtCoordinates[cap];
	if (m_size)
		memcpy(newPos, m_pos, sizeof(dtCoordinates)*m_size);
	delete [] m_pos;
	m_pos = newPos;

	unsigned int* newColor = new unsigned int[cap];
	if (m_size)
		memcpy(newColor, m_color, sizeof(unsigned int)*m_size);
	delete [] m_color;
	m_color = newColor;
	
	m_cap = cap;
}

void duDisplayList::clear()
{
	m_size = 0;
}

void duDisplayList::depthMask(bool state)
{
	m_depthMask = state;
}

void duDisplayList::begin(duDebugDrawPrimitives prim, float size)
{
	clear();
	m_prim = prim;
	m_primSize = size;
}

void duDisplayList::vertex(const float x, const float y, const float z, unsigned int color)
{
	if (m_size+1 >= m_cap)
		resize(m_cap*2);
	dtCoordinates* p = &m_pos[m_size];
	p->SetX( x );
	p->SetY( y );
	p->SetZ( z );
	m_color[m_size] = color;
	m_size++;
}

void duDisplayList::vertex(const float* pos, unsigned int color)
{
	vertex(pos[0],pos[1],pos[2],color);
}

void duDisplayList::vertex(const dtCoordinates& pos, unsigned int color)
{
	vertex(pos.X(),pos.Y(),pos.Z(),color);
}

void duDisplayList::end()
{
}

void duDisplayList::draw(struct duDebugDraw* dd)
{
	if (!dd) return;
	if (!m_size) return;
	dd->depthMask(m_depthMask);
	dd->begin(m_prim, m_primSize);
	for (int i = 0; i < m_size; ++i)
		dd->vertex(m_pos[i], m_color[i]);
	dd->end();
}
