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

#include <float.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// @par 
/// 
/// Basically, any spans that are closer to a boundary or obstruction than the specified radius 
/// are marked as unwalkable.
///
/// This method is usually called immediately after the heightfield has been built.
///
/// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
bool rcErodeWalkableArea(rcContext* ctx, int radius, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	
	ctx->startTimer(RC_TIMER_ERODE_AREA);
	
	unsigned char* dist = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dist)
	{
		ctx->log(RC_LOG_ERROR, "erodeWalkableArea: Out of memory 'dist' (%d).", chf.spanCount);
		return false;
	}
	
	// Init distance.
	memset(dist, 0xff, sizeof(unsigned char)*chf.spanCount);
	
	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
#ifdef MODIFY_VOXEL_MOLD
				if( !rcCanMovableArea( chf.areas[i] ) ) {
					dist[i] = 0;
					continue;
				}
				const rcCompactSpan& s = chf.spans[i];
				for( int dir = 0; dir < 4; ++dir ) {
					//////////////////////////////////////////////////////////////////////////
					// temporary
					if( rcGetCon(s, dir) == RC_EDGE_CONNECTED ) {
						continue;
					}
					//////////////////////////////////////////////////////////////////////////
					if( rcIsConnectedWalkableArea( s, dir ) ) {
						const int nx = x + rcGetDirOffsetX(dir);
						const int ny = y + rcGetDirOffsetY(dir);
						const int nidx = (int)chf.cells[nx+ny*w].index + rcGetCon(s, dir);
						if( !rcCanMovableArea( chf.areas[nidx] ) ) {
							dist[i] = 0;
							break;
						}
					}
					else {
						dist[i] = 0;
						break;
					}
				}
#else // MODIFY_VOXEL_MOLD
				if (chf.areas[i] == RC_NULL_AREA)
				{
					dist[i] = 0;
				}
				else
				{
					const rcCompactSpan& s = chf.spans[i];
					int nc = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int nx = x + rcGetDirOffsetX(dir);
							const int ny = y + rcGetDirOffsetY(dir);
							const int nidx = (int)chf.cells[nx+ny*w].index + rcGetCon(s, dir);
							if (chf.areas[nidx] != RC_NULL_AREA)
							{
								nc++;
							}
						}
					}
					// At least one missing neighbour.
					if (nc != 4)
						dist[i] = 0;
				}
#endif // MODIFY_VOXEL_MOLD
			}
		}
	}
	
#ifdef MODIFY_VOXEL_MOLD
#else // MODIFY_VOXEL_MOLD
	unsigned char nd;
	
	// Pass 1
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if( rcIsConnectedWalkableArea( s, 0 ) )
				{
					// (-1,0)
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (-1,-1)
					if( rcIsConnectedWalkableArea( as, 3 ) )
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 3);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
				if( rcIsConnectedWalkableArea( s, 3 ) )
				{
					// (0,-1)
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (1,-1)
					if( rcIsConnectedWalkableArea( as, 2 ) )
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 2);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
			}
		}
	}
	
	// Pass 2
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				
				if( rcIsConnectedWalkableArea( s, 2 ) )
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 2);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (1,1)
					if( rcIsConnectedWalkableArea( as, 1 ) )
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 1);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
				if( rcIsConnectedWalkableArea( s, 1 ) )
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 1);
					const rcCompactSpan& as = chf.spans[ai];
					nd = (unsigned char)rcMin((int)dist[ai]+2, 255);
					if (nd < dist[i])
						dist[i] = nd;
					
					// (-1,1)
					if( rcIsConnectedWalkableArea( as, 0 ) )
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 0);
						nd = (unsigned char)rcMin((int)dist[aai]+3, 255);
						if (nd < dist[i])
							dist[i] = nd;
					}
				}
			}
		}
	}
#endif // !MODIFY_VOXEL_MOLD
	
	const unsigned char thr = (unsigned char)(radius*2);
	for (int i = 0; i < chf.spanCount; ++i)
		if ( dist[i] < thr )
			chf.areas[i] = RC_NULL_AREA;
	
	rcFree(dist);
	
	ctx->stopTimer(RC_TIMER_ERODE_AREA);
	
	return true;
}

static void insertSort(unsigned char* a, const int n)
{
	int i, j;
	for (i = 1; i < n; i++)
	{
		const unsigned char value = a[i];
		for (j = i - 1; j >= 0 && a[j] > value; j--)
			a[j+1] = a[j];
		a[j+1] = value;
	}
}

/// @par
///
/// This filter is usually applied after applying area id's using functions
/// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
/// 
/// @see rcCompactHeightfield
bool rcMedianFilterWalkableArea(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	
	ctx->startTimer(RC_TIMER_MEDIAN_AREA);
	
	unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!areas)
	{
		ctx->log(RC_LOG_ERROR, "medianFilterWalkableArea: Out of memory 'areas' (%d).", chf.spanCount);
		return false;
	}
	
	// Init distance.
	memset(areas, 0xff, sizeof(unsigned char)*chf.spanCount);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if( !rcCanMovableArea( chf.areas[i] ) )
				{
					areas[i] = chf.areas[i];
					continue;
				}
				
				unsigned char nei[9];
				for (int j = 0; j < 9; ++j)
					nei[j] = chf.areas[i];
				
				for (int dir = 0; dir < 4; ++dir)
				{
					if( rcIsConnectedWalkableArea( s, dir ) )
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						if( rcCanMovableArea( chf.areas[ai] ) )
							nei[dir*2+0] = chf.areas[ai];
						
						const rcCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir+1) & 0x3;
						if( rcIsConnectedWalkableArea( as, dir2 ) )
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
							if( rcCanMovableArea( chf.areas[ai2] ) )
								nei[dir*2+1] = chf.areas[ai2];
						}
					}
				}
				insertSort(nei, 9);
				areas[i] = nei[4];
			}
		}
	}
	
	memcpy(chf.areas, areas, sizeof(unsigned char)*chf.spanCount);
	
	rcFree(areas);

	ctx->stopTimer(RC_TIMER_MEDIAN_AREA);
	
	return true;
}

/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkBoxArea(rcContext* ctx, const dtCoordinates& bmin, const dtCoordinates& bmax, unsigned char areaId,
				   rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_MARK_BOX_AREA);

	int minx = (int)((bmin.X()-chf.bmin.X())/chf.cs);
	int miny = (int)((bmin.Y()-chf.bmin.Y())/chf.ch);
	int minz = (int)((bmin.Z()-chf.bmin.Z())/chf.cs);
	int maxx = (int)((bmax.X()-chf.bmin.X())/chf.cs);
	int maxy = (int)((bmax.Y()-chf.bmin.Y())/chf.ch);
	int maxz = (int)((bmax.Z()-chf.bmin.Z())/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;

	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				if ((int)s.y >= miny && (int)s.y <= maxy)
				{
					if( rcCanMovableArea( chf.areas[i] ) )
						chf.areas[i] = areaId;
				}
			}
		}
	}

	ctx->stopTimer(RC_TIMER_MARK_BOX_AREA);

}


static int pointInPoly(int nvert, const dtCoordinates* verts, const dtCoordinates& p)
{
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++)
	{
		const dtCoordinates vi( verts[i] );
		const dtCoordinates vj( verts[j] );
		if (((vi.Z() > p.Z()) != (vj.Z() > p.Z())) &&
			(p.X() < (vj.X()-vi.X()) * (p.Z()-vi.Z()) / (vj.Z()-vi.Z()) + vi.X()) )
			c = !c;
	}
	return c;
}

/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// The y-values of the polygon vertices are ignored. So the polygon is effectively 
/// projected onto the xz-plane at @p hmin, then extruded to @p hmax.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkConvexPolyArea(rcContext* ctx, const dtCoordinates* verts, const int nverts,
						  const float hmin, const float hmax, unsigned char areaId,
						  rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_MARK_CONVEXPOLY_AREA);

	dtCoordinates bmin, bmax;
	rcVcopy(bmin, verts[0]);
	rcVcopy(bmax, verts[0]);
	for (int i = 1; i < nverts; ++i)
	{
		rcVmin(bmin, verts[i]);
		rcVmax(bmax, verts[i]);
	}
	bmin.SetY( hmin );
	bmax.SetY( hmax );

	int minx = (int)((bmin.X()-chf.bmin.X())/chf.cs);
	int miny = (int)((bmin.Y()-chf.bmin.Y())/chf.ch);
	int minz = (int)((bmin.Z()-chf.bmin.Z())/chf.cs);
	int maxx = (int)((bmax.X()-chf.bmin.X())/chf.cs);
	int maxy = (int)((bmax.Y()-chf.bmin.Y())/chf.ch);
	int maxz = (int)((bmax.Z()-chf.bmin.Z())/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;
	
	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	
	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				if( !rcCanMovableArea( chf.areas[i] ) )
					continue;
				if ((int)s.y >= miny && (int)s.y <= maxy)
				{
					dtCoordinates p;
					p.SetX( chf.bmin.X() + (x+0.5f)*chf.cs );
					p.SetY( 0 );
					p.SetZ( chf.bmin.Z() + (z+0.5f)*chf.cs );

					if (pointInPoly(nverts, verts, p))
					{
						chf.areas[i] = areaId;
					}
				}
			}
		}
	}

	ctx->stopTimer(RC_TIMER_MARK_CONVEXPOLY_AREA);
}

int rcOffsetPoly(const dtCoordinates* verts, const int nverts, const float offset,
				 dtCoordinates* outVerts, const int maxOutVerts)
{
	const float	MITER_LIMIT = 1.20f;

	int n = 0;

	for (int i = 0; i < nverts; i++)
	{
		const int a = (i+nverts-1) % nverts;
		const int b = i;
		const int c = (i+1) % nverts;
		const dtCoordinates va( verts[a] );
		const dtCoordinates vb( verts[b] );
		const dtCoordinates vc( verts[c] );
		float dx0 = vb.X() - va.X();
		float dy0 = vb.Z() - va.Z();
		float d0 = dx0*dx0 + dy0*dy0;
		if (d0 > 1e-6f)
		{
			d0 = 1.0f/rcSqrt(d0);
			dx0 *= d0;
			dy0 *= d0;
		}
		float dx1 = vc.X() - vb.X();
		float dy1 = vc.Z() - vb.Z();
		float d1 = dx1*dx1 + dy1*dy1;
		if (d1 > 1e-6f)
		{
			d1 = 1.0f/rcSqrt(d1);
			dx1 *= d1;
			dy1 *= d1;
		}
		const float dlx0 = -dy0;
		const float dly0 = dx0;
		const float dlx1 = -dy1;
		const float dly1 = dx1;
		float cross = dx1*dy0 - dx0*dy1;
		float dmx = (dlx0 + dlx1) * 0.5f;
		float dmy = (dly0 + dly1) * 0.5f;
		float dmr2 = dmx*dmx + dmy*dmy;
		bool bevel = dmr2 * MITER_LIMIT*MITER_LIMIT < 1.0f;
		if (dmr2 > 1e-6f)
		{
			const float scale = 1.0f / dmr2;
			dmx *= scale;
			dmy *= scale;
		}

		if (bevel && cross < 0.0f)
		{
			if (n+2 >= maxOutVerts)
				return 0;
			float d = (1.0f - (dx0*dx1 + dy0*dy1))*0.5f;
			outVerts[n].SetX( vb.X() + (-dlx0+dx0*d)*offset );
			outVerts[n].SetY( vb.Y() );
			outVerts[n].SetZ( vb.Z() + (-dly0+dy0*d)*offset );
			n++;
			outVerts[n].SetX( vb.X() + (-dlx1-dx1*d)*offset );
			outVerts[n].SetY( vb.Y() );
			outVerts[n].SetZ( vb.Z() + (-dly1-dy1*d)*offset );
			n++;
		}
		else
		{
			if (n+1 >= maxOutVerts)
				return 0;
			outVerts[n].SetX( vb.X() - dmx*offset );
			outVerts[n].SetY( vb.Y() );
			outVerts[n].SetZ( vb.Z() - dmy*offset );
			n++;
		}
	}
	
	return n;
}


/// @par
///
/// The value of spacial parameters are in world units.
/// 
/// @see rcCompactHeightfield, rcMedianFilterWalkableArea
void rcMarkCylinderArea(rcContext* ctx, const dtCoordinates& pos,
						const float r, const float h, unsigned char areaId,
						rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_MARK_CYLINDER_AREA);
	
	dtCoordinates bmin, bmax;
	bmin.SetX( pos.X() - r );
	bmin.SetY( pos.Y() );
	bmin.SetZ( pos.Z() - r );
	bmax.SetX( pos.X() + r );
	bmax.SetY( pos.Y() + h );
	bmax.SetZ( pos.Z() + r );
	const float r2 = r*r;
	
	int minx = (int)((bmin.X()-chf.bmin.X())/chf.cs);
	int miny = (int)((bmin.Y()-chf.bmin.Y())/chf.ch);
	int minz = (int)((bmin.Z()-chf.bmin.Z())/chf.cs);
	int maxx = (int)((bmax.X()-chf.bmin.X())/chf.cs);
	int maxy = (int)((bmax.Y()-chf.bmin.Y())/chf.ch);
	int maxz = (int)((bmax.Z()-chf.bmin.Z())/chf.cs);
	
	if (maxx < 0) return;
	if (minx >= chf.width) return;
	if (maxz < 0) return;
	if (minz >= chf.height) return;
	
	if (minx < 0) minx = 0;
	if (maxx >= chf.width) maxx = chf.width-1;
	if (minz < 0) minz = 0;
	if (maxz >= chf.height) maxz = chf.height-1;	
	
	
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const rcCompactCell& c = chf.cells[x+z*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
				
				if( !rcCanMovableArea( chf.areas[i] ) )
					continue;
				
				if ((int)s.y >= miny && (int)s.y <= maxy)
				{
					const float sx = chf.bmin.X() + (x+0.5f)*chf.cs; 
					const float sz = chf.bmin.Z() + (z+0.5f)*chf.cs; 
					const float dx = sx - pos.X();
					const float dz = sz - pos.Z();
					
					if (dx*dx + dz*dz < r2)
					{
						chf.areas[i] = areaId;
					}
				}
			}
		}
	}
	
	ctx->stopTimer(RC_TIMER_MARK_CYLINDER_AREA);
}
