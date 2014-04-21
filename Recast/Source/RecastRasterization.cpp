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
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

inline bool overlapBounds(const dtCoordinates& amin, const dtCoordinates& amax, const dtCoordinates& bmin, const dtCoordinates& bmax)
{
	bool overlap = true;
	overlap = (amin.X() > bmax.X() || amax.X() < bmin.X()) ? false : overlap;
	overlap = (amin.Y() > bmax.Y() || amax.Y() < bmin.Y()) ? false : overlap;
	overlap = (amin.Z() > bmax.Z() || amax.Z() < bmin.Z()) ? false : overlap;
	return overlap;
}

inline bool overlapInterval(unsigned short amin, unsigned short amax,
							unsigned short bmin, unsigned short bmax)
{
	if (amax < bmin) return false;
	if (amin > bmax) return false;
	return true;
}


static rcSpan* allocSpan(rcHeightfield& hf)
{
	// If running out of memory, allocate new page and update the freelist.
	if (!hf.freelist || !hf.freelist->next)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* pool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (!pool) return 0;
		pool->next = 0;
		// Add the pool into the list of pools.
		pool->next = hf.pools;
		hf.pools = pool;
		// Add new items to the free list.
		rcSpan* freelist = hf.freelist;
		rcSpan* head = &pool->items[0];
		rcSpan* it = &pool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freelist;
			freelist = it;
		}
		while (it != head);
		hf.freelist = it;
	}
	
	// Pop item from in front of the free list.
	rcSpan* it = hf.freelist;
	hf.freelist = hf.freelist->next;
	return it;
}

static void freeSpan(rcHeightfield& hf, rcSpan* ptr)
{
	if (!ptr) return;
	// Add the node in front of the free list.
	ptr->next = hf.freelist;
	hf.freelist = ptr;
}

static void addSpan(rcHeightfield& hf, const int x, const int y,
	const unsigned short smin, const unsigned short smax,
	const unsigned char area, const int flagMergeThr)
{
	const int idx = x + y*hf.width;

	rcSpan* s = allocSpan(hf);
	s->smin = smin;
	s->smax = smax;
	s->area = area;
	s->next = NULL;

	// Empty cell, add the first span.
	if( hf.spans[idx] == NULL ) {
		hf.spans[idx] = s;
		return;
	}
	rcSpan* prev = 0;
	rcSpan* cur = hf.spans[idx];

	// Insert and merge spans.
	do {
		if (cur->smin > s->smax)
		{
#ifdef MODIFY_VOXEL_FLAG
			if( rcIsTerrainArea( cur->area ) ) {
				freeSpan( hf, s );
				return;
			}
			else {
				break;
			}
#else // MODIFY_VOXEL_FLAG
			// Current span is further than the new span, break.
			break;
#endif // MODIFY_VOXEL_FLAG
		}
		else if (cur->smax < s->smin)
		{
#ifdef MODIFY_VOXEL_FLAG
			if( rcIsTerrainArea( s->area ) ) {
				rcSpan* next = cur->next;
				freeSpan( hf, cur );
				if( prev != NULL ) {
					prev->next = next;
				}
				else {
					hf.spans[idx] = next;
				}
				cur = next;
			}
			else {
				prev = cur;
				cur = cur->next;
			}
#else // MODIFY_VOXEL_FLAG
			// Current span is before the new span advance.
			prev = cur;
			cur = cur->next;
#endif // MODIFY_VOXEL_FLAG
		}
		else
		{
#ifdef MODIFY_VOXEL_FLAG
			//////////////////////////////////////////////////////////////////////////
			if( rcIsSimilarTypeArea( s->area, cur->area ) ) {
				// merge
				s->smin = rcMin( s->smin, cur->smin );
				s->smax = rcMax( s->smax, cur->smax );
				if( rcAbs( static_cast<int>( s->smax ) - static_cast<int>( cur->smax ) ) <= flagMergeThr ) {
					s->area = rcMax( s->area, cur->area );

// 					const bool walkable = rcIsWalkableArea( s->area ) && rcIsWalkableArea( cur->area );
// 					s->area = walkable ? s->area | RC_WALKABLE_AREA : s->area | RC_UNWALKABLE_AREA;
				}
			}
			else {
				s->smin = rcIsTerrainArea( s->area ) ? s->smin : cur->smin;
				s->smax = rcMax( s->smax, cur->smax );
				if( rcAbs( static_cast<int>( s->smax ) - static_cast<int>( cur->smax ) ) <= flagMergeThr ) {
					if( rcIsWalkableArea( s->area ) || rcIsWalkableArea( cur->area ) ) {
						s->area = RC_TERRAIN_AREA | RC_OBJECT_AREA | RC_WALKABLE_AREA;
					}
					else {
						s->area = RC_TERRAIN_AREA | RC_OBJECT_AREA | RC_UNWALKABLE_AREA;
					}

// 					bool walkable = false;
// 					walkable = rcIsObjectArea( s->area ) && rcIsWalkableArea( s->area ) ? true : walkable;
// 					walkable = rcIsObjectArea( cur->area ) && rcIsWalkableArea( cur->area ) ? true : walkable;
// 					s->area = walkable ? RC_OBJECT_AREA | RC_WALKABLE_AREA : RC_OBJECT_AREA | RC_UNWALKABLE_AREA;
				}
			}
			//////////////////////////////////////////////////////////////////////////
#else // MODIFY_VOXEL_FLAG
			// Merge spans.
			if (cur->smin < s->smin)
				s->smin = cur->smin;
			if (cur->smax > s->smax)
				s->smax = cur->smax;

			// Merge flags.
			if (rcAbs((int)s->smax - (int)cur->smax) <= flagMergeThr) {
// #ifdef MODIFY_VOXEL_FLAG
// 				if( rcIsSimilarTypeArea( s->area, cur->area ) ) {
// 					s->area = rcMax(s->area, cur->area);
// 				}
// 				else {
// 					if( rcIsWalkableArea( s->area ) || rcIsWalkableArea( cur->area ) ) {
// 						s->area = RC_OBJECT_AREA | RC_WALKABLE_AREA;
// 					}
// 					else {
// 						s->area = RC_OBJECT_AREA | RC_UNWALKABLE_AREA;
// 					}
// 				}
// #else // MODIFY_VOXEL_FLAG
				s->area = rcMax(s->area, cur->area);
//#endif // MODIFY_VOXEL_FLAG
			}
#endif // MODIFY_VOXEL_FLAG

			// Remove current span.
			rcSpan* next = cur->next;
			freeSpan(hf, cur);
			if (prev)
				prev->next = next;
			else
				hf.spans[idx] = next;
			cur = next;
		}
	} while( cur != NULL );

	// Insert new span.
	if( prev != NULL ) {
		s->next = prev->next;
		prev->next = s;
	}
	else {
		s->next = hf.spans[idx];
		hf.spans[idx] = s;
	}
}

// divides a convex polygons into two convex polygons on both sides of a line
static void dividePoly(const dtCoordinates* in, int nin,
					  dtCoordinates* out1, int* nout1,
					  dtCoordinates* out2, int* nout2,
					  float x, int axis)
{
	float d[4*3];
	for (int i = 0; i < nin; ++i) {
		if( axis == 0 ) {
			d[i] = x - in[i].X();
		}
		else if( axis == 1 ) {
			d[i] = x - in[i].Y();
		}
		else if( axis == 2 ) {
			d[i] = x - in[i].Z();
		}
	}
	
	int m = 0, n = 0;
	for (int i = 0, j = nin-1; i < nin; j=i, ++i)
	{
		bool ina = d[j] >= 0;
		bool inb = d[i] >= 0;
		if (ina != inb)
		{
			float s = d[j] / (d[j] - d[i]);
			out1[m].SetX( in[j].X() + (in[i].X() - in[j].X())*s );
			out1[m].SetY( in[j].Y() + (in[i].Y() - in[j].Y())*s );
			out1[m].SetZ( in[j].Z() + (in[i].Z() - in[j].Z())*s );
			rcVcopy(out2[n], out1[m]);
			m++;
			n++;
			// add the i'th point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (d[i] > 0)
			{
				rcVcopy(out1[m], in[i]);
				m++;
			}
			else if (d[i] < 0)
			{
				rcVcopy(out2[n], in[i]);
				n++;
			}
		}
		else // same side
		{
			// add the i'th point to the right polygon. Addition is done even for points on the dividing line
			if (d[i] >= 0)
			{
				rcVcopy(out1[m], in[i]);
				m++;
				if (d[i] != 0)
					continue;
			}
			rcVcopy(out2[n], in[i]);
			n++;
		}
	}

	*nout1 = m;
	*nout2 = n;
}



static void rasterizeTri(const dtCoordinates& v0, const dtCoordinates& v1, const dtCoordinates& v2,
						 const unsigned char area, rcHeightfield& hf,
						 const dtCoordinates& bmin, const dtCoordinates& bmax,
						 const float cs, const float ics, const float ich,
						 const int flagMergeThr)
{
	const int w = hf.width;
	const int h = hf.height;
	dtCoordinates tmin, tmax;
	const float by = bmax.Y() - bmin.Y();
	
	// Calculate the bounding box of the triangle.
	rcVcopy(tmin, v0);
	rcVcopy(tmax, v0);
	rcVmin(tmin, v1);
	rcVmin(tmin, v2);
	rcVmax(tmax, v1);
	rcVmax(tmax, v2);

	// If the triangle does not touch the bbox of the heightfield, skip the triagle.
	if (!overlapBounds(bmin, bmax, tmin, tmax))
		return;
	
	// Calculate the footprint of the triangle on the grid's y-axis
	int y0 = (int)((tmin.Z() - bmin.Z())*ics);
	int y1 = (int)/*ceilf*/((tmax.Z() - bmin.Z())*ics);
	y0 = rcClamp(y0, 0, h-1);
	y1 = rcClamp(y1, 0, h-1);
	
	// Clip the triangle into all grid cells it touches.
	dtCoordinates buf[7*4];
	dtCoordinates *in = buf, *inrow = buf+7, *p1 = inrow+7, *p2 = p1+7;
	
	rcVcopy(in[0], v0);
	rcVcopy(in[1], v1);
	rcVcopy(in[2], v2);
	int nvrow, nvIn = 3;
	
	for (int y = y0; y <= y1; ++y)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cz = bmin.Z() + y*cs;
		dividePoly(in, nvIn, inrow, &nvrow, p1, &nvIn, cz+cs, 2);
		rcSwap(in, p1);
		if (nvrow < 3) continue;
		
		// find the horizontal bounds in the row
		float minX = inrow[0].X(), maxX = inrow[0].X();
		for (int i=1; i<nvrow; ++i)
		{
			if (minX > inrow[i].X())	minX = inrow[i].X();
			if (maxX < inrow[i].X())	maxX = inrow[i].X();
		}
		int x0 = (int)((minX - bmin.X())*ics);
		int x1 = (int)/*ceilf*/((maxX - bmin.X())*ics);
		x0 = rcClamp(x0, 0, w-1);
		x1 = rcClamp(x1, 0, w-1);

		int nv, nv2 = nvrow;
		
		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = bmin.X() + x*cs;
			dividePoly(inrow, nv2, p1, &nv, p2, &nv2, cx+cs, 0);
			rcSwap(inrow, p2);
			if (nv < 3) continue;

			// Calculate min and max of the span.
			float smin = p1[0].Y(), smax = p1[0].Y();
			for (int i = 1; i < nv; ++i)
			{
				smin = rcMin(smin, p1[i].Y());
				smax = rcMax(smax, p1[i].Y());
			}
			smin -= bmin.Y();
			smax -= bmin.Y();
			// Skip the span if it is outside the heightfield bbox
			if (smax < 0.0f) continue;
			if (smin > by) continue;
			// Clamp the span to the heightfield bbox.
			if (smin < 0.0f) smin = 0;
			if (smax > by) smax = by;
			
			// Snap the span to the heightfield height grid.
			unsigned short ismin = (unsigned short)rcClamp((int)floorf(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short ismax = (unsigned short)rcClamp((int)ceilf(smax * ich), (int)ismin+1, RC_SPAN_MAX_HEIGHT);
			
			addSpan(hf, x, y, ismin, ismax, area, flagMergeThr);
		}
	}
}

/// @par
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
// void rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
// 						 const unsigned char area, rcHeightfield& solid,
// 						 const int flagMergeThr)
// {
// 	rcAssert(ctx);
// 
// 	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
// 
// 	const float ics = 1.0f/solid.cs;
// 	const float ich = 1.0f/solid.ch;
// 	rasterizeTri(v0, v1, v2, area, solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
// 
// 	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
// }

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const dtCoordinates* verts, const int /*nv*/,
						  const int* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const dtCoordinates v0( verts[tris[i*3+0]] );
		const dtCoordinates v1( verts[tris[i*3+1]] );
		const dtCoordinates v2( verts[tris[i*3+2]] );
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
// void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
// 						  const unsigned short* tris, const unsigned char* areas, const int nt,
// 						  rcHeightfield& solid, const int flagMergeThr)
// {
// 	rcAssert(ctx);
// 
// 	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
// 	
// 	const float ics = 1.0f/solid.cs;
// 	const float ich = 1.0f/solid.ch;
// 	// Rasterize triangles.
// 	for (int i = 0; i < nt; ++i)
// 	{
// 		const float* v0 = &verts[tris[i*3+0]*3];
// 		const float* v1 = &verts[tris[i*3+1]*3];
// 		const float* v2 = &verts[tris[i*3+2]*3];
// 		// Rasterize.
// 		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
// 	}
// 	
// 	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
// }

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
// void rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt,
// 						  rcHeightfield& solid, const int flagMergeThr)
// {
// 	rcAssert(ctx);
// 	
// 	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
// 	
// 	const float ics = 1.0f/solid.cs;
// 	const float ich = 1.0f/solid.ch;
// 	// Rasterize triangles.
// 	for (int i = 0; i < nt; ++i)
// 	{
// 		const float* v0 = &verts[(i*3+0)*3];
// 		const float* v1 = &verts[(i*3+1)*3];
// 		const float* v2 = &verts[(i*3+2)*3];
// 		// Rasterize.
// 		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
// 	}
// 	
// 	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
// }
