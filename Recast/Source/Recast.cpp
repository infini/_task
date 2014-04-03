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
#include <stdarg.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

float rcSqrt(float x)
{
	return sqrtf(x);
}

/// @class rcContext
/// @par
///
/// This class does not provide logging or timer functionality on its 
/// own.  Both must be provided by a concrete implementation 
/// by overriding the protected member functions.  Also, this class does not 
/// provide an interface for extracting log messages. (Only adding them.) 
/// So concrete implementations must provide one.
///
/// If no logging or timers are required, just pass an instance of this 
/// class through the Recast build process.
///

/// @par
///
/// Example:
/// @code
/// // Where ctx is an instance of rcContext and filepath is a char array.
/// ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
/// @endcode
void rcContext::log(const rcLogCategory category, const char* format, ...)
{
	if (!m_logEnabled)
		return;
	static const int MSG_SIZE = 512;
	char msg[MSG_SIZE];
	va_list ap;
	va_start(ap, format);
	int len = vsnprintf(msg, MSG_SIZE, format, ap);
	if (len >= MSG_SIZE)
	{
		len = MSG_SIZE-1;
		msg[MSG_SIZE-1] = '\0';
	}
	va_end(ap);
	doLog(category, msg, len);
}

rcHeightfield* rcAllocHeightfield()
{
	rcHeightfield* hf = (rcHeightfield*)rcAlloc(sizeof(rcHeightfield), RC_ALLOC_PERM);
	memset(hf, 0, sizeof(rcHeightfield));
	return hf;
}

void rcFreeHeightField(rcHeightfield* hf)
{
	if (!hf) return;
	// Delete span array.
	rcFree(hf->spans);
	// Delete span pools.
	while (hf->pools)
	{
		rcSpanPool* next = hf->pools->next;
		rcFree(hf->pools);
		hf->pools = next;
	}
	rcFree(hf);
}

rcCompactHeightfield* rcAllocCompactHeightfield()
{
	rcCompactHeightfield* chf = (rcCompactHeightfield*)rcAlloc(sizeof(rcCompactHeightfield), RC_ALLOC_PERM);
	memset(chf, 0, sizeof(rcCompactHeightfield));
	return chf;
}

void rcFreeCompactHeightfield(rcCompactHeightfield* chf)
{
	if (!chf) return;
	rcFree(chf->cells);
	rcFree(chf->spans);
	rcFree(chf->dist);
	rcFree(chf->areas);
	rcFree(chf);
}


rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet()
{
	rcHeightfieldLayerSet* lset = (rcHeightfieldLayerSet*)rcAlloc(sizeof(rcHeightfieldLayerSet), RC_ALLOC_PERM);
	memset(lset, 0, sizeof(rcHeightfieldLayerSet));
	return lset;
}

void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* lset)
{
	if (!lset) return;
	for (int i = 0; i < lset->nlayers; ++i)
	{
		rcFree(lset->layers[i].heights);
		rcFree(lset->layers[i].areas);
		rcFree(lset->layers[i].cons);
	}
	rcFree(lset->layers);
	rcFree(lset);
}


rcContourSet* rcAllocContourSet()
{
	rcContourSet* cset = (rcContourSet*)rcAlloc(sizeof(rcContourSet), RC_ALLOC_PERM);
	memset(cset, 0, sizeof(rcContourSet));
	return cset;
}

void rcFreeContourSet(rcContourSet* cset)
{
	if (!cset) return;
	for (int i = 0; i < cset->nconts; ++i)
	{
		rcFree(cset->conts[i].verts);
		rcFree(cset->conts[i].rverts);
	}
	rcFree(cset->conts);
	rcFree(cset);
}

rcPolyMesh* rcAllocPolyMesh()
{
	rcPolyMesh* pmesh = (rcPolyMesh*)rcAlloc(sizeof(rcPolyMesh), RC_ALLOC_PERM);
	memset(pmesh, 0, sizeof(rcPolyMesh));
	return pmesh;
}

void rcFreePolyMesh(rcPolyMesh* pmesh)
{
	if (!pmesh) return;
	rcFree(pmesh->verts);
	rcFree(pmesh->polys);
	rcFree(pmesh->regs);
	rcFree(pmesh->flags);
	rcFree(pmesh->areas);
	rcFree(pmesh);
}

rcPolyMeshDetail* rcAllocPolyMeshDetail()
{
	rcPolyMeshDetail* dmesh = (rcPolyMeshDetail*)rcAlloc(sizeof(rcPolyMeshDetail), RC_ALLOC_PERM);
	memset(dmesh, 0, sizeof(rcPolyMeshDetail));
	return dmesh;
}

void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh)
{
	if (!dmesh) return;
	rcFree(dmesh->meshes);
	rcFree(dmesh->verts);
	rcFree(dmesh->tris);
	rcFree(dmesh);
}

#ifdef MODIFY_SQUARE_SECTOR
void rcCalcBounds( const float* verts, const int nv, const float* square_min, const float* square_max, float* bmin, float* bmax )
{
	// Calculate bounding box.
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < nv; ++i)
	{
		const float* v = &verts[i*3];
		rcVmin(bmin, v);
		rcVmax(bmax, v);

		bmin[0] = rcClamp( bmin[0], square_min[0], square_max[0] );
		bmin[2] = rcClamp( bmin[2], square_min[2], square_max[2] );

		bmax[0] = rcClamp( bmax[0], square_min[0], square_max[0] );
		bmax[2] = rcClamp( bmax[2], square_min[2], square_max[2] );
	}
}
#else // MODIFY_SQUARE_SECTOR
void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax)
{
	// Calculate bounding box.
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < nv; ++i)
	{
		const float* v = &verts[i*3];
		rcVmin(bmin, v);
		rcVmax(bmax, v);
	}
}
#endif // MODIFY_SQUARE_SECTOR

void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h)
{
	*w = (int)((bmax[0] - bmin[0])/cs+0.5f);
	*h = (int)((bmax[2] - bmin[2])/cs+0.5f);
}

/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcAllocHeightfield, rcHeightfield 
bool rcCreateHeightfield(rcContext* ctx, rcHeightfield& hf, int width, int height,
						 const float* bmin, const float* bmax,
						 float cs, float ch)
{
	rcIgnoreUnused(ctx);
	
	hf.width = width;
	hf.height = height;
	rcVcopy(hf.bmin, bmin);
	rcVcopy(hf.bmax, bmax);
	hf.cs = cs;
	hf.ch = ch;
	hf.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*)*hf.width*hf.height, RC_ALLOC_PERM);
	if (!hf.spans)
		return false;
	memset(hf.spans, 0, sizeof(rcSpan*)*hf.width*hf.height);
	return true;
}

static void calcTriNormal(const float* v0, const float* v1, const float* v2, float* norm)
{
	float e0[3], e1[3];
	rcVsub(e0, v1, v0);
	rcVsub(e1, v2, v0);
	rcVcross(norm, e0, e1);
	rcVnormalize(norm);
}

/// @par
///
/// Only sets the aread id's for the walkable triangles.  Does not alter the
/// area id's for unwalkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
void rcMarkWalkableTriangles(rcContext* ctx, const float walkableSlopeAngle,
							 const float* verts, int /*nv*/,
							 const int* tris, int nt,
							 unsigned char* areas)
{
	rcIgnoreUnused(ctx);
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*RC_PI);
	float norm[3];
	
	for (int i = 0; i < nt; ++i)
	{
		const int* tri = &tris[i*3];
		calcTriNormal(&verts[tri[0]*3], &verts[tri[1]*3], &verts[tri[2]*3], norm);
//#ifdef MODIFY_VOXEL_FLAG
		const bool terrain = tri[0] < RC_MAX_GROUND_FLOOR_VERTICES && tri[1] < RC_MAX_GROUND_FLOOR_VERTICES && tri[2] < RC_MAX_GROUND_FLOOR_VERTICES;
//#endif // MODIFY_VOXEL_FLAG
		// Check if the face is walkable.
#ifdef MODIFY_VOXEL_FLAG
		if( walkableThr < norm[1] ) {
			areas[i] = terrain ? RC_TERRAIN_AREA | RC_WALKABLE_AREA : RC_OBJECT_AREA | RC_WALKABLE_AREA;
		}
		else {
			areas[i] = terrain ? RC_TERRAIN_AREA | RC_WALKABLE_AREA : RC_OBJECT_AREA | RC_UNWALKABLE_AREA;;
		}
#else // MODIFY_VOXEL_FLAG
		if (norm[1] > walkableThr) {
			//areas[i] = RC_WALKABLE_AREA;
			areas[i] = terrain ? RC_WALKABLE_AREA : 1;
		}
		else {
			areas[i] = terrain ? RC_NULL_AREA : 2;
		}
#endif // MODIFY_VOXEL_FLAG
	}
}

/// @par
///
/// Only sets the aread id's for the unwalkable triangles.  Does not alter the
/// area id's for walkable triangles.
/// 
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 
/// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
void rcClearUnwalkableTriangles(rcContext* ctx, const float walkableSlopeAngle,
								const float* verts, int /*nv*/,
								const int* tris, int nt,
								unsigned char* areas)
{
	rcIgnoreUnused(ctx);
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*RC_PI);
	
	float norm[3];
	
	for (int i = 0; i < nt; ++i)
	{
		const int* tri = &tris[i*3];
		calcTriNormal(&verts[tri[0]*3], &verts[tri[1]*3], &verts[tri[2]*3], norm);
		// Check if the face is walkable.
		if (norm[1] <= walkableThr)
			areas[i] = RC_NULL_AREA;
	}
}

int rcGetHeightFieldSpanCount(rcContext* ctx, rcHeightfield& hf)
{
	rcIgnoreUnused(ctx);
	
	const int w = hf.width;
	const int h = hf.height;
	int spanCount = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = hf.spans[x + y*w]; s; s = s->next)
			{
				if( rcCanMovableArea( s->area ) )
					spanCount++;
			}
		}
	}
	return spanCount;
}

/// @par
///
/// This is just the beginning of the process of fully building a compact heightfield.
/// Various filters may be applied applied, then the distance field and regions built.
/// E.g: #rcBuildDistanceField and #rcBuildRegions
///
/// See the #rcConfig documentation for more information on the configuration parameters.
///
/// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig
bool rcBuildCompactHeightfield(rcContext* ctx, const int walkableHeight, const int walkableClimb,
							   rcHeightfield& hf, rcCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_BUILD_COMPACTHEIGHTFIELD);
	
	const int w = hf.width;
	const int h = hf.height;
	const int spanCount = rcGetHeightFieldSpanCount(ctx, hf);

	// Fill in header.
	chf.width = w;
	chf.height = h;
	chf.spanCount = spanCount;
	chf.walkableHeight = walkableHeight;
	chf.walkableClimb = walkableClimb;
	chf.maxRegions = 0;
	rcVcopy(chf.bmin, hf.bmin);
	rcVcopy(chf.bmax, hf.bmax);
	chf.bmax[1] += walkableHeight*hf.ch;
	chf.cs = hf.cs;
	chf.ch = hf.ch;
	chf.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell)*w*h, RC_ALLOC_PERM);
	if (!chf.cells)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", w*h);
		return false;
	}
	memset(chf.cells, 0, sizeof(rcCompactCell)*w*h);
	chf.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan)*spanCount, RC_ALLOC_PERM);
	if (!chf.spans)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(chf.spans, 0, sizeof(rcCompactSpan)*spanCount);
	chf.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*spanCount, RC_ALLOC_PERM);
	if (!chf.areas)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(chf.areas, RC_NULL_AREA, sizeof(unsigned char)*spanCount);
	
	const int MAX_HEIGHT = 0xffff;
	
	// Fill in cells and spans.
	int idx = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcSpan* s = hf.spans[x + y*w];
			// If there are no spans at this cell, just leave the data to index=0, count=0.
			if (!s) continue;
			rcCompactCell& c = chf.cells[x+y*w];
			c.index = idx;
			c.count = 0;
			while (s)
			{
				if( rcCanMovableArea( s->area ) )
				{
					const int bot = (int)s->smax;
					const int top = s->next ? (int)s->next->smin : MAX_HEIGHT;
					chf.spans[idx].y = (unsigned short)rcClamp(bot, 0, 0xffff);
					chf.spans[idx].h = (unsigned char)rcClamp(top - bot, 0, 0xff);
					chf.areas[idx] = s->area;
					idx++;
					c.count++;
				}
				s = s->next;
			}
		}
	}

	// Find neighbour connections.
	const int MAX_LAYERS = RC_NOT_CONNECTED-1;
	int tooHighNeighbour = 0;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				rcCompactSpan& s = chf.spans[i];
#ifdef MODIFY_VOXEL_FLAG
				const unsigned char area_flag = chf.areas[i];
#endif // MODIFY_VOXEL_FLAG
				
				for (int dir = 0; dir < 4; ++dir)
				{
					rcSetCon(s, dir, RC_NOT_CONNECTED);
					const int nx = x + rcGetDirOffsetX(dir);
					const int ny = y + rcGetDirOffsetY(dir);
					// First check that the neighbour cell is in bounds.
					if( nx < 0 + chf.borderSize/2 || ny < 0 + chf.borderSize/2 || nx >= w - chf.borderSize/2 || ny >= h - chf.borderSize/2 ) {
					//if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
#ifdef MODIFY_VOXEL_MOLD
						rcSetCon( s, dir, RC_EDGE_CONNECTED );
#endif // MODIFY_VOXEL_MOLD
						continue;
					}
						
					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const rcCompactCell& nc = chf.cells[nx+ny*w];
					for (int k = (int)nc.index, nk = (int)(nc.index+nc.count); k < nk; ++k)
					{
						const rcCompactSpan& ns = chf.spans[k];
#ifdef MODIFY_VOXEL_FLAG
						const unsigned char side_area_flag = chf.areas[k];
#endif // MODIFY_VOXEL_FLAG
						const int bot = rcMax(s.y, ns.y);
						const int top = rcMin(s.y+s.h, ns.y+ns.h);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
#ifdef MODIFY_VOXEL_FLAG
						if( (top - bot) >= walkableHeight && rcAbs((int)ns.y - (int)s.y) <= walkableClimb && rcIsSimilarTypeArea(area_flag, side_area_flag) )
#else // MODIFY_VOXEL_FLAG
						if ((top - bot) >= walkableHeight && rcAbs((int)ns.y - (int)s.y) <= walkableClimb)
#endif // MODIFY_VOXEL_FLAG
						{
#ifdef MODIFY_VOXEL_FLAG
							//if( rcIsTerrainArea(area_flag) || (rcIsObjectArea(area_flag) && (top - bot) >= walkableHeight) ) {
#endif // MODIFY_VOXEL_FLAG
							// Mark direction as walkable.
							const int lidx = k - (int)nc.index;
							if (lidx < 0 || lidx > MAX_LAYERS)
							{
								tooHighNeighbour = rcMax(tooHighNeighbour, lidx);
								continue;
							}
							rcSetCon(s, dir, lidx);
							break;
#ifdef MODIFY_VOXEL_FLAG
							//}
#endif // MODIFY_VOXEL_FLAG
						}
					}
				}
			}
		}
	}
	
	if (tooHighNeighbour > MAX_LAYERS)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
				 tooHighNeighbour, MAX_LAYERS);
	}
		
	ctx->stopTimer(RC_TIMER_BUILD_COMPACTHEIGHTFIELD);
	
	return true;
}

/*
static int getHeightfieldMemoryUsage(const rcHeightfield& hf)
{
	int size = 0;
	size += sizeof(hf);
	size += hf.width * hf.height * sizeof(rcSpan*);
	
	rcSpanPool* pool = hf.pools;
	while (pool)
	{
		size += (sizeof(rcSpanPool) - sizeof(rcSpan)) + sizeof(rcSpan)*RC_SPANS_PER_POOL;
		pool = pool->next;
	}
	return size;
}

static int getCompactHeightFieldMemoryusage(const rcCompactHeightfield& chf)
{
	int size = 0;
	size += sizeof(rcCompactHeightfield);
	size += sizeof(rcCompactSpan) * chf.spanCount;
	size += sizeof(rcCompactCell) * chf.width * chf.height;
	return size;
}
*/

#ifdef MODIFY_VOXEL_FLAG
namespace
{
	void	rcMarkWalkableLowerFloorSpan( const int x, const int y, const int walkableClimb, rcHeightfield& solid )
	{
		const int w = solid.width;
		const int h = solid.height;

		for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
			if( (s->area & RC_UNDER_FLOOR_AREA) != RC_UNDER_FLOOR_AREA ) {
				continue;
			}
			for( int dir = 0; dir < 4; ++dir ) {
				const int dx = x + rcGetDirOffsetX(dir);
				const int dy = y + rcGetDirOffsetY(dir);
				if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
					continue;
				}
				for( const rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
					if( (ns->area & RC_UNDER_FLOOR_AREA) == RC_UNDER_FLOOR_AREA ) {
						continue;
					}
					if( !rcCanMovableArea( ns->area ) ) {
						continue;
					}
					const int gap = rcAbs( static_cast<int>(s->smax) - static_cast<int>(ns->smax) );
					if( gap <= walkableClimb ) {
						s->area &= ~RC_UNDER_FLOOR_AREA;
					}
				}
			}
		}
	}

	rcSpan* allocSpan( rcHeightfield& hf )
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

	void	freeSpan( rcHeightfield& hf, rcSpan* ptr )
	{
		if( ptr == NULL ) {
			return;
		}
		ptr->next = hf.freelist;
		hf.freelist = ptr;
	}

	void addSpan(rcHeightfield& hf, const int x, const int y,
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
			// similar area
			if( rcIsSimilarTypeArea(s->area, cur->area) ) {
				if( s->smax < cur->smin ) {
					break;
				}
				else if( cur->smax < s->smin ) {
					prev = cur;
					cur = cur->next;
				}
				else {
					s->smin = rcMin( s->smin, cur->smin );
					s->smax = rcMax( s->smax, cur->smax );
					if( rcAbs(static_cast<int>(s->smax) - static_cast<int>(cur->smax)) <= flagMergeThr ) {
						s->area = rcMax( s->area, cur->area );
					}
					else {
						s->area = rcMin( s->area, cur->area );
					}

					rcSpan* next = cur->next;
					freeSpan( hf, cur );
					if( prev ) {
						prev->next = next;
					}
					else {
						hf.spans[idx] = next;
					}
					cur = next;
				}
			}
			// different type
			else {
				if( rcIsTerrainArea( s->area ) ) {
					if( cur->smin == 0 && cur->smax == 0 ) {
						cur->smin = s->smin;
						cur->smax = s->smax;
						cur->area = s->area;
						freeSpan( hf, s );
						return;
					}
					break;
				}
				else {
					prev = cur;
					cur = cur->next;
				}
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
}

void	rcModifySpans( rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid )
{
	///*
	rcMarkWalkableLowHangingObstacles( ctx, walkableClimb, solid );
	rcFilterUnwalkableLowHeightSpans( ctx, walkableHeight, solid );

	//rcFilterUnwalkableLedgeSpans( ctx, walkableHeight, walkableClimb, solid );
	//rcFilterLedgeSpans( ctx, walkableHeight, walkableClimb, solid );

	rcFilterUnderFloorObjectSpans( ctx, solid );
	rcMarkTerrainWalkableUnderFloorSpans( ctx, walkableClimb, solid );
	//*/

	///*
	rcFilterSpans( ctx, walkableHeight, walkableClimb, solid );
	//*/

	/*
	rcFilterLowHangingWalkableObstacles( ctx, walkableClimb, solid );
	rcFilterLedgeSpans( ctx, walkableHeight, walkableClimb, solid );
	rcFilterWalkableLowHeightSpans( ctx, walkableHeight, solid );
	*/
}

/*
void	rcForm( rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
			}
		}
	}

	for( int dir = 0; dir < 4; ++dir ) {
		const int dx = x + rcGetDirOffsetX(dir);
		const int dy = y + rcGetDirOffsetY(dir);
		if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
			continue;
		}
		for( rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}
*/

//////////////////////////////////////////////////////////////////////////
void	rcFilterAlignmentSpans( rcContext* ctx, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			rcSpan* prev = NULL;
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL;  ) {
				prev = prev == NULL ? s : prev;
				rcSpan* next = s->next;
				if( rcIsTerrainArea(s->area) && s->smin == 0 && s->smax == 0 ) {
					freeSpan( solid, s );
					s = solid.spans[x + y*w] = next;
					prev = NULL;
					continue;
				}

				if( rcIsSimilarTypeArea( prev->area, s->area ) ) {
					if( s->smax < prev->smin ) {
						prev->area = rcMin( prev->area, s->area );
						freeSpan( solid, s );
						prev->next = next;
						s = next;
					}
					else if( prev->smax < s->smin ) {
						prev = s;
						s = next;
					}
					else {
						prev->smin = rcMin( prev->smin, s->smin );
						prev->smax = rcMax( prev->smax, s->smax );
						prev->area = rcMin( prev->area, s->area );
						freeSpan( solid, s );
						prev->next = next;
						s = next;
					}
				}
				// different
				else {
					if( prev->smax <= s->smax ) {
						s->smin = s->smin <= prev->smax ? prev->smax+1 : s->smin;
						s->smax = s->smax <= s->smin ? s->smin+1 : s->smax;
						prev = s;
						s = next;
					}
					else {
						freeSpan( solid, s );
						prev->next = next;
						s = next;
					}
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcFilterUnderFloorObjectSpans( rcContext* ctx, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL && s->next != NULL; s = s->next ) {
				if( rcCanMovableArea( s->area ) /*&& rcCanMovableArea( s->next->area )*/ ) {
					s->area |= RC_UNDER_FLOOR_AREA;
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcMarkTerrainWalkableUnderFloorSpans( rcContext* ctx, const int walkableClimb, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			rcMarkWalkableLowerFloorSpan( x, y, walkableClimb, solid );
		}
	}

	for( int x = 0; x < w; ++x ) {
		for( int y = 0; y < h; ++y ) {
			rcMarkWalkableLowerFloorSpan( x, y, walkableClimb, solid );
		}
	}

	for( int y = h-1; 0 <= y; --y ) {
		for( int x = w-1; 0 <= x; --x ) {
			rcMarkWalkableLowerFloorSpan( x, y, walkableClimb, solid );
		}
	}

	for( int x = w-1; 0 <= x; --x ) {
		for( int y = h-1; 0 <= y; --y ) {
			rcMarkWalkableLowerFloorSpan( x, y, walkableClimb, solid );
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcFilterUnwalkableUnderFloorObjectInsideSpans( rcContext* ctx, rcHeightfield& solid )
{
// 	rcAssert( ctx );
// 
// 	ctx->startTimer( RC_TIMER_TEMPORARY );
// 
// 	const int w = solid.width;
// 	const int h = solid.height;
// 
// 	for( int y = 0; y < h; ++y ) {
// 		for( int x = 0; x < w; ++x ) {
// 			rcSpan* s = solid.spans[x + y*w];
// 			if( s == NULL || s->area != (RC_TERRAIN_UNWALKABLE_AREA | RC_TERRAIN_CLIMBABLE_AREA) ) {
// 				continue;
// 			}
// 
// 			for( int dir = 0; dir < 4; ++dir ) {
// 				const int dx = x + rcGetDirOffsetX(dir);
// 				const int dy = y + rcGetDirOffsetY(dir);
// 				if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
// 					continue;
// 				}
// 				const rcSpan* ns = solid.spans[dx + dy*w];
// 				if( ns == NULL || ns->area != RC_TERRAIN_WALKABLE_AREA ) {
// 					continue;
// 				}
// 
// 				const int opposite_dir = rcGetOppositeSideDir( dir );
// 				const int opposite_x = x + rcGetDirOffsetX(opposite_dir);
// 				const int opposite_y = y + rcGetDirOffsetY(opposite_dir);
// 				if( opposite_x < 0 || opposite_y < 0 || opposite_x >= w || opposite_y >= h ) {
// 					continue;
// 				}
// 				rcSpan* opposite_ns = solid.spans[opposite_x + opposite_y*w];
// 				if( opposite_ns != NULL && rcIsTerrainArea(opposite_ns->area) && (opposite_ns->area & RC_TERRAIN_WALKABLE_AREA ) != RC_TERRAIN_WALKABLE_AREA ) {
// 					opposite_ns->area |= RC_TERRAIN_UNWALKABLE_AREA;
// 					//////////////////////////////////////////////////////////////////////////
// 					s->area |= RC_TERRAIN_WALKABLE_AREA;
// 				}
// 			}
// 		}
// 	}
// 
// 	for( int y = 0; y < h; ++y ) {
// 		for( int x = 0; x < w; ++x ) {
// 			rcSpan* s = solid.spans[x + y*w];
// 			if( s == NULL || s->area != RC_TERRAIN_CLIMBABLE_AREA ) {
// 				continue;
// 			}
// 
// 			for( int dir = 0; dir < 4; ++dir ) {
// 				const int dx = x + rcGetDirOffsetX(dir);
// 				const int dy = y + rcGetDirOffsetY(dir);
// 				if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
// 					continue;
// 				}
// 				rcSpan* ns = solid.spans[dx + dy*w];
// 				if( ns != NULL && ns->area == (RC_TERRAIN_UNWALKABLE_AREA | RC_TERRAIN_CLIMBABLE_AREA) ) {
// 					s->area = RC_TERRAIN_UNWALKABLE_AREA;
// 				}
// 			}
// 		}
// 	}
// 
// 	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcMarkTerrainUnderFloorObjectSpans( rcContext* ctx, const int walkableClimb, rcHeightfield& solid )
{
// 	rcFilterUnderFloorObjectSpans( ctx, walkableClimb, solid );
// 	rcMarkTerrainWalkableUnderFloorSpans( ctx, solid );
// 	rcFilterUnwalkableUnderFloorObjectInsideSpans( ctx, solid );
}

void	rcMarkObjectLedgeSpans( rcContext* ctx, const int walkableClimb, rcHeightfield& solid )
{
// 	rcAssert( ctx );
// 
// 	ctx->startTimer( RC_TIMER_TEMPORARY );
// 
// 	const int w = solid.width;
// 	const int h = solid.height;
// 
// 	for( int y = 0; y < h; ++y ) {
// 		for( int x = 0; x < w; ++x ) {
// 			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
// 				if( s->area != RC_OBJECT_UNWALKABLE_AREA ) {
// 					continue;
// 				}
// 				for( int dir = 0; dir < 4; ++dir ) {
// 					const int dx = x + rcGetDirOffsetX(dir);
// 					const int dy = y + rcGetDirOffsetY(dir);
// 					if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
// 						continue;
// 					}
// 					for( rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
// 						if( ns->area == RC_OBJECT_WALKABLE_AREA ) {
// 							if( rcAbs( static_cast<int>(s->smax) - static_cast<int>(ns->smax) ) < walkableClimb ) {
// 								s->area |= RC_OBJECT_WALKABLE_AREA;
// 							}
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// 
// 	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcMarkExpandObjectLedgeSpans( rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid )
{
// 	rcAssert( ctx );
// 
// 	ctx->startTimer( RC_TIMER_TEMPORARY );
// 
// 	const int w = solid.width;
// 	const int h = solid.height;
// 
// 	for( int y = 0; y < h; ++y ) {
// 		for( int x = 0; x < w; ++x ) {
// 			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
// 				if( !rcIsObjectArea(s->area) || s->area != (RC_OBJECT_UNWALKABLE_AREA | RC_OBJECT_WALKABLE_AREA) ) {
// 					continue;
// 				}
// 				for( int dir = 0; dir < 4; ++dir ) {
// 					const int dx = x + rcGetDirOffsetX(dir);
// 					const int dy = y + rcGetDirOffsetY(dir);
// 					if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
// 						continue;
// 					}
// 					bool connected = false;
// 					for( rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
// 						if( !rcIsObjectArea( ns->area ) ) {
// 							continue;
// 						}
// 						const int gap = rcAbs( static_cast<int>(s->smax) - static_cast<int>(ns->smax) );
// 						if( gap <= walkableClimb && (ns->area & RC_OBJECT_WALKABLE_AREA) == RC_OBJECT_WALKABLE_AREA ) {
// 							connected = true;
// 							break;
// 						}
// 					}
// 					if( !connected ) {
// 						addSpan( solid, dx, dy, s->smax-1, s->smax, RC_OBJECT_CLIMBABLE_AREA, walkableClimb );
// 						const int left = rcGetLeftSideDir( dir );
// 						const int right = rcGetRightSideDir( dir );
// 						for( int nth = 0; nth < 2; ++nth ) {
// 							int ddx = 0, ddy = 0;
// 							if( (nth&1) == 0 ) {
// 								ddx = dx + rcGetDirOffsetX(left);
// 								ddy = dy + rcGetDirOffsetY(left);
// 							}
// 							else {
// 								ddx = dx + rcGetDirOffsetX(right);
// 								ddy = dy + rcGetDirOffsetY(right);
// 							}
// 							if( 0 <= ddx && 0 <= ddy && ddx < w && ddy < h ) {
// 								addSpan( solid, ddx, ddy, s->smax-1, s->smax, RC_OBJECT_CLIMBABLE_AREA, walkableClimb );
// 							}
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
// 
// 	ctx->stopTimer( RC_TIMER_TEMPORARY );
}


void	rcFilterSpans( rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
				if( (s->area & RC_UNWALKABLE_AREA) == RC_UNWALKABLE_AREA ) {
					s->area = RC_NULL_AREA;
				}

				//////////////////////////////////////////////////////////////////////////
				// temporary
// 				if( (s->area & RC_UNDER_FLOOR_AREA) == RC_UNDER_FLOOR_AREA ) {
// 					s->area = RC_NULL_AREA;
// 				}
				//////////////////////////////////////////////////////////////////////////

				if( rcCanMovableArea( s->area ) ) {
					s->area |= RC_TERRAIN_AREA;
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcMarkSideLedgeSpans( rcContext* ctx, const int walkableClimb, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
				if( !rcIsObjectArea( s->area ) || !rcIsWalkableObjectArea( s->area ) ) {
					continue;
				}
				for( int dir = 0; dir < 4; ++dir ) {
					const int dx = x + rcGetDirOffsetX(dir);
					const int dy = y + rcGetDirOffsetY(dir);
					if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
						continue;
					}
					for( rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
						if( (ns->area & RC_UNWALKABLE_AREA) == RC_UNWALKABLE_AREA ) {
							const float gap = rcAbs( s->smax - ns->smax );
							if( gap <= walkableClimb ) {
								s->area |= RC_CLIMBABLE_AREA;
							}
						}
					}
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

//////////////////////////////////////////////////////////////////////////
void	rcMarkWalkableLowHangingObstacles( rcContext* ctx, const int walkableClimb, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			rcSpan* ps = 0;
			bool previousWalkable = false;

			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; ps = s, s = s->next ) {
				const bool walkable = rcIsWalkableArea( s->area );
				if( !walkable && previousWalkable ) {
					if( rcAbs((int)s->smax - (int)ps->smax) <= walkableClimb ) {
						if( rcIsSimilarTypeArea( s->area, ps->area ) ) {
							s->area = rcMax( s->area, ps->area );
						}
						else {
							s->area = RC_OBJECT_AREA | RC_WALKABLE_AREA;
						}
					}
				}
				previousWalkable = walkable;
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcFilterUnwalkableLowHeightSpans( rcContext* ctx, const int walkableHeight, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL && s->next != NULL; s = s->next ) {
				const int bot = static_cast<int>( s->smax );
				const int top = static_cast<int>( s->next->smin );
				if( ( top - bot ) <= walkableHeight ) {
					s->area = RC_NULL_AREA;
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcFilterUnwalkableLedgeSpans( rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
				if( !rcCanMovableArea( s->area ) || !rcIsObjectArea( s->area ) ) {
					continue;
				}

				const int smax = static_cast<int>( s->smax );
				int connectedEdgeCount = 0;

				for( int dir = 0; dir < 4; ++dir ) {
					int dx = x + rcGetDirOffsetX(dir);
					int dy = y + rcGetDirOffsetY(dir);
					if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
						++connectedEdgeCount;
						continue;
					}

					for( rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
						const int nsmax = static_cast<int>( ns->smax );
						if( rcAbs( smax - nsmax ) <= walkableClimb*0.25f ) {
							++connectedEdgeCount;
						}
					}
				}

				if( connectedEdgeCount < 2 ) {
					s->area = RC_NULL_AREA;
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}

void	rcFilterUnwalkableAreaSpans( rcContext* ctx, const unsigned char area, rcHeightfield& solid )
{
	rcAssert( ctx );

	ctx->startTimer( RC_TIMER_TEMPORARY );

	const int w = solid.width;
	const int h = solid.height;

	for( int y = 0; y < h; ++y ) {
		for( int x = 0; x < w; ++x ) {
			for( rcSpan* s = solid.spans[x + y*w]; s; s = s->next ) {
				if( s->area & area ) {
					s->area = RC_NULL_AREA;
				}
			}
		}
	}

	ctx->stopTimer( RC_TIMER_TEMPORARY );
}
//////////////////////////////////////////////////////////////////////////

void	rcTest( const int walkableHeight, const int walkableClimb, rcHeightfield& solid )
{
	const int w = solid.width;
	const int h = solid.height;

// 	for( int y = 0; y < h; ++y ) {
// 		for( int x = 0; x < w; ++x ) {
// 			for( rcSpan* s = solid.spans[x + y*w]; s != NULL; s = s->next ) {
// 				for( int dir = 0; dir < 4; ++dir ) {
// 					const int dx = x + rcGetDirOffsetX(dir);
// 					const int dy = y + rcGetDirOffsetY(dir);
// 					if( dx < 0 || dy < 0 || dx >= w || dy >= h ) {
// 						continue;
// 					}
// 					for( rcSpan* ns = solid.spans[dx + dy*w]; ns != NULL; ns = ns->next ) {
// 						if( ns->smin < s->smax && s->smax < ns->smax ) {
// 							_asm int 3;
// 						}
// 					}
// 				}
// 			}
// 		}
// 	}
}
#endif // MODIFY_VOXEL_FLAG

#ifdef MODIFY_OFF_MESH_CONNECTION
bool	rcIsOverlapBounds2D( const float* vertexPoint, const float* bmin, const float* bmax )
{
	const float extend[3] = {0.4f, 0.0f, 0.4f};
	float amin[3] = {0}, amax[3] = {0};
	rcVsub( amin, vertexPoint, extend );
	rcVadd( amax, vertexPoint, extend );

	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}
#endif // MODIFY_OFF_MESH_CONNECTION
