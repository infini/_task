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
#include <string.h>
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCoordinates.h"
#include <new>

/// @class dtQueryFilter
///
/// <b>The Default Implementation</b>
/// 
/// At construction: All area costs default to 1.0.  All flags are included
/// and none are excluded.
/// 
/// If a polygon has both an include and an exclude flag, it will be excluded.
/// 
/// The way filtering works, a navigation mesh polygon must have at least one flag 
/// set to ever be considered by a query. So a polygon with no flags will never
/// be considered.
///
/// Setting the include flags to 0 will result in all polygons being excluded.
///
/// <b>Custom Implementations</b>
/// 
/// DT_VIRTUAL_QUERYFILTER must be defined in order to extend this class.
/// 
/// Implement a custom query filter by overriding the virtual passFilter() 
/// and getCost() functions. If this is done, both functions should be as 
/// fast as possible. Use cached local copies of data rather than accessing 
/// your own objects where possible.
/// 
/// Custom implementations do not need to adhere to the flags or cost logic 
/// used by the default implementation.  
/// 
/// In order for A* searches to work properly, the cost should be proportional to
/// the travel distance. Implementing a cost modifier less than 1.0 is likely 
/// to lead to problems during pathfinding.
///
/// @see dtNavMeshQuery

dtQueryFilter::dtQueryFilter() :
	m_includeFlags(0xffff),
	m_excludeFlags(0)
{
	for (int i = 0; i < DT_MAX_AREAS; ++i)
		m_areaCost[i] = 1.0f;
}

#ifdef DT_VIRTUAL_QUERYFILTER
bool dtQueryFilter::passFilter(const dtPolyRef /*ref*/,
							   const dtMeshTile* /*tile*/,
							   const dtPoly* poly) const
{
	return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
}

float dtQueryFilter::getCost(const dtCoordinates& pa, const dtCoordinates& pb,
							 const dtPolyRef /*prevRef*/, const dtMeshTile* /*prevTile*/, const dtPoly* /*prevPoly*/,
							 const dtPolyRef /*curRef*/, const dtMeshTile* /*curTile*/, const dtPoly* curPoly,
							 const dtPolyRef /*nextRef*/, const dtMeshTile* /*nextTile*/, const dtPoly* /*nextPoly*/) const
{
	return dtVdist(pa, pb) * m_areaCost[curPoly->getArea()];
}
#else
inline bool dtQueryFilter::passFilter(const dtPolyRef /*ref*/,
									  const dtMeshTile* /*tile*/,
									  const dtPoly* poly) const
{
	return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
}

inline float dtQueryFilter::getCost(const dtCoordinates& pa, const dtCoordinates& pb,
									const dtPolyRef /*prevRef*/, const dtMeshTile* /*prevTile*/, const dtPoly* /*prevPoly*/,
									const dtPolyRef /*curRef*/, const dtMeshTile* /*curTile*/, const dtPoly* curPoly,
									const dtPolyRef /*nextRef*/, const dtMeshTile* /*nextTile*/, const dtPoly* /*nextPoly*/) const
{
	return dtVdist(pa, pb) * m_areaCost[curPoly->getArea()];
}
#endif	
	
static const float H_SCALE = 0.999f; // Search heuristic scale.


dtNavMeshQuery* dtAllocNavMeshQuery()
{
	void* mem = dtAlloc(sizeof(dtNavMeshQuery), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtNavMeshQuery;
}

void dtFreeNavMeshQuery(dtNavMeshQuery* navmesh)
{
	if (!navmesh) return;
	navmesh->~dtNavMeshQuery();
	dtFree(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////

/// @class dtNavMeshQuery
///
/// For methods that support undersized buffers, if the buffer is too small 
/// to hold the entire result set the return status of the method will include 
/// the #DT_BUFFER_TOO_SMALL flag.
///
/// Constant member functions can be used by multiple clients without side
/// effects. (E.g. No change to the closed list. No impact on an in-progress
/// sliced path query. Etc.)
/// 
/// Walls and portals: A @e wall is a polygon segment that is 
/// considered impassable. A @e portal is a passable segment between polygons.
/// A portal may be treated as a wall based on the dtQueryFilter used for a query.
///
/// @see dtNavMesh, dtQueryFilter, #dtAllocNavMeshQuery(), #dtAllocNavMeshQuery()

dtNavMeshQuery::dtNavMeshQuery() :
	m_nav(0),
	m_tinyNodePool(0),
	m_nodePool(0),
	m_openList(0)
{
	memset(&m_query, 0, sizeof(dtQueryData));
}

dtNavMeshQuery::~dtNavMeshQuery()
{
	if (m_tinyNodePool)
		m_tinyNodePool->~dtNodePool();
	if (m_nodePool)
		m_nodePool->~dtNodePool();
	if (m_openList)
		m_openList->~dtNodeQueue();
	dtFree(m_tinyNodePool);
	dtFree(m_nodePool);
	dtFree(m_openList);
}

/// @par 
///
/// Must be the first function called after construction, before other
/// functions are used.
///
/// This function can be used multiple times.
dtStatus dtNavMeshQuery::init(const dtNavMesh* nav, const int maxNodes)
{
	m_nav = nav;
	
	if (!m_nodePool || m_nodePool->getMaxNodes() < maxNodes)
	{
		if (m_nodePool)
		{
			m_nodePool->~dtNodePool();
			dtFree(m_nodePool);
			m_nodePool = 0;
		}
		m_nodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(maxNodes, dtNextPow2(maxNodes/4));
		if (!m_nodePool)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_nodePool->clear();
	}
	
	if (!m_tinyNodePool)
	{
		m_tinyNodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(64, 32);
		if (!m_tinyNodePool)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_tinyNodePool->clear();
	}
	
	// TODO: check the open list size too.
	if (!m_openList || m_openList->getCapacity() < maxNodes)
	{
		if (m_openList)
		{
			m_openList->~dtNodeQueue();
			dtFree(m_openList);
			m_openList = 0;
		}
		m_openList = new (dtAlloc(sizeof(dtNodeQueue), DT_ALLOC_PERM)) dtNodeQueue(maxNodes);
		if (!m_openList)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_openList->clear();
	}
	
	return DT_SUCCESS;
}

dtStatus dtNavMeshQuery::findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
										 dtPolyRef* randomRef, dtCoordinates& randomPt) const
{
	dtAssert(m_nav);
	
	// Randomly pick one tile. Assume that all tiles cover roughly the same area.
	const dtMeshTile* tile = 0;
	float tsum = 0.0f;
	for (int i = 0; i < m_nav->getMaxTiles(); i++)
	{
		const dtMeshTile* t = m_nav->getTile(i);
		if (!t || !t->header) continue;
		
		// Choose random tile using reservoi sampling.
		const float area = 1.0f; // Could be tile area too.
		tsum += area;
		const float u = frand();
		if (u*tsum <= area)
			tile = t;
	}
	if (!tile)
		return DT_FAILURE;

	// Randomly pick one polygon weighted by polygon area.
	const dtPoly* poly = 0;
	dtPolyRef polyRef = 0;
	const dtPolyRef base = m_nav->getPolyRefBase(tile);

	float areaSum = 0.0f;
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		// Do not return off-mesh connection polygons.
#ifdef MODIFY_OFF_MESH_CONNECTION
		if( !dtIsGroundTypeMesh( p->getType() ) ) {
#else // MODIFY_OFF_MESH_CONNECTION
		if (p->getType() != DT_POLYTYPE_GROUND) {
#endif // MODIFY_OFF_MESH_CONNECTION
			continue;
		}
		// Must pass filter
		const dtPolyRef ref = base | (dtPolyRef)i;
		if (!filter->passFilter(ref, tile, p))
			continue;

		// Calc area of the polygon.
		float polyArea = 0.0f;
		for (int j = 2; j < p->vertCount; ++j)
		{
			const dtCoordinates va( tile->verts[p->verts[0]] );
			const dtCoordinates vb( tile->verts[p->verts[j-1]] );
			const dtCoordinates vc( tile->verts[p->verts[j]] );
			polyArea += dtTriArea2D(va,vb,vc);
		}

		// Choose random polygon weighted by area, using reservoi sampling.
		areaSum += polyArea;
		const float u = frand();
		if (u*areaSum <= polyArea)
		{
			poly = p;
			polyRef = ref;
		}
	}
	
	if (!poly)
		return DT_FAILURE;

	// Randomly pick point on polygon.
	const dtCoordinates v( tile->verts[poly->verts[0]] );
	dtCoordinates verts[DT_VERTS_PER_POLYGON];
	float areas[DT_VERTS_PER_POLYGON];
	dtVcopy(verts[0],v);
	for (int j = 1; j < poly->vertCount; ++j)
	{
		const dtCoordinates v( tile->verts[poly->verts[j]] );
		dtVcopy(verts[j],v);
	}
	
	const float s = frand();
	const float t = frand();
	
	dtCoordinates pt;
	dtRandomPointInConvexPoly(verts, poly->vertCount, areas, s, t, pt);
	
	float h = 0.0f;
	dtStatus status = getPolyHeight(polyRef, pt, &h);
	if (dtStatusFailed(status))
		return status;
	pt.SetY( h );
	
	dtVcopy(randomPt, pt);
	*randomRef = polyRef;

	return DT_SUCCESS;
}

dtStatus dtNavMeshQuery::findRandomPointAroundCircle(dtPolyRef startRef, const dtCoordinates& centerPos, const float radius,
													 const dtQueryFilter* filter, float (*frand)(),
													 dtPolyRef* randomRef, dtCoordinates& randomPt) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	const dtMeshTile* startTile = 0;
	const dtPoly* startPoly = 0;
	m_nav->getTileAndPolyByRefUnsafe(startRef, &startTile, &startPoly);
	if (!filter->passFilter(startRef, startTile, startPoly))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtStatus status = DT_SUCCESS;
	
	const float radiusSqr = dtSqr(radius);
	float areaSum = 0.0f;

	const dtMeshTile* randomTile = 0;
	const dtPoly* randomPoly = 0;
	dtPolyRef randomPolyRef = 0;

	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Place random locations on on ground.
#ifdef MODIFY_OFF_MESH_CONNECTION
		if( dtIsGroundTypeMesh( bestPoly->getType() ) ) {
#else // MODIFY_OFF_MESH_CONNECTION
		if (bestPoly->getType() == DT_POLYTYPE_GROUND) {
#endif // MODIFY_OFF_MESH_CONNECTION
			// Calc area of the polygon.
			float polyArea = 0.0f;
			for (int j = 2; j < bestPoly->vertCount; ++j)
			{
				const dtCoordinates va( bestTile->verts[bestPoly->verts[0]] );
				const dtCoordinates vb( bestTile->verts[bestPoly->verts[j-1]] );
				const dtCoordinates vc( bestTile->verts[bestPoly->verts[j]] );
				polyArea += dtTriArea2D(va,vb,vc);
			}
			// Choose random polygon weighted by area, using reservoi sampling.
			areaSum += polyArea;
			const float u = frand();
			if (u*areaSum <= polyArea)
			{
				randomTile = bestTile;
				randomPoly = bestPoly;
				randomPolyRef = bestRef;
			}
		}
		
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			dtCoordinates va, vb;
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);
			
			const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	if (!randomPoly)
		return DT_FAILURE;
	
	// Randomly pick point on polygon.
	const dtCoordinates v( randomTile->verts[randomPoly->verts[0]] );
	dtCoordinates verts[DT_VERTS_PER_POLYGON];
	float areas[DT_VERTS_PER_POLYGON];
	dtVcopy(verts[0],v);
	for (int j = 1; j < randomPoly->vertCount; ++j)
	{
		const dtCoordinates v( randomTile->verts[randomPoly->verts[j]] );
		dtVcopy(verts[j],v);
	}
	
	const float s = frand();
	const float t = frand();
	
	dtCoordinates pt;
	dtRandomPointInConvexPoly(verts, randomPoly->vertCount, areas, s, t, pt);
	
	float h = 0.0f;
	dtStatus stat = getPolyHeight(randomPolyRef, pt, &h);
	if (dtStatusFailed(status))
		return stat;
	pt.SetY( h );
	
	dtVcopy(randomPt, pt);
	*randomRef = randomPolyRef;
	
	return DT_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////////////////////

/// @par
///
/// Uses the detail polygons to find the surface height. (Most accurate.)
///
/// @p pos does not have to be within the bounds of the polygon or navigation mesh.
///
/// See closestPointOnPolyBoundary() for a limited but faster option.
///
dtStatus dtNavMeshQuery::closestPointOnPoly(dtPolyRef ref, const dtCoordinates& pos, dtCoordinates& closest, bool* posOverPoly) const
{
	dtAssert(m_nav);
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!tile)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Off-mesh connections don't have detail polygons.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const dtCoordinates v0( tile->verts[poly->verts[0]] );
		const dtCoordinates v1( tile->verts[poly->verts[1]] );
		const float d0 = dtVdist(pos, v0);
		const float d1 = dtVdist(pos, v1);
		const float u = d0 / (d0+d1);
		dtVlerp(closest, v0, v1, u);
		if (posOverPoly)
			*posOverPoly = false;
		return DT_SUCCESS;
	}

	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];

	// Clamp point to be inside the polygon.
	dtCoordinates verts[DT_VERTS_PER_POLYGON];
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	const int nv = poly->vertCount;
	for (int i = 0; i < nv; ++i)
		dtVcopy(verts[i], tile->verts[poly->verts[i]]);
	
	dtVcopy(closest, pos);
	if (!dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget))
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		float dmin = FLT_MAX;
		int imin = -1;
		for (int i = 0; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const dtCoordinates va( verts[imin] );
		const dtCoordinates vb( verts[((imin+1)%nv)] );
		dtVlerp(closest, va, vb, edget[imin]);

		if (posOverPoly)
			*posOverPoly = false;
	}
	else
	{
		if (posOverPoly)
			*posOverPoly = true;
	}

	// Find height at the location.
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
		dtCoordinates v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = tile->verts[poly->verts[t[k]]];
			else
				v[k] = tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))];
		}
		float h;
		if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h))
		{
			closest.SetY( h );
			break;
		}
	}

	return DT_SUCCESS;
}

/// @par
///
/// Much faster than closestPointOnPoly().
///
/// If the provided position lies within the polygon's xz-bounds (above or below), 
/// then @p pos and @p closest will be equal.
///
/// The height of @p closest will be the polygon boundary.  The height detail is not used.
/// 
/// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
/// 
dtStatus dtNavMeshQuery::closestPointOnPolyBoundary(dtPolyRef ref, const dtCoordinates& pos, dtCoordinates& closest) const
{
	dtAssert(m_nav);
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Collect vertices.
	dtCoordinates verts[DT_VERTS_PER_POLYGON];
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int nv = 0;
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		dtVcopy(verts[nv], tile->verts[poly->verts[i]]);
		nv++;
	}		
	
	bool inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget);
	if (inside)
	{
		// Point is inside the polygon, return the point.
		dtVcopy(closest, pos);
	}
	else
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		float dmin = FLT_MAX;
		int imin = -1;
		for (int i = 0; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const dtCoordinates va( verts[imin] );
		const dtCoordinates vb( verts[((imin+1)%nv)] );
		dtVlerp(closest, va, vb, edget[imin]);
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Will return #DT_FAILURE if the provided position is outside the xz-bounds 
/// of the polygon.
/// 
dtStatus dtNavMeshQuery::getPolyHeight(dtPolyRef ref, const dtCoordinates& pos, float* height) const
{
	dtAssert(m_nav);

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const dtCoordinates v0( tile->verts[poly->verts[0]] );
		const dtCoordinates v1( tile->verts[poly->verts[1]] );
		const float d0 = dtVdist2D(pos, v0);
		const float d1 = dtVdist2D(pos, v1);
		const float u = d0 / (d0+d1);
		if (height)
			*height = v0.Y() + (v1.Y() - v0.Y()) * u;
		return DT_SUCCESS;
	}
	else
	{
		const unsigned int ip = (unsigned int)(poly - tile->polys);
		const dtPolyDetail* pd = &tile->detailMeshes[ip];
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
			dtCoordinates v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < poly->vertCount)
					v[k] = tile->verts[poly->verts[t[k]]];
				else
					v[k] = tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))];
			}
			float h;
			if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h))
			{
				if (height)
					*height = h;
				return DT_SUCCESS;
			}
		}
	}
	
	return DT_FAILURE | DT_INVALID_PARAM;
}

/// @par 
///
/// @note If the search box does not intersect any polygons the search will 
/// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check 
/// @p nearestRef before using @p nearestPt.
///
/// @warning This function is not suitable for large area searches.  If the search
/// extents overlaps more than 128 polygons it may return an invalid result.
///
dtStatus dtNavMeshQuery::findNearestPoly(const dtCoordinates& center, const dtCoordinates& extents,
										 const dtQueryFilter* filter,
										 dtPolyRef* nearestRef, dtCoordinates* nearestPt) const
{
	dtAssert(m_nav);

	*nearestRef = 0;
	
	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = 0;
	if (dtStatusFailed(queryPolygons(center, extents, filter, polys, &polyCount, 128)))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polys[i];
		dtCoordinates closestPtPoly;
		dtCoordinates diff;
		bool posOverPoly = false;
		float d = 0;
		closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff, center, closestPtPoly);
		if (posOverPoly)
		{
			const dtMeshTile* tile = 0;
			const dtPoly* poly = 0;
			m_nav->getTileAndPolyByRefUnsafe(polys[i], &tile, &poly);
			d = dtAbs(diff.Y()) - tile->header->walkableClimb;
			d = d > 0 ? d*d : 0;			
		}
		else
		{
			d = dtVlenSqr(diff);
		}
		
		if (d < nearestDistanceSqr)
		{
			if (nearestPt)
				dtVcopy(*nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	if (nearestRef)
		*nearestRef = nearest;
	
	return DT_SUCCESS;
}

int dtNavMeshQuery::queryPolygonsInTile(const dtMeshTile* tile, const dtCoordinates& qmin, const dtCoordinates& qmax,
										const dtQueryFilter* filter,
										dtPolyRef* polys, const int maxPolys) const
{
	dtAssert(m_nav);

	if (tile->bvTree)
	{
		const dtBVNode* node = &tile->bvTree[0];
		const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const dtCoordinates tbmin( tile->header->bmin );
		const dtCoordinates tbmax( tile->header->bmax );
		const float qfac = tile->header->bvQuantFactor;
		
		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// dtClamp query box to world box.
		float minx = dtClamp(qmin.X(), tbmin.X(), tbmax.X()) - tbmin.X();
		float miny = dtClamp(qmin.Y(), tbmin.Y(), tbmax.Y()) - tbmin.Y();
		float minz = dtClamp(qmin.Z(), tbmin.Z(), tbmax.Z()) - tbmin.Z();
		float maxx = dtClamp(qmax.X(), tbmin.X(), tbmax.X()) - tbmin.X();
		float maxy = dtClamp(qmax.Y(), tbmin.Y(), tbmax.Y()) - tbmin.Y();
		float maxz = dtClamp(qmax.Z(), tbmin.Z(), tbmax.Z()) - tbmin.Z();
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;
		
		// Traverse tree
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
		int n = 0;
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;
			
			if (isLeafNode && overlap)
			{
				dtPolyRef ref = base | (dtPolyRef)node->i;
				if (filter->passFilter(ref, tile, &tile->polys[node->i]))
				{
					if (n < maxPolys)
						polys[n++] = ref;
				}
			}
			
			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
		
		return n;
	}
	else
	{
		dtCoordinates bmin, bmax;
		int n = 0;
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const dtPoly* p = &tile->polys[i];
			// Do not return off-mesh connection polygons.
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			// Must pass filter
			const dtPolyRef ref = base | (dtPolyRef)i;
			if (!filter->passFilter(ref, tile, p))
				continue;
			// Calc polygon bounds.
			const dtCoordinates v( tile->verts[p->verts[0]] );
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				const dtCoordinates v( tile->verts[p->verts[j]] );
				dtVmin(bmin, v);
				dtVmax(bmax, v);
			}
			if (dtOverlapBounds(qmin,qmax, bmin,bmax))
			{
				if (n < maxPolys)
					polys[n++] = ref;
			}
		}
		return n;
	}
}

/// @par 
///
/// If no polygons are found, the function will return #DT_SUCCESS with a
/// @p polyCount of zero.
///
/// If @p polys is too small to hold the entire result set, then the array will 
/// be filled to capacity. The method of choosing which polygons from the 
/// full set are included in the partial result set is undefined.
///
dtStatus dtNavMeshQuery::queryPolygons(const dtCoordinates& center, const dtCoordinates& extents,
									   const dtQueryFilter* filter,
									   dtPolyRef* polys, int* polyCount, const int maxPolys) const
{
	dtAssert(m_nav);
	
	dtCoordinates bmin, bmax;
	dtVsub(bmin, center, extents);
	dtVadd(bmax, center, extents);
	
	// Find tiles the query touches.
	int minx, miny, maxx, maxy;
	m_nav->calcTileLoc(bmin, &minx, &miny);
	m_nav->calcTileLoc(bmax, &maxx, &maxy);

	static const int MAX_NEIS = 32;
	const dtMeshTile* neis[MAX_NEIS];
	
	int n = 0;
	for (int y = miny; y <= maxy; ++y)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const int nneis = m_nav->getTilesAt(x,y,neis,MAX_NEIS);
			for (int j = 0; j < nneis; ++j)
			{
				n += queryPolygonsInTile(neis[j], bmin, bmax, filter, polys+n, maxPolys-n);
				if (n >= maxPolys)
				{
					*polyCount = n;
					return DT_SUCCESS | DT_BUFFER_TOO_SMALL;
				}
			}
		}
	}
	*polyCount = n;
	
	return DT_SUCCESS;
}

/// @par
///
/// If the end polygon cannot be reached through the navigation graph,
/// the last polygon in the path will be the nearest the end polygon.
///
/// If the path array is to small to hold the full result, it will be filled as 
/// far as possible from the start polygon toward the end polygon.
///
/// The start and end positions are used to calculate traversal costs. 
/// (The y-values impact the result.)
///
dtStatus dtNavMeshQuery::findPath(dtPolyRef startRef, dtPolyRef endRef,
								  const dtCoordinates& startPos, const dtCoordinates& endPos,
								  const dtQueryFilter* filter,
								  dtPolyRef* path, int* pathCount, const int maxPath) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	*pathCount = 0;
	
	if (!startRef || !endRef)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	if (!maxPath)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	if (startRef == endRef)
	{
		path[0] = startRef;
		*pathCount = 1;
		return DT_SUCCESS;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, startPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;
	
	dtStatus status = DT_SUCCESS;
	
	while (!m_openList->empty())
	{
		// Remove node from open list and put it in closed list.
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Reached the goal, stop searching.
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}
		
		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;
			
			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);			
			
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			// If the node is visited the first time, calculate node position.
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
								neighbourRef, neighbourPoly, neighbourTile,
								neighbourNode->pos);
			}

			// Calculate cost and heuristic.
			float cost = 0;
			float heuristic = 0;
			
			// Special case for last node.
			if (neighbourRef == endRef)
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
													  parentRef, parentTile, parentPoly,
													  bestRef, bestTile, bestPoly,
													  neighbourRef, neighbourTile, neighbourPoly);
				const float endCost = filter->getCost(neighbourNode->pos, endPos,
													  bestRef, bestTile, bestPoly,
													  neighbourRef, neighbourTile, neighbourPoly,
													  0, 0, 0);
				
				cost = bestNode->cost + curCost + endCost;
				heuristic = 0;
			}
			else
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
													  parentRef, parentTile, parentPoly,
													  bestRef, bestTile, bestPoly,
													  neighbourRef, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
				heuristic = dtVdist(neighbourNode->pos, endPos)*H_SCALE;
			}

			const float total = cost + heuristic;
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			// The node is already visited and process, and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;
			
			// Add or update the node.
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->cost = cost;
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				m_openList->modify(neighbourNode);
			}
			else
			{
				// Put the node in open list.
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
			
			// Update nearest node to target so far.
			if (heuristic < lastBestNodeCost)
			{
				lastBestNodeCost = heuristic;
				lastBestNode = neighbourNode;
			}
		}
	}
	
	if (lastBestNode->id != endRef)
		status |= DT_PARTIAL_RESULT;
	
	// Reverse the path.
	dtNode* prev = 0;
	dtNode* node = lastBestNode;
	do
	{
		dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
		node->pidx = m_nodePool->getNodeIdx(prev);
		prev = node;
		node = next;
	}
	while (node);
	
	// Store path
	node = prev;
	int n = 0;
	do
	{
		path[n++] = node->id;
		if (n >= maxPath)
		{
			status |= DT_BUFFER_TOO_SMALL;
			break;
		}
		node = m_nodePool->getNodeAtIdx(node->pidx);
	}
	while (node);
	
	*pathCount = n;
	
	return status;
}

/// @par
///
/// @warning Calling any non-slice methods before calling finalizeSlicedFindPath() 
/// or finalizeSlicedFindPathPartial() may result in corrupted data!
///
/// The @p filter pointer is stored and used for the duration of the sliced
/// path query.
///
dtStatus dtNavMeshQuery::initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
											const dtCoordinates& startPos, const dtCoordinates& endPos,
											const dtQueryFilter* filter)
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Init path state.
	memset(&m_query, 0, sizeof(dtQueryData));
	m_query.status = DT_FAILURE;
	m_query.startRef = startRef;
	m_query.endRef = endRef;
	dtVcopy(m_query.startPos, startPos);
	dtVcopy(m_query.endPos, endPos);
	m_query.filter = filter;
	
	if (!startRef || !endRef)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	if (startRef == endRef)
	{
		m_query.status = DT_SUCCESS;
		return DT_SUCCESS;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, startPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	m_query.status = DT_IN_PROGRESS;
	m_query.lastBestNode = startNode;
	m_query.lastBestNodeCost = startNode->total;
	
	return m_query.status;
}
	
dtStatus dtNavMeshQuery::updateSlicedFindPath(const int maxIter, int* doneIters)
{
	if (!dtStatusInProgress(m_query.status))
		return m_query.status;

	// Make sure the request is still valid.
	if (!m_nav->isValidPolyRef(m_query.startRef) || !m_nav->isValidPolyRef(m_query.endRef))
	{
		m_query.status = DT_FAILURE;
		return DT_FAILURE;
	}
		
	int iter = 0;
	while (iter < maxIter && !m_openList->empty())
	{
		iter++;
		
		// Remove node from open list and put it in closed list.
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Reached the goal, stop searching.
		if (bestNode->id == m_query.endRef)
		{
			m_query.lastBestNode = bestNode;
			const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
			m_query.status = DT_SUCCESS | details;
			if (doneIters)
				*doneIters = iter;
			return m_query.status;
		}
		
		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(bestRef, &bestTile, &bestPoly)))
		{
			// The polygon has disappeared during the sliced query, fail.
			m_query.status = DT_FAILURE;
			if (doneIters)
				*doneIters = iter;
			return m_query.status;
		}
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
		{
			if (dtStatusFailed(m_nav->getTileAndPolyByRef(parentRef, &parentTile, &parentPoly)))
			{
				// The polygon has disappeared during the sliced query, fail.
				m_query.status = DT_FAILURE;
				if (doneIters)
					*doneIters = iter;
				return m_query.status;
			}
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;
			
			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);			
			
			if (!m_query.filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				m_query.status |= DT_OUT_OF_NODES;
				continue;
			}
			
			// If the node is visited the first time, calculate node position.
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
								neighbourRef, neighbourPoly, neighbourTile,
								neighbourNode->pos);
			}
			
			// Calculate cost and heuristic.
			float cost = 0;
			float heuristic = 0;
			
			// Special case for last node.
			if (neighbourRef == m_query.endRef)
			{
				// Cost
				const float curCost = m_query.filter->getCost(bestNode->pos, neighbourNode->pos,
															  parentRef, parentTile, parentPoly,
															  bestRef, bestTile, bestPoly,
															  neighbourRef, neighbourTile, neighbourPoly);
				const float endCost = m_query.filter->getCost(neighbourNode->pos, m_query.endPos,
															  bestRef, bestTile, bestPoly,
															  neighbourRef, neighbourTile, neighbourPoly,
															  0, 0, 0);
				
				cost = bestNode->cost + curCost + endCost;
				heuristic = 0;
			}
			else
			{
				// Cost
				const float curCost = m_query.filter->getCost(bestNode->pos, neighbourNode->pos,
															  parentRef, parentTile, parentPoly,
															  bestRef, bestTile, bestPoly,
															  neighbourRef, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
				heuristic = dtVdist(neighbourNode->pos, m_query.endPos)*H_SCALE;
			}
			
			const float total = cost + heuristic;
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			// The node is already visited and process, and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;
			
			// Add or update the node.
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->cost = cost;
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				m_openList->modify(neighbourNode);
			}
			else
			{
				// Put the node in open list.
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
			
			// Update nearest node to target so far.
			if (heuristic < m_query.lastBestNodeCost)
			{
				m_query.lastBestNodeCost = heuristic;
				m_query.lastBestNode = neighbourNode;
			}
		}
	}
	
	// Exhausted all nodes, but could not find path.
	if (m_openList->empty())
	{
		const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
		m_query.status = DT_SUCCESS | details;
	}

	if (doneIters)
		*doneIters = iter;

	return m_query.status;
}

dtStatus dtNavMeshQuery::finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath)
{
	*pathCount = 0;
	
	if (dtStatusFailed(m_query.status))
	{
		// Reset query.
		memset(&m_query, 0, sizeof(dtQueryData));
		return DT_FAILURE;
	}

	int n = 0;

	if (m_query.startRef == m_query.endRef)
	{
		// Special case: the search starts and ends at same poly.
		path[n++] = m_query.startRef;
	}
	else
	{
		// Reverse the path.
		dtAssert(m_query.lastBestNode);
		
		if (m_query.lastBestNode->id != m_query.endRef)
			m_query.status |= DT_PARTIAL_RESULT;
		
		dtNode* prev = 0;
		dtNode* node = m_query.lastBestNode;
		do
		{
			dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_nodePool->getNodeIdx(prev);
			prev = node;
			node = next;
		}
		while (node);
		
		// Store path
		node = prev;
		do
		{
			path[n++] = node->id;
			if (n >= maxPath)
			{
				m_query.status |= DT_BUFFER_TOO_SMALL;
				break;
			}
			node = m_nodePool->getNodeAtIdx(node->pidx);
		}
		while (node);
	}
	
	const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

	// Reset query.
	memset(&m_query, 0, sizeof(dtQueryData));
	
	*pathCount = n;
	
	return DT_SUCCESS | details;
}

dtStatus dtNavMeshQuery::finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
													   dtPolyRef* path, int* pathCount, const int maxPath)
{
	*pathCount = 0;
	
	if (existingSize == 0)
	{
		return DT_FAILURE;
	}
	
	if (dtStatusFailed(m_query.status))
	{
		// Reset query.
		memset(&m_query, 0, sizeof(dtQueryData));
		return DT_FAILURE;
	}
	
	int n = 0;
	
	if (m_query.startRef == m_query.endRef)
	{
		// Special case: the search starts and ends at same poly.
		path[n++] = m_query.startRef;
	}
	else
	{
		// Find furthest existing node that was visited.
		dtNode* prev = 0;
		dtNode* node = 0;
		for (int i = existingSize-1; i >= 0; --i)
		{
			node = m_nodePool->findNode(existing[i]);
			if (node)
				break;
		}
		
		if (!node)
		{
			m_query.status |= DT_PARTIAL_RESULT;
			dtAssert(m_query.lastBestNode);
			node = m_query.lastBestNode;
		}
		
		// Reverse the path.
		do
		{
			dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_nodePool->getNodeIdx(prev);
			prev = node;
			node = next;
		}
		while (node);
		
		// Store path
		node = prev;
		do
		{
			path[n++] = node->id;
			if (n >= maxPath)
			{
				m_query.status |= DT_BUFFER_TOO_SMALL;
				break;
			}
			node = m_nodePool->getNodeAtIdx(node->pidx);
		}
		while (node);
	}
	
	const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

	// Reset query.
	memset(&m_query, 0, sizeof(dtQueryData));
	
	*pathCount = n;
	
	return DT_SUCCESS | details;
}


dtStatus dtNavMeshQuery::appendVertex(const dtCoordinates& pos, const unsigned char flags, const dtPolyRef ref,
									  dtCoordinates* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
									  int* straightPathCount, const int maxStraightPath) const
{
	if ((*straightPathCount) > 0 && dtVequal(straightPath[((*straightPathCount)-1)], pos))
	{
		// The vertices are equal, update flags and poly.
		if (straightPathFlags)
			straightPathFlags[(*straightPathCount)-1] = flags;
		if (straightPathRefs)
			straightPathRefs[(*straightPathCount)-1] = ref;
	}
	else
	{
		// Append new vertex.
		dtVcopy(straightPath[(*straightPathCount)], pos);
		if (straightPathFlags)
			straightPathFlags[(*straightPathCount)] = flags;
		if (straightPathRefs)
			straightPathRefs[(*straightPathCount)] = ref;
		(*straightPathCount)++;
		// If reached end of path or there is no space to append more vertices, return.
		if (flags == DT_STRAIGHTPATH_END || (*straightPathCount) >= maxStraightPath)
		{
			return DT_SUCCESS | (((*straightPathCount) >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
		}
	}
	return DT_IN_PROGRESS;
}

dtStatus dtNavMeshQuery::appendPortals(const int startIdx, const int endIdx, const dtCoordinates& endPos, const dtPolyRef* path,
									  dtCoordinates* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
									  int* straightPathCount, const int maxStraightPath, const int options) const
{
	const dtCoordinates startPos( straightPath[(*straightPathCount-1)] );
	// Append or update last vertex
	dtStatus stat = 0;
	for (int i = startIdx; i < endIdx; i++)
	{
		// Calculate portal
		const dtPolyRef from = path[i];
		const dtMeshTile* fromTile = 0;
		const dtPoly* fromPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		const dtPolyRef to = path[i+1];
		const dtMeshTile* toTile = 0;
		const dtPoly* toPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		dtCoordinates left, right;
		if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
			break;
	
		if (options & DT_STRAIGHTPATH_AREA_CROSSINGS)
		{
			// Skip intersection if only area crossings are requested.
			if (fromPoly->getArea() == toPoly->getArea())
				continue;
		}
		
		// Append intersection
		float s,t;
		if (dtIntersectSegSeg2D(startPos, endPos, left, right, s, t))
		{
			dtCoordinates pt;
			dtVlerp(pt, left,right, t);

			stat = appendVertex(pt, 0, path[i+1],
								straightPath, straightPathFlags, straightPathRefs,
								straightPathCount, maxStraightPath);
			if (stat != DT_IN_PROGRESS)
				return stat;
		}
	}
	return DT_IN_PROGRESS;
}

/// @par
/// 
/// This method peforms what is often called 'string pulling'.
///
/// The start position is clamped to the first polygon in the path, and the 
/// end position is clamped to the last. So the start and end positions should 
/// normally be within or very near the first and last polygons respectively.
///
/// The returned polygon references represent the reference id of the polygon 
/// that is entered at the associated path position. The reference id associated 
/// with the end point will always be zero.  This allows, for example, matching 
/// off-mesh link points to their representative polygons.
///
/// If the provided result buffers are too small for the entire result set, 
/// they will be filled as far as possible from the start toward the end 
/// position.
///
dtStatus dtNavMeshQuery::findStraightPath(const dtCoordinates& startPos, const dtCoordinates& endPos,
										  const dtPolyRef* path, const int pathSize,
										  dtCoordinates* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
										  int* straightPathCount, const int maxStraightPath, const int options) const
{
	dtAssert(m_nav);
	
	*straightPathCount = 0;
	
	if (!maxStraightPath)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	if (!path[0])
		return DT_FAILURE | DT_INVALID_PARAM;
	
	dtStatus stat = 0;
	
	// TODO: Should this be callers responsibility?
	dtCoordinates closestStartPos;
	if (dtStatusFailed(closestPointOnPolyBoundary(path[0], startPos, closestStartPos)))
		return DT_FAILURE | DT_INVALID_PARAM;

	dtCoordinates closestEndPos;
	if (dtStatusFailed(closestPointOnPolyBoundary(path[pathSize-1], endPos, closestEndPos)))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Add start point.
	stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path[0],
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);
	if (stat != DT_IN_PROGRESS)
		return stat;
	
	if (pathSize > 1)
	{
		dtCoordinates portalApex, portalLeft, portalRight;
		dtVcopy(portalApex, closestStartPos);
		dtVcopy(portalLeft, portalApex);
		dtVcopy(portalRight, portalApex);
		int apexIndex = 0;
		int leftIndex = 0;
		int rightIndex = 0;
		
		unsigned char leftPolyType = 0;
		unsigned char rightPolyType = 0;
		
		dtPolyRef leftPolyRef = path[0];
		dtPolyRef rightPolyRef = path[0];
		
		for (int i = 0; i < pathSize; ++i)
		{
			dtCoordinates left, right;
			unsigned char fromType = 0, toType = 0;
			
			if (i+1 < pathSize)
			{
				// Next portal.
				if (dtStatusFailed(getPortalPoints(path[i], path[i+1], left, right, fromType, toType)))
				{
					// Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
					// Clamp the end point to path[i], and return the path so far.
					
					if (dtStatusFailed(closestPointOnPolyBoundary(path[i], endPos, closestEndPos)))
					{
						// This should only happen when the first polygon is invalid.
						return DT_FAILURE | DT_INVALID_PARAM;
					}

					// Apeend portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, i, closestEndPos, path,
											 straightPath, straightPathFlags, straightPathRefs,
											 straightPathCount, maxStraightPath, options);
					}

					stat = appendVertex(closestEndPos, 0, path[i],
										straightPath, straightPathFlags, straightPathRefs,
										straightPathCount, maxStraightPath);
					
					return DT_SUCCESS | DT_PARTIAL_RESULT | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
				}
				
				// If starting really close the portal, advance.
				if (i == 0)
				{
					float t;
					if (dtDistancePtSegSqr2D(portalApex, left, right, t) < dtSqr(0.001f))
						continue;
				}
			}
			else
			{
				// End of the path.
				dtVcopy(left, closestEndPos);
				dtVcopy(right, closestEndPos);

#ifdef MODIFY_OFF_MESH_CONNECTION
				fromType |= DT_POLYTYPE_GROUND;
				toType |= DT_POLYTYPE_GROUND;
#else // MODIFY_OFF_MESH_CONNECTION
				fromType = toType = DT_POLYTYPE_GROUND;
#endif // MODIFY_OFF_MESH_CONNECTION
			}
			
			// Right vertex.
			if (dtTriArea2D(portalApex, portalRight, right) <= 0.0f)
			{
				if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) > 0.0f)
				{
					dtVcopy(portalRight, right);
					rightPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
					rightPolyType = toType;
					rightIndex = i;
				}
				else
				{
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, leftIndex, portalLeft, path,
											 straightPath, straightPathFlags, straightPathRefs,
											 straightPathCount, maxStraightPath, options);
						if (stat != DT_IN_PROGRESS)
							return stat;					
					}
				
					dtVcopy(portalApex, portalLeft);
					apexIndex = leftIndex;
					
					unsigned char flags = 0;
					if (!leftPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = leftPolyRef;
					
					// Append or update vertex
					stat = appendVertex(portalApex, flags, ref,
										straightPath, straightPathFlags, straightPathRefs,
										straightPathCount, maxStraightPath);
					if (stat != DT_IN_PROGRESS)
						return stat;
					
					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;
					
					// Restart
					i = apexIndex;
					
					continue;
				}
			}
			
			// Left vertex.
			if (dtTriArea2D(portalApex, portalLeft, left) >= 0.0f)
			{
				if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) < 0.0f)
				{
					dtVcopy(portalLeft, left);
					leftPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
					leftPolyType = toType;
					leftIndex = i;
				}
				else
				{
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, rightIndex, portalRight, path,
											 straightPath, straightPathFlags, straightPathRefs,
											 straightPathCount, maxStraightPath, options);
						if (stat != DT_IN_PROGRESS)
							return stat;
					}

					dtVcopy(portalApex, portalRight);
					apexIndex = rightIndex;
					
					unsigned char flags = 0;
					if (!rightPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = rightPolyRef;

					// Append or update vertex
					stat = appendVertex(portalApex, flags, ref,
										straightPath, straightPathFlags, straightPathRefs,
										straightPathCount, maxStraightPath);
					if (stat != DT_IN_PROGRESS)
						return stat;
					
					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;
					
					// Restart
					i = apexIndex;
					
					continue;
				}
			}
		}

		// Append portals along the current straight path segment.
		if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			stat = appendPortals(apexIndex, pathSize-1, closestEndPos, path,
								 straightPath, straightPathFlags, straightPathRefs,
								 straightPathCount, maxStraightPath, options);
			if (stat != DT_IN_PROGRESS)
				return stat;
		}
	}

	stat = appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0,
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);
	
	return DT_SUCCESS | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
}

/// @par
///
/// This method is optimized for small delta movement and a small number of 
/// polygons. If used for too great a distance, the result set will form an 
/// incomplete path.
///
/// @p resultPos will equal the @p endPos if the end is reached. 
/// Otherwise the closest reachable position will be returned.
/// 
/// @p resultPos is not projected onto the surface of the navigation 
/// mesh. Use #getPolyHeight if this is needed.
///
/// This method treats the end position in the same manner as 
/// the #raycast method. (As a 2D point.) See that method's documentation 
/// for details.
/// 
/// If the @p visited array is too small to hold the entire result set, it will 
/// be filled as far as possible from the start position toward the end 
/// position.
///
dtStatus dtNavMeshQuery::moveAlongSurface(dtPolyRef startRef, const dtCoordinates& startPos, const dtCoordinates& endPos,
										  const dtQueryFilter* filter,
										  dtCoordinates& resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	*visitedCount = 0;
	
	// Validate input
	if (!startRef)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	dtStatus status = DT_SUCCESS;
	
	static const int MAX_STACK = 48;
	dtNode* stack[MAX_STACK];
	int nstack = 0;
	
	m_tinyNodePool->clear();
	
	dtNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;
	
	dtCoordinates bestPos;
	float bestDist = FLT_MAX;
	dtNode* bestNode = 0;
	dtVcopy(bestPos, startPos);
	
	// Search constraints
	dtCoordinates searchPos;
	float searchRadSqr;
	dtVlerp(searchPos, startPos, endPos, 0.5f);
	searchRadSqr = dtSqr(dtVdist(startPos, endPos)/2.0f + 0.001f);
	
	dtCoordinates verts[DT_VERTS_PER_POLYGON];
	
	while (nstack)
	{
		// Pop front.
		dtNode* curNode = stack[0];
		for (int i = 0; i < nstack-1; ++i)
			stack[i] = stack[i+1];
		nstack--;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const dtMeshTile* curTile = 0;
		const dtPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);			
		
		// Collect vertices.
		const int nverts = curPoly->vertCount;
		for (int i = 0; i < nverts; ++i)
			dtVcopy(verts[i], curTile->verts[curPoly->verts[i]]);
		
		// If target is inside the poly, stop search.
		if (dtPointInPolygon(endPos, verts, nverts))
		{
			bestNode = curNode;
			dtVcopy(bestPos, endPos);
			break;
		}
		
		// Find wall edges and find nearest point inside the walls.
		for (int i = 0, j = (int)curPoly->vertCount-1; i < (int)curPoly->vertCount; j = i++)
		{
			// Find links to neighbours.
			static const int MAX_NEIS = 8;
			int nneis = 0;
			dtPolyRef neis[MAX_NEIS];
			
			if (curPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					const dtLink* link = &curTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const dtMeshTile* neiTile = 0;
							const dtPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
							{
								if (nneis < MAX_NEIS)
									neis[nneis++] = link->ref;
							}
						}
					}
				}
			}
			else if (curPoly->neis[j])
			{
				const unsigned int idx = (unsigned int)(curPoly->neis[j]-1);
				const dtPolyRef ref = m_nav->getPolyRefBase(curTile) | idx;
				if (filter->passFilter(ref, curTile, &curTile->polys[idx]))
				{
					// Internal edge, encode id.
					neis[nneis++] = ref;
				}
			}
			
			if (!nneis)
			{
				// Wall edge, calc distance.
				const dtCoordinates vj( verts[j] );
				const dtCoordinates vi( verts[i] );
				float tseg;
				const float distSqr = dtDistancePtSegSqr2D(endPos, vj, vi, tseg);
				if (distSqr < bestDist)
				{
                    // Update nearest distance.
					dtVlerp(bestPos, vj,vi, tseg);
					bestDist = distSqr;
					bestNode = curNode;
				}
			}
			else
			{
				for (int k = 0; k < nneis; ++k)
				{
					// Skip if no node can be allocated.
					dtNode* neighbourNode = m_tinyNodePool->getNode(neis[k]);
					if (!neighbourNode)
						continue;
					// Skip if already visited.
					if (neighbourNode->flags & DT_NODE_CLOSED)
						continue;
					
					// Skip the link if it is too far from search constraint.
					// TODO: Maybe should use getPortalPoints(), but this one is way faster.
					const dtCoordinates vj( verts[j] );
					const dtCoordinates vi( verts[i] );
					float tseg;
					float distSqr = dtDistancePtSegSqr2D(searchPos, vj, vi, tseg);
					if (distSqr > searchRadSqr)
						continue;
					
					// Mark as the node as visited and push to queue.
					if (nstack < MAX_STACK)
					{
						neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
						neighbourNode->flags |= DT_NODE_CLOSED;
						stack[nstack++] = neighbourNode;
					}
				}
			}
		}
	}
	
	int n = 0;
	if (bestNode)
	{
		// Reverse the path.
		dtNode* prev = 0;
		dtNode* node = bestNode;
		do
		{
			dtNode* next = m_tinyNodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_tinyNodePool->getNodeIdx(prev);
			prev = node;
			node = next;
		}
		while (node);
		
		// Store result
		node = prev;
		do
		{
			visited[n++] = node->id;
			if (n >= maxVisitedSize)
			{
				status |= DT_BUFFER_TOO_SMALL;
				break;
			}
			node = m_tinyNodePool->getNodeAtIdx(node->pidx);
		}
		while (node);
	}
	
	dtVcopy(resultPos, bestPos);
	
	*visitedCount = n;
	
	return status;
}


dtStatus dtNavMeshQuery::getPortalPoints(dtPolyRef from, dtPolyRef to, dtCoordinates& left, dtCoordinates& right,
										 unsigned char& fromType, unsigned char& toType) const
{
	dtAssert(m_nav);
	
	const dtMeshTile* fromTile = 0;
	const dtPoly* fromPoly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	fromType = fromPoly->getType();

	const dtMeshTile* toTile = 0;
	const dtPoly* toPoly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	toType = toPoly->getType();
		
	return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right);
}

// Returns portal points between two polygons.
dtStatus dtNavMeshQuery::getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
										 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
										 dtCoordinates& left, dtCoordinates& right) const
{
	// Find the link that points to the 'to' polygon.
	const dtLink* link = 0;
	for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
	{
		if (fromTile->links[i].ref == to)
		{
			link = &fromTile->links[i];
			break;
		}
	}
	if (!link)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Handle off-mesh connections.
	if (fromPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		// Find link that points to first vertex.
		for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
		{
			if (fromTile->links[i].ref == to)
			{
				const int v = fromTile->links[i].edge;
				dtVcopy(left, fromTile->verts[fromPoly->verts[v]]);
				dtVcopy(right, fromTile->verts[fromPoly->verts[v]]);
				return DT_SUCCESS;
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	if (toPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		for (unsigned int i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next)
		{
			if (toTile->links[i].ref == from)
			{
				const int v = toTile->links[i].edge;
				dtVcopy(left, toTile->verts[toPoly->verts[v]]);
				dtVcopy(right, toTile->verts[toPoly->verts[v]]);
				return DT_SUCCESS;
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	// Find portal vertices.
	const int v0 = fromPoly->verts[link->edge];
	const int v1 = fromPoly->verts[(link->edge+1) % (int)fromPoly->vertCount];
	dtVcopy(left, fromTile->verts[v0]);
	dtVcopy(right, fromTile->verts[v1]);
	
	// If the link is at tile boundary, dtClamp the vertices to
	// the link width.
	if (link->side != 0xff)
	{
		// Unpack portal limits.
		if (link->bmin != 0 || link->bmax != 255)
		{
			const float s = 1.0f/255.0f;
			const float tmin = link->bmin*s;
			const float tmax = link->bmax*s;
			dtVlerp(left, fromTile->verts[v0], fromTile->verts[v1], tmin);
			dtVlerp(right, fromTile->verts[v0], fromTile->verts[v1], tmax);
		}
	}
	
	return DT_SUCCESS;
}

// Returns edge mid point between two polygons.
dtStatus dtNavMeshQuery::getEdgeMidPoint(dtPolyRef from, dtPolyRef to, dtCoordinates& mid) const
{
	dtCoordinates left, right;
	unsigned char fromType, toType;
	if (dtStatusFailed(getPortalPoints(from, to, left,right, fromType, toType)))
		return DT_FAILURE | DT_INVALID_PARAM;
	mid.SetX( (left.X()+right.X())*0.5f );
	mid.SetY( (left.Y()+right.Y())*0.5f );
	mid.SetZ( (left.Z()+right.Z())*0.5f );
	return DT_SUCCESS;
}

dtStatus dtNavMeshQuery::getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
										 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
										 dtCoordinates& mid) const
{
	dtCoordinates left, right;
	if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
		return DT_FAILURE | DT_INVALID_PARAM;
	mid.SetX( (left.X()+right.X())*0.5f );
	mid.SetY( (left.Y()+right.Y())*0.5f );
	mid.SetZ( (left.Z()+right.Z())*0.5f );
	return DT_SUCCESS;
}

/// @par
///
/// This method is meant to be used for quick, short distance checks.
///
/// If the path array is too small to hold the result, it will be filled as 
/// far as possible from the start postion toward the end position.
///
/// <b>Using the Hit Parameter (t)</b>
/// 
/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
/// the end position. In this case the path represents a valid corridor to the 
/// end position and the value of @p hitNormal is undefined.
///
/// If the hit parameter is zero, then the start position is on the wall that 
/// was hit and the value of @p hitNormal is undefined.
///
/// If 0 < t < 1.0 then the following applies:
///
/// @code
/// distanceToHitBorder = distanceToEndPosition * t
/// hitPoint = startPos + (endPos - startPos) * t
/// @endcode
///
/// <b>Use Case Restriction</b>
///
/// The raycast ignores the y-value of the end position. (2D check.) This 
/// places significant limits on how it can be used. For example:
///
/// Consider a scene where there is a main floor with a second floor balcony 
/// that hangs over the main floor. So the first floor mesh extends below the 
/// balcony mesh. The start position is somewhere on the first floor. The end 
/// position is on the balcony.
///
/// The raycast will search toward the end position along the first floor mesh. 
/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
/// (no wall hit), meaning it reached the end position. This is one example of why
/// this method is meant for short distance checks.
///
dtStatus dtNavMeshQuery::raycast(dtPolyRef startRef, const dtCoordinates& startPos, const dtCoordinates& endPos,
								 const dtQueryFilter* filter,
								 float* t, dtCoordinates& hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const
{
	dtAssert(m_nav);
	
	*t = 0;
	if (pathCount)
		*pathCount = 0;
	
	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	dtPolyRef curRef = startRef;
	dtCoordinates verts[DT_VERTS_PER_POLYGON];
	int n = 0;
	
	hitNormal = dtCoordinates();
	
	dtStatus status = DT_SUCCESS;
	
	while (curRef)
	{
		// Cast ray against current polygon.
		
		// The API input has been cheked already, skip checking internal data.
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &tile, &poly);
		
		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			dtVcopy(verts[nv], tile->verts[poly->verts[i]]);
			nv++;
		}
		
		float tmin, tmax;
		int segMin, segMax;
		if (!dtIntersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not hit the polygon, keep the old t and report hit.
			if (pathCount)
				*pathCount = n;
			return status;
		}
		// Keep track of furthest t so far.
		if (tmax > *t)
			*t = tmax;
		
		// Store visited polygons.
		if (n < maxPath)
			path[n++] = curRef;
		else
			status |= DT_BUFFER_TOO_SMALL;
		
		// Ray end is completely inside the polygon.
		if (segMax == -1)
		{
			*t = FLT_MAX;
			if (pathCount)
				*pathCount = n;
			return status;
		}
		
		// Follow neighbours.
		dtPolyRef nextRef = 0;
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
		{
			const dtLink* link = &tile->links[i];

			// Find link which contains this edge.
			if ((int)link->edge != segMax)
				continue;
			
			// Get pointer to the next polygon.
			const dtMeshTile* nextTile = 0;
			const dtPoly* nextPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(link->ref, &nextTile, &nextPoly);

			// Skip off-mesh connections.
			if (nextPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			// Skip links based on filter.
			if (!filter->passFilter(link->ref, nextTile, nextPoly))
				continue;
			
			// If the link is internal, just return the ref.
			if (link->side == 0xff)
			{
				nextRef = link->ref;
				break;
			}
			
			// If the link is at tile boundary,
			
			// Check if the link spans the whole edge, and accept.
			if (link->bmin == 0 && link->bmax == 255)
			{
				nextRef = link->ref;
				break;
			}
			
			// Check for partial edge links.
			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
			const dtCoordinates left( tile->verts[v0] );
			const dtCoordinates right( tile->verts[v1] );
			
			// Check that the intersection lies inside the link portal.
			if (link->side == 0 || link->side == 4)
			{
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left.Z() + (right.Z() - left.Z())*(link->bmin*s);
				float lmax = left.Z() + (right.Z() - left.Z())*(link->bmax*s);
				if (lmin > lmax) dtSwap(lmin, lmax);
				
				// Find Z intersection.
				float z = startPos.Z() + (endPos.Z()-startPos.Z())*tmax;
				if (z >= lmin && z <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
			else if (link->side == 2 || link->side == 6)
			{
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left.X() + (right.X() - left.X())*(link->bmin*s);
				float lmax = left.X() + (right.X() - left.X())*(link->bmax*s);
				if (lmin > lmax) dtSwap(lmin, lmax);
				
				// Find X intersection.
				float x = startPos.X() + (endPos.X()-startPos.X())*tmax;
				if (x >= lmin && x <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
		}
		
		if (!nextRef)
		{
			// No neighbour, we hit a wall.
			
			// Calculate hit normal.
			const int a = segMax;
			const int b = segMax+1 < nv ? segMax+1 : 0;
			const dtCoordinates va( verts[a] );
			const dtCoordinates vb( verts[b] );
			const float dx = vb.X() - va.X();
			const float dz = vb.Z() - va.Z();
			hitNormal.SetX( dz );
			hitNormal.SetY( 0 );
			hitNormal.SetZ( -dx );
			dtVnormalize(hitNormal);
			
			if (pathCount)
				*pathCount = n;
			return status;
		}
		
		// No hit, advance to neighbour polygon.
		curRef = nextRef;
	}
	
	if (pathCount)
		*pathCount = n;
	
	return status;
}

/// @par
///
/// At least one result array must be provided.
///
/// The order of the result set is from least to highest cost to reach the polygon.
///
/// A common use case for this method is to perform Dijkstra searches. 
/// Candidate polygons are found by searching the graph beginning at the start polygon.
///
/// If a polygon is not found via the graph search, even if it intersects the 
/// search circle, it will not be included in the result set. For example:
///
/// polyA is the start polygon.
/// polyB shares an edge with polyA. (Is adjacent.)
/// polyC shares an edge with polyB, but not with polyA
/// Even if the search circle overlaps polyC, it will not be included in the 
/// result set unless polyB is also in the set.
/// 
/// The value of the center point is used as the start position for cost 
/// calculations. It is not projected onto the surface of the mesh, so its 
/// y-value will effect the costs.
///
/// Intersection tests occur in 2D. All polygons and the search circle are 
/// projected onto the xz-plane. So the y-value of the center point does not 
/// effect intersection tests.
///
/// If the result arrays are to small to hold the entire result set, they will be 
/// filled to capacity.
/// 
dtStatus dtNavMeshQuery::findPolysAroundCircle(dtPolyRef startRef, const dtCoordinates& centerPos, const float radius,
											   const dtQueryFilter* filter,
											   dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
											   int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	*resultCount = 0;
	
	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtStatus status = DT_SUCCESS;
	
	int n = 0;
	if (n < maxResult)
	{
		if (resultRef)
			resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		if (resultCost)
			resultCost[n] = 0;
		++n;
	}
	else
	{
		status |= DT_BUFFER_TOO_SMALL;
	}
	
	const float radiusSqr = dtSqr(radius);
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
		
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			dtCoordinates va, vb;
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
				
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);
			
			const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				if (n < maxResult)
				{
					if (resultRef)
						resultRef[n] = neighbourNode->id;
					if (resultParent)
						resultParent[n] = m_nodePool->getNodeAtIdx(neighbourNode->pidx)->id;
					if (resultCost)
						resultCost[n] = neighbourNode->total;
					++n;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	*resultCount = n;
	
	return status;
}

/// @par
///
/// The order of the result set is from least to highest cost.
/// 
/// At least one result array must be provided.
///
/// A common use case for this method is to perform Dijkstra searches. 
/// Candidate polygons are found by searching the graph beginning at the start 
/// polygon.
/// 
/// The same intersection test restrictions that apply to findPolysAroundCircle()
/// method apply to this method.
/// 
/// The 3D centroid of the search polygon is used as the start position for cost 
/// calculations.
/// 
/// Intersection tests occur in 2D. All polygons are projected onto the 
/// xz-plane. So the y-values of the vertices do not effect intersection tests.
/// 
/// If the result arrays are is too small to hold the entire result set, they will 
/// be filled to capacity.
///
dtStatus dtNavMeshQuery::findPolysAroundShape(dtPolyRef startRef, const dtCoordinates* verts, const int nverts,
											  const dtQueryFilter* filter,
											  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
											  int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	*resultCount = 0;
	
	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtCoordinates centerPos;
	for (int i = 0; i < nverts; ++i)
		dtVadd(centerPos,centerPos,verts[i]);
	dtVscale(centerPos,centerPos,1.0f/nverts);

	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	dtStatus status = DT_SUCCESS;

	int n = 0;
	if (n < maxResult)
	{
		if (resultRef)
			resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		if (resultCost)
			resultCost[n] = 0;
		++n;
	}
	else
	{
		status |= DT_BUFFER_TOO_SMALL;
	}
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			dtCoordinates va, vb;
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the poly is not touching the edge to the next polygon, skip the connection it.
			float tmin, tmax;
			int segMin, segMax;
			if (!dtIntersectSegmentPoly2D(va, vb, verts, nverts, tmin, tmax, segMin, segMax))
				continue;
			if (tmin > 1.0f || tmax < 0.0f)
				continue;
			
			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);
			
			const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				if (n < maxResult)
				{
					if (resultRef)
						resultRef[n] = neighbourNode->id;
					if (resultParent)
						resultParent[n] = m_nodePool->getNodeAtIdx(neighbourNode->pidx)->id;
					if (resultCost)
						resultCost[n] = neighbourNode->total;
					++n;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	*resultCount = n;
	
	return status;
}

/// @par
///
/// This method is optimized for a small search radius and small number of result 
/// polygons.
///
/// Candidate polygons are found by searching the navigation graph beginning at 
/// the start polygon.
///
/// The same intersection test restrictions that apply to the findPolysAroundCircle 
/// mehtod applies to this method.
///
/// The value of the center point is used as the start point for cost calculations. 
/// It is not projected onto the surface of the mesh, so its y-value will effect 
/// the costs.
/// 
/// Intersection tests occur in 2D. All polygons and the search circle are 
/// projected onto the xz-plane. So the y-value of the center point does not 
/// effect intersection tests.
/// 
/// If the result arrays are is too small to hold the entire result set, they will 
/// be filled to capacity.
/// 
dtStatus dtNavMeshQuery::findLocalNeighbourhood(dtPolyRef startRef, const dtCoordinates& centerPos, const float radius,
												const dtQueryFilter* filter,
												dtPolyRef* resultRef, dtPolyRef* resultParent,
												int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);
	
	*resultCount = 0;

	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	static const int MAX_STACK = 48;
	dtNode* stack[MAX_STACK];
	int nstack = 0;
	
	m_tinyNodePool->clear();
	
	dtNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;
	
	const float radiusSqr = dtSqr(radius);
	
	dtCoordinates pa[DT_VERTS_PER_POLYGON];
	dtCoordinates pb[DT_VERTS_PER_POLYGON];
	
	dtStatus status = DT_SUCCESS;
	
	int n = 0;
	if (n < maxResult)
	{
		resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		++n;
	}
	else
	{
		status |= DT_BUFFER_TOO_SMALL;
	}
	
	while (nstack)
	{
		// Pop front.
		dtNode* curNode = stack[0];
		for (int i = 0; i < nstack-1; ++i)
			stack[i] = stack[i+1];
		nstack--;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const dtMeshTile* curTile = 0;
		const dtPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);
		
		for (unsigned int i = curPoly->firstLink; i != DT_NULL_LINK; i = curTile->links[i].next)
		{
			const dtLink* link = &curTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours.
			if (!neighbourRef)
				continue;
			
			// Skip if cannot alloca more nodes.
			dtNode* neighbourNode = m_tinyNodePool->getNode(neighbourRef);
			if (!neighbourNode)
				continue;
			// Skip visited.
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Skip off-mesh connections.
			if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			dtCoordinates va, vb;
			if (!getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			// Mark node visited, this is done before the overlap test so that
			// we will not visit the poly again if the test fails.
			neighbourNode->flags |= DT_NODE_CLOSED;
			neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
			
			// Check that the polygon does not collide with existing polygons.
			
			// Collect vertices of the neighbour poly.
			const int npa = neighbourPoly->vertCount;
			for (int k = 0; k < npa; ++k)
				dtVcopy(pa[k], neighbourTile->verts[neighbourPoly->verts[k]]);
			
			bool overlap = false;
			for (int j = 0; j < n; ++j)
			{
				dtPolyRef pastRef = resultRef[j];
				
				// Connected polys do not overlap.
				bool connected = false;
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					if (curTile->links[k].ref == pastRef)
					{
						connected = true;
						break;
					}
				}
				if (connected)
					continue;
				
				// Potentially overlapping.
				const dtMeshTile* pastTile = 0;
				const dtPoly* pastPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(pastRef, &pastTile, &pastPoly);
				
				// Get vertices and test overlap
				const int npb = pastPoly->vertCount;
				for (int k = 0; k < npb; ++k)
					dtVcopy(pb[k], pastTile->verts[pastPoly->verts[k]]);
				
				if (dtOverlapPolyPoly2D(pa,npa, pb,npb))
				{
					overlap = true;
					break;
				}
			}
			if (overlap)
				continue;
			
			// This poly is fine, store and advance to the poly.
			if (n < maxResult)
			{
				resultRef[n] = neighbourRef;
				if (resultParent)
					resultParent[n] = curRef;
				++n;
			}
			else
			{
				status |= DT_BUFFER_TOO_SMALL;
			}
			
			if (nstack < MAX_STACK)
			{
				stack[nstack++] = neighbourNode;
			}
		}
	}
	
	*resultCount = n;
	
	return status;
}


struct dtSegInterval
{
	dtPolyRef ref;
	short tmin, tmax;
};

static void insertInterval(dtSegInterval* ints, int& nints, const int maxInts,
						   const short tmin, const short tmax, const dtPolyRef ref)
{
	if (nints+1 > maxInts) return;
	// Find insertion point.
	int idx = 0;
	while (idx < nints)
	{
		if (tmax <= ints[idx].tmin)
			break;
		idx++;
	}
	// Move current results.
	if (nints-idx)
		memmove(ints+idx+1, ints+idx, sizeof(dtSegInterval)*(nints-idx));
	// Store
	ints[idx].ref = ref;
	ints[idx].tmin = tmin;
	ints[idx].tmax = tmax;
	nints++;
}

/// @par
///
/// If the @p segmentRefs parameter is provided, then all polygon segments will be returned. 
/// Otherwise only the wall segments are returned.
/// 
/// A segment that is normally a portal will be included in the result set as a 
/// wall if the @p filter results in the neighbor polygon becoomming impassable.
/// 
/// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the 
/// maximum segments per polygon of the source navigation mesh.
/// 
dtStatus dtNavMeshQuery::getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
											 dtCoordinates* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
											 const int maxSegments) const
{
	dtAssert(m_nav);
	
	*segmentCount = 0;
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	int n = 0;
	static const int MAX_INTERVAL = 16;
	dtSegInterval ints[MAX_INTERVAL];
	int nints;
	
	const bool storePortals = segmentRefs != 0;
	
	dtStatus status = DT_SUCCESS;
	
	for (int i = 0, j = (int)poly->vertCount-1; i < (int)poly->vertCount; j = i++)
	{
		// Skip non-solid edges.
		nints = 0;
		if (poly->neis[j] & DT_EXT_LINK)
		{
			// Tile border.
			for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				const dtLink* link = &tile->links[k];
				if (link->edge == j)
				{
					if (link->ref != 0)
					{
						const dtMeshTile* neiTile = 0;
						const dtPoly* neiPoly = 0;
						m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
						if (filter->passFilter(link->ref, neiTile, neiPoly))
						{
							insertInterval(ints, nints, MAX_INTERVAL, link->bmin, link->bmax, link->ref);
						}
					}
				}
			}
		}
		else
		{
			// Internal edge
			dtPolyRef neiRef = 0;
			if (poly->neis[j])
			{
				const unsigned int idx = (unsigned int)(poly->neis[j]-1);
				neiRef = m_nav->getPolyRefBase(tile) | idx;
				if (!filter->passFilter(neiRef, tile, &tile->polys[idx]))
					neiRef = 0;
			}

			// If the edge leads to another polygon and portals are not stored, skip.
			if (neiRef != 0 && !storePortals)
				continue;
			
			if (n < maxSegments)
			{
				const dtCoordinates vj( tile->verts[poly->verts[j]] );
				const dtCoordinates vi( tile->verts[poly->verts[i]] );
				dtVcopy(segmentVerts[n*2+0], vj);
				dtVcopy(segmentVerts[n*2+1], vi);
				if (segmentRefs)
					segmentRefs[n] = neiRef;
				n++;
			}
			else
			{
				status |= DT_BUFFER_TOO_SMALL;
			}
			
			continue;
		}
		
		// Add sentinels
		insertInterval(ints, nints, MAX_INTERVAL, -1, 0, 0);
		insertInterval(ints, nints, MAX_INTERVAL, 255, 256, 0);
		
		// Store segments.
		const dtCoordinates vj( tile->verts[poly->verts[j]] );
		const dtCoordinates vi( tile->verts[poly->verts[i]] );
		for (int k = 1; k < nints; ++k)
		{
			// Portal segment.
			if (storePortals && ints[k].ref)
			{
				const float tmin = ints[k].tmin/255.0f; 
				const float tmax = ints[k].tmax/255.0f; 
				if (n < maxSegments)
				{
					dtVlerp(segmentVerts[n*2+0], vj,vi, tmin);
					dtVlerp(segmentVerts[n*2+1], vj,vi, tmax);
					if (segmentRefs)
						segmentRefs[n] = ints[k].ref;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
			}

			// Wall segment.
			const int imin = ints[k-1].tmax;
			const int imax = ints[k].tmin;
			if (imin != imax)
			{
				const float tmin = imin/255.0f; 
				const float tmax = imax/255.0f; 
				if (n < maxSegments)
				{
					dtVlerp(segmentVerts[n*2+0], vj,vi, tmin);
					dtVlerp(segmentVerts[n*2+1], vj,vi, tmax);
					if (segmentRefs)
						segmentRefs[n] = 0;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
			}
		}
	}
	
	*segmentCount = n;
	
	return status;
}

/// @par
///
/// @p hitPos is not adjusted using the height detail data.
///
/// @p hitDist will equal the search radius if there is no wall within the 
/// radius. In this case the values of @p hitPos and @p hitNormal are
/// undefined.
///
/// The normal will become unpredicable if @p hitDist is a very small number.
///
dtStatus dtNavMeshQuery::findDistanceToWall(dtPolyRef startRef, const dtCoordinates& centerPos, const float maxRadius,
											const dtQueryFilter* filter,
											float* hitDist, dtCoordinates& hitPos, dtCoordinates& hitNormal) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_nodePool->clear();
	m_openList->clear();
	
	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	float radiusSqr = dtSqr(maxRadius);
	
	dtStatus status = DT_SUCCESS;
	
	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		// Hit test walls.
		for (int i = 0, j = (int)bestPoly->vertCount-1; i < (int)bestPoly->vertCount; j = i++)
		{
			// Skip non-solid edges.
			if (bestPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				bool solid = true;
				for (unsigned int k = bestPoly->firstLink; k != DT_NULL_LINK; k = bestTile->links[k].next)
				{
					const dtLink* link = &bestTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const dtMeshTile* neiTile = 0;
							const dtPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
								solid = false;
						}
						break;
					}
				}
				if (!solid) continue;
			}
			else if (bestPoly->neis[j])
			{
				// Internal edge
				const unsigned int idx = (unsigned int)(bestPoly->neis[j]-1);
				const dtPolyRef ref = m_nav->getPolyRefBase(bestTile) | idx;
				if (filter->passFilter(ref, bestTile, &bestTile->polys[idx]))
					continue;
			}
			
			// Calc distance to the edge.
			const dtCoordinates vj( bestTile->verts[bestPoly->verts[j]] );
			const dtCoordinates vi( bestTile->verts[bestPoly->verts[i]] );
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, vj, vi, tseg);
			
			// Edge is too far, skip.
			if (distSqr > radiusSqr)
				continue;
			
			// Hit wall, update radius.
			radiusSqr = distSqr;
			// Calculate hit pos.
			hitPos.SetX( vj.X() + (vi.X() - vj.X())*tseg );
			hitPos.SetY( vj.Y() + (vi.Y() - vj.Y())*tseg );
			hitPos.SetZ( vj.Z() + (vi.Z() - vj.Z())*tseg );
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Skip off-mesh connections.
			if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Calc distance to the edge.
			const dtCoordinates va( bestTile->verts[bestPoly->verts[link->edge]] );
			const dtCoordinates vb( bestTile->verts[bestPoly->verts[(link->edge+1) % bestPoly->vertCount]] );
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			
			// If the circle is not touching the next polygon, skip it.
			if (distSqr > radiusSqr)
				continue;
			
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
								neighbourRef, neighbourPoly, neighbourTile, neighbourNode->pos);
			}
			
			const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
				
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	// Calc hit normal.
	dtVsub(hitNormal, centerPos, hitPos);
	dtVnormalize(hitNormal);
	
	*hitDist = dtMathSqrtf(radiusSqr);
	
	return status;
}

bool dtNavMeshQuery::isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const
{
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	dtStatus status = m_nav->getTileAndPolyByRef(ref, &tile, &poly);
	// If cannot get polygon, assume it does not exists and boundary is invalid.
	if (dtStatusFailed(status))
		return false;
	// If cannot pass filter, assume flags has changed and boundary is invalid.
	if (!filter->passFilter(ref, tile, poly))
		return false;
	return true;
}

/// @par
///
/// The closed list is the list of polygons that were fully evaluated during 
/// the last navigation graph search. (A* or Dijkstra)
/// 
bool dtNavMeshQuery::isInClosedList(dtPolyRef ref) const
{
	if (!m_nodePool) return false;
	const dtNode* node = m_nodePool->findNode(ref);
	return node && node->flags & DT_NODE_CLOSED;
}

//////////////////////////////////////////////////////////////////////////
// MIRCHANG
dtStatus	dtNavMeshQuery::findCorrectPoly( const dtCoordinates& position, dtPolyRef& polyID ) const
{
	dtAssert( m_nav );

	polyID = 0;

	if( dtStatusFailed( queryCorrectPolygons( position, polyID ) ) )
		return DT_FAILURE | DT_INVALID_PARAM;
	
	return polyID != 0 ? DT_SUCCESS : DT_FAILURE;
}

dtStatus	dtNavMeshQuery::queryCorrectPolygons( const dtCoordinates& position, dtPolyRef& polyID ) const
{
	dtAssert( m_nav );

	polyID = 0;

	const dtCoordinates extents( 0.5f, 0.f, 0.5f );
	dtCoordinates bmin, bmax;
	dtVsub(bmin, position, extents);
	dtVadd(bmax, position, extents);
	int minx, miny, maxx, maxy;
	m_nav->calcTileLoc(bmin, &minx, &miny);
	m_nav->calcTileLoc(bmax, &maxx, &maxy);

// 	int x, y;
// 	m_nav->calcTileLoc(position, &x, &y);

	for( int y = miny; y <= maxy; ++y )
	{
		for( int x = minx; x <= maxx; ++x )
		{
	const dtMeshTile* tile = m_nav->getTilesAt( x, y );
	if( tile ) {
		polyID = queryCorrectPolygonsInTile( tile, position );
				if( polyID != 0 ) {
					return DT_SUCCESS;
				}
			}
	}
	}
	return DT_FAILURE;
}

dtPolyRef	dtNavMeshQuery::queryCorrectPolygonsInTile( const dtMeshTile* tile, const dtCoordinates& position ) const
{
	dtAssert( m_nav );
	dtAssert( tile->bvTree );

	const dtBVNode* node = &tile->bvTree[0];
	const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
	const dtCoordinates tbmin( tile->header->bmin );
	const dtCoordinates tbmax( tile->header->bmax );
	const float qfac = tile->header->bvQuantFactor;

	// Calculate quantized box
	unsigned short bmin[3], bmax[3];
	// dtClamp query box to world box.
	float minx = dtClamp(position.X(), tbmin.X(), tbmax.X()) - tbmin.X();
	float miny = dtClamp(position.Y() - tile->header->walkableClimb, tbmin.Y(), tbmax.Y()) - tbmin.Y();
	float minz = dtClamp(position.Z(), tbmin.Z(), tbmax.Z()) - tbmin.Z();
	float maxx = dtClamp(position.X(), tbmin.X(), tbmax.X()) - tbmin.X();
	float maxy = dtClamp(position.Y() + tile->header->walkableClimb, tbmin.Y(), tbmax.Y()) - tbmin.Y();
	float maxz = dtClamp(position.Z(), tbmin.Z(), tbmax.Z()) - tbmin.Z();
	// Quantize
	bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
	bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
	bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
	bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
	bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
	bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;

	// Traverse tree
	const dtPolyRef base( m_nav->getPolyRefBase(tile) );
	while( node < end ) {
		const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;

		if( isLeafNode && overlap ) {
			const dtPolyRef pid = base | (dtPolyRef)node->i;
			const dtMeshTile* _tile	= 0;
			const dtPoly* _poly		= 0;
			m_nav->getTileAndPolyByRefUnsafe( pid, &_tile, &_poly );
			dtCoordinates verts[DT_VERTS_PER_POLYGON];
			const int vertCount = static_cast<int>( _poly->vertCount );
			for( int nth = 0; nth < vertCount; ++nth ) {
				dtVcopy( verts[nth], _tile->verts[_poly->verts[nth]] );
			}
			if( dtPointInPolygon( position, verts, vertCount ) ) {
				return pid;
			}
		}

		if( overlap || isLeafNode ) {
			node++;
		}
		else {
			const int escapeIndex = -node->i;
			node += escapeIndex;
		}
	}

	return 0;
}

dtStatus	dtNavMeshQuery::findCorrectPoly2D( const dtCoordinates& position, dtPolyRef& polyID, const bool ground ) const
{
	dtAssert( m_nav );

	polyID = 0;

	if( dtStatusFailed( queryCorrectPolygons2D( position, polyID, ground ) ) )
		return DT_FAILURE | DT_INVALID_PARAM;

	return polyID != 0 ? DT_SUCCESS : DT_FAILURE;
}

dtStatus	dtNavMeshQuery::queryCorrectPolygons2D( const dtCoordinates& position, dtPolyRef& polyID, const bool ground ) const
{
	dtAssert( m_nav );

	polyID = 0;

	const dtCoordinates extents( 0.5f, 0.f, 0.5f );
	dtCoordinates bmin, bmax;
	dtVsub(bmin, position, extents);
	dtVadd(bmax, position, extents);
	int minx, miny, maxx, maxy;
	m_nav->calcTileLoc(bmin, &minx, &miny);
	m_nav->calcTileLoc(bmax, &maxx, &maxy);

// 	int x, y;
// 	m_nav->calcTileLoc(position, &x, &y);

	for( int y = miny; y <= maxy; ++y )
	{
		for( int x = minx; x <= maxx; ++x )
		{
	const dtMeshTile* tile = m_nav->getTilesAt( x, y );
	if( tile ) {
				polyID = queryCorrectPolygonsInTile2D( tile, position, ground );
				if( polyID != 0 ) {
					return DT_SUCCESS;
				}
			}
		}
	}

	return DT_FAILURE;
}

dtPolyRef	dtNavMeshQuery::queryCorrectPolygonsInTile2D( const dtMeshTile* tile, const dtCoordinates& pos, const bool ground ) const
{
	dtAssert( m_nav );
	dtAssert( tile->bvTree );

	dtPolyRef navigationMeshID = 0;
	unsigned short compareHeight = ground ? 0xffff : 0;

	const dtBVNode* node = &tile->bvTree[0];
	const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
	const dtCoordinates tbmin( tile->header->bmin );
	const dtCoordinates tbmax( tile->header->bmax );
	const float qfac = tile->header->bvQuantFactor;

	// Calculate quantized box
	unsigned short bmin[3], bmax[3];
	// dtClamp query box to world box.
	float minx = dtClamp(pos.X(), tbmin.X(), tbmax.X()) - tbmin.X();
	//float miny = dtClamp(pos.Y() - tile->header->walkableClimb, tbmin.Y(), tbmax.Y()) - tbmin.Y();
	float miny = 0;
	float minz = dtClamp(pos.Z(), tbmin.Z(), tbmax.Z()) - tbmin.Z();
	float maxx = dtClamp(pos.X(), tbmin.X(), tbmax.X()) - tbmin.X();
	//float maxy = dtClamp(pos.Y() + tile->header->walkableClimb, tbmin.Y(), tbmax.Y()) - tbmin.Y();
	float maxy = ground ? ( tbmax.Y() - tbmin.Y() ) : dtClamp(pos.Y() + tile->header->walkableClimb, tbmin.Y(), tbmax.Y()) - tbmin.Y();
	float maxz = dtClamp(pos.Z(), tbmin.Z(), tbmax.Z()) - tbmin.Z();
	// Quantize
	bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
	bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
	bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
	bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
	bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
	bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;

	// Traverse tree
	const dtPolyRef base = m_nav->getPolyRefBase(tile);
	while( node < end ) {
		const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;

		if( isLeafNode && overlap ) {
			const dtPolyRef pid = base | (dtPolyRef)node->i;
			const dtMeshTile* _tile	= 0;
			const dtPoly* _poly		= 0;
			m_nav->getTileAndPolyByRefUnsafe( pid, &_tile, &_poly );
			dtCoordinates verts[DT_VERTS_PER_POLYGON];
			const int vertCount = static_cast<int>( _poly->vertCount );
			for( int nth = 0; nth < vertCount; ++nth ) {
				dtVcopy( verts[nth], _tile->verts[_poly->verts[nth]] );
			}
			if( dtPointInPolygon( pos, verts, vertCount ) ) {
				const bool compare = ground ? node->bmin[1] < compareHeight : compareHeight < node->bmin[1];
				if( compare ) {
						compareHeight = node->bmin[1];
						navigationMeshID = pid;
					}
				}
		}

		if( overlap || isLeafNode ) {
			node++;
		}
		else {
			const int escapeIndex = -node->i;
			node += escapeIndex;
		}
	}

	return navigationMeshID;
}

dtStatus	dtNavMeshQuery::castRay( const dtPolyRef startPolyID, const dtCoordinates& startPosition, const dtCoordinates& endPosition, float& interpolationFactor, dtCoordinates& normal, dtPolyRef& endPolyID ) const
{
	dtAssert( m_nav );

	interpolationFactor = 0;
	normal = dtCoordinates();
	endPolyID = 0;

	if( !startPolyID || !m_nav->isValidPolyRef(startPolyID) )
		return DT_FAILURE | DT_INVALID_PARAM;

	dtPolyRef currentPolyID = startPolyID;
	dtStatus status = DT_SUCCESS;

	while( currentPolyID ) {
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		m_nav->getTileAndPolyByRefUnsafe( currentPolyID, &tile, &poly );

		dtCoordinates verts[DT_VERTS_PER_POLYGON];
		int nv = 0;
		for( int i = 0; i < (int)poly->vertCount; ++i ) {
			dtVcopy(verts[nv], tile->verts[poly->verts[i]]);
			++nv;
		}

		///*
		if( dtPointInPolygon( endPosition, verts, nv ) && dtPointInPolygon( startPosition, verts, nv ) ) {
			endPolyID = currentPolyID;
			interpolationFactor = FLT_MAX;
			return status;
		}
		//*/

		float tmin, tmax;
		int segMin, segMax;
		if( !dtIntersectSegmentPoly2D(startPosition, endPosition, verts, nv, tmin, tmax, segMin, segMax) ) {
			// Could not hit the polygon, keep the old t and report hit.
			return status;
		}

		// Keep track of furthest t so far.
		if( interpolationFactor < tmax ) {
			interpolationFactor = tmax;
		}

		// Store visited polygons.
		endPolyID = currentPolyID;

		// Ray end is completely inside the polygon.
		if( segMax == -1 ) {
			interpolationFactor = FLT_MAX;
			return status;
		}

		// Follow neighbours.
		dtPolyRef nextPolyID = 0;

		for( unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next ) {
			const dtLink* link = &tile->links[i];

			// Find link which contains this edge.
			if( (int)link->edge != segMax )
				continue;

			// Get pointer to the next polygon.
			const dtMeshTile* nextTile = 0;
			const dtPoly* nextPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(link->ref, &nextTile, &nextPoly);

			// Skip off-mesh connections.
			if( nextPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION )
				continue;

			// If the link is internal, just return the ref.
			if( link->side == 0xff ) {
				nextPolyID = link->ref;
				break;
			}

			// Check if the link spans the whole edge, and accept.
			if( link->bmin == 0 && link->bmax == 0xff ) {
				nextPolyID = link->ref;
				break;
			}

			// Check for partial edge links.
			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
			const dtCoordinates left( tile->verts[v0] );
			const dtCoordinates right( tile->verts[v1] );

			// Check that the intersection lies inside the link portal.
			if( link->side == 0 || link->side == 4 ) {
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left.Z() + (right.Z() - left.Z())*(link->bmin*s);
				float lmax = left.Z() + (right.Z() - left.Z())*(link->bmax*s);
				if( lmax < lmin )
					dtSwap( lmin, lmax );

				// Find Z intersection.
				float z = startPosition.Z() + (endPosition.Z()-startPosition.Z())*tmax;
				if( z >= lmin && z <= lmax ) {
					nextPolyID = link->ref;
					break;
				}
			}
			else if( link->side == 2 || link->side == 6 ) {
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left.X() + (right.X() - left.X())*(link->bmin*s);
				float lmax = left.X() + (right.X() - left.X())*(link->bmax*s);
				if( lmax < lmin )
					dtSwap( lmin, lmax );

				// Find X intersection.
				float x = startPosition.X() + (endPosition.X()-startPosition.X())*tmax;
				if( lmin <= x && x <= lmax ) {
					nextPolyID = link->ref;
					break;
				}
			}
		}

		if( !nextPolyID ) {
			// No neighbour, we hit a wall.

			// Calculate hit normal.
			const int a = segMax;
			const int b = segMax+1 < nv ? segMax+1 : 0;
			const dtCoordinates va( verts[a] );
			const dtCoordinates vb( verts[b] );
			const float dx = vb.X() - va.X();
			const float dz = vb.Z() - va.Z();
			normal.SetX( dz );
			normal.SetY( 0 );
			normal.SetZ( -dx );
			dtVnormalize(normal);

			return status;
		}

		// No hit, advance to neighbour polygon.
		currentPolyID = nextPolyID;
	}

	return status;
}

dtStatus	dtNavMeshQuery::findMovablePosition( const dtPolyRef startPolyID, const dtCoordinates& startPosition, const dtCoordinates& endPosition, dtPolyRef& resultPolyID, dtCoordinates& resultPosition, dtCoordinates& normal ) const
{
	dtAssert( m_nav );
	if( startPolyID == 0 || !m_nav->isValidPolyRef( startPolyID ) ) {
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	normal = dtCoordinates();
	resultPolyID = 0;

	dtPolyRef currentPolyID( startPolyID );
	resultPolyID = currentPolyID;
	resultPosition = startPosition;
	do {
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		m_nav->getTileAndPolyByRefUnsafe( currentPolyID, &tile, &poly );

		dtCoordinates verts[DT_VERTS_PER_POLYGON];
		const int vertCount = static_cast<int>( poly->vertCount );
		for( int i = 0; i < vertCount; ++i ) {
			dtVcopy( verts[i], tile->verts[poly->verts[i]] );
		}

		if( dtPointInPolygon( endPosition, verts, vertCount ) ) {
			resultPolyID = currentPolyID;
			dtVcopy( resultPosition, endPosition );
			return DT_SUCCESS;
		}

		float tmin, tmax;
		int segMin, segMax;
		if( !_dtIntersectSegmentPoly2D( startPosition, endPosition, verts, vertCount, tmin, tmax, segMin, segMax, resultPosition ) ) {
			return DT_SUCCESS | DT_COLLISION;
		}

		resultPolyID = currentPolyID;

		if( segMax == -1 ) {
			dtVcopy( resultPosition, endPosition );
			return DT_SUCCESS;
		}

		dtPolyRef nextPolyID( 0 );
		for( unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next ) {
			const dtLink* link = &tile->links[i];
			if( static_cast<int>( link->edge ) != segMax ) {
				continue;
			}

			//////////////////////////////////////////////////////////////////////////
			const dtMeshTile* nextTile = 0;
			const dtPoly* nextPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(link->ref, &nextTile, &nextPoly);

			if( link->side == 0xff ) {
				nextPolyID = link->ref;
				break;
			}

			if( link->bmin == 0 && link->bmax == 0xff ) {
				nextPolyID = link->ref;
				break;
			}

			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
			const dtCoordinates left( tile->verts[v0] );
			const dtCoordinates right( tile->verts[v1] );

			if( link->side == 0 || link->side == 4 ) {
				const float s = 1.0f/255.0f;
				float lmin = left.Z() + (right.Z() - left.Z())*(link->bmin*s);
				float lmax = left.Z() + (right.Z() - left.Z())*(link->bmax*s);
				if( lmax < lmin )
					dtSwap( lmin, lmax );

				const float z = startPosition.Z() + (endPosition.Z()-startPosition.Z())*tmax;
				if( z >= lmin && z <= lmax ) {
				nextPolyID = link->ref;
				break;
			}
		}
			else if( link->side == 2 || link->side == 6 ) {
				const float s = 1.0f/255.0f;
				float lmin = left.X() + (right.X() - left.X())*(link->bmin*s);
				float lmax = left.X() + (right.X() - left.X())*(link->bmax*s);
				if( lmax < lmin )
					dtSwap( lmin, lmax );

				const float x = startPosition.X() + (endPosition.X()-startPosition.X())*tmax;
				if( lmin <= x && x <= lmax ) {
					nextPolyID = link->ref;
					break;
				}
			}
			//////////////////////////////////////////////////////////////////////////
		}

		if( nextPolyID == 0 && dtIsJumpableMeshType( poly->getType() ) ) {
			for( unsigned int i = poly->jumpMeshFirstLink; i != DT_NULL_LINK; i = tile->jumpMeshLink[i].next ) {
				const dtJumpMeshLink* jumpMeshLink = &tile->jumpMeshLink[i];

				const dtMeshTile* next_tile = 0;
				const dtPoly* next_poly = 0;
				m_nav->getTileAndPolyByRefUnsafe( jumpMeshLink->ref, &next_tile, &next_poly );

				dtCoordinates next_verts[DT_VERTS_PER_POLYGON];
				const int next_vertCount = static_cast<int>( next_poly->vertCount );
				for( int i = 0; i < next_vertCount; ++i ) {
					dtVcopy( next_verts[i], next_tile->verts[next_poly->verts[i]] );
				}

				if( dtPointInPolygon( endPosition, next_verts, next_vertCount ) ) {
					float height = 0;
					if( dtStatusSucceed( getCorrectPolyHeight( jumpMeshLink->ref, endPosition, height ) ) ) {
					if( height <= endPosition.Y() ) {
						nextPolyID = jumpMeshLink->ref;
						break;
						}
					}
				}
			}
		}

		if( nextPolyID == 0 ) {
			const int a = segMax;
			const int b = segMax+1 < vertCount ? segMax+1 : 0;
			const dtCoordinates va( verts[a] );
			const dtCoordinates vb( verts[b] );
			const float dx = vb.X() - va.X();
			const float dz = vb.Z() - va.Z();
			normal.SetX( dz );
			normal.SetY( 0 );
			normal.SetZ( -dx );
			dtVnormalize( normal );
		}

		currentPolyID = nextPolyID;

	} while ( currentPolyID != 0 );

	return resultPolyID != 0 ? DT_SUCCESS | DT_COLLISION : DT_FAILURE;
}

dtStatus	dtNavMeshQuery::getCorrectPolyHeight( const dtPolyRef polyID, const dtCoordinates& position, float& height ) const
{
	dtAssert( m_nav );

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	m_nav->getTileAndPolyByRefUnsafe( polyID, &tile, &poly );

	const unsigned int ip = static_cast<unsigned int>( poly - tile->polys );
	const dtPolyDetail* pd = &tile->detailMeshes[ip];
	const int triCount = static_cast<int>( pd->triCount );
	for( int nth = 0; nth < triCount; ++nth ) {
		const unsigned char* t = &tile->detailTris[(pd->triBase+nth)*4];
		dtCoordinates verts[3];
		for( int i = 0; i < 3; ++i ) {
			if( t[i] < poly->vertCount ) {
				dtVcopy( verts[i], tile->verts[poly->verts[t[i]]] );
			}
			else {
				dtVcopy( verts[i], tile->detailVerts[(pd->vertBase+(t[i]-poly->vertCount))] );
			}
		}
		if( dtClosestHeightPointTriangle( position, verts[0], verts[1], verts[2], height ) ) {
			return DT_SUCCESS;
		}
	}

	return DT_FAILURE;
}
// MIRCHANG
//////////////////////////////////////////////////////////////////////////
