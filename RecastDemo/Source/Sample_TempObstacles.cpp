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
#include <string.h>
#include <float.h>
#include <new>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TempObstacles.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourAssert.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "fastlz.h"
#include <DetourCoordinates.h>

#ifdef WIN32
#	define snprintf _snprintf
#endif


// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static const int EXPECTED_LAYERS_PER_TILE = 4;


static bool isectSegAABB(const dtCoordinates& _sp, const dtCoordinates& _sq,
						 const dtCoordinates& _amin, const dtCoordinates& _amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	dtCoordinates _d;
	rcVsub(_d, _sq, _sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)

	//////////////////////////////////////////////////////////////////////////
	float d[3], sp[3], sq[3], amin[3], amax[3];
	TransformCoordinates::transform( _d, d);
	TransformCoordinates::transform( _sp, sp);
	TransformCoordinates::transform( _sq, sq);
	TransformCoordinates::transform( _amin, amin);
	TransformCoordinates::transform( _amax, amax);
	//////////////////////////////////////////////////////////////////////////
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) rcSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize*4;
}




struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}
	
	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}
	
	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	int capacity;
	int top;
	int high;
	
	LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}
	
	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const int cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}
	
	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}
	
	virtual void* alloc(const int size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}
	
	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	InputGeom* m_geom;

	inline MeshProcess() : m_geom(0)
	{
	}

	inline void init(InputGeom* geom)
	{
		m_geom = geom;
	}
	
	virtual void process(struct dtNavMeshCreateParams* params,
						 unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

#ifndef MODIFY_OFF_MESH_CONNECTION
		// Pass in off-mesh connections.
		if (m_geom)
		{
			params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
			params->offMeshConRad = m_geom->getOffMeshConnectionRads();
			params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
			params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
			params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
			params->offMeshConUserID = m_geom->getOffMeshConnectionId();
			params->offMeshConCount = m_geom->getOffMeshConnectionCount();	
		}
#endif // !MODIFY_OFF_MESH_CONNECTION
	}
};




static const int MAX_LAYERS = 32;

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

struct RasterizationContext
{
	RasterizationContext() :
		solid(0),
		triareas(0),
		lset(0),
		chf(0),
		ntiles(0)
	{
		memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
	}
	
	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		delete [] triareas;
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(tiles[i].data);
			tiles[i].data = 0;
		}
	}
	
	rcHeightfield* solid;
	unsigned char* triareas;
	rcHeightfieldLayerSet* lset;
	rcCompactHeightfield* chf;
	TileCacheData tiles[MAX_LAYERS];
	int ntiles;
};

static int rasterizeTileLayers(BuildContext* ctx, InputGeom* geom,
							   const int tx, const int ty,
							   const rcConfig& cfg,
							   TileCacheData* tiles,
							   const int maxTiles)
{
	if (!geom || !geom->getMesh() || !geom->getChunkyMesh())
	{
		ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
		return 0;
	}
	
	FastLZCompressor comp;
	RasterizationContext rc;
	
	const dtCoordinates* verts = geom->getMesh()->getVerts();
	const int nverts = geom->getMesh()->getVertCount();
	const rcChunkyTriMesh* chunkyMesh = geom->getChunkyMesh();
	
	// Tile bounds.
	const float tcs = cfg.tileSize * cfg.cs;
	
	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));

	tcfg.bmin.SetX( cfg.bmin.X() + tx*tcs );
	tcfg.bmin.SetY( cfg.bmin.Y() );
	tcfg.bmin.SetZ( cfg.bmin.Z() + ty*tcs );
	tcfg.bmax.SetX( cfg.bmin.X() + (tx+1)*tcs );
	tcfg.bmax.SetY( cfg.bmax.Y() );
	tcfg.bmax.SetZ( cfg.bmin.Z() + (ty+1)*tcs );
	tcfg.bmin.SetX( tcfg.bmin.X() - tcfg.borderSize*tcfg.cs );
	tcfg.bmin.SetZ( tcfg.bmin.Z() - tcfg.borderSize*tcfg.cs );
	tcfg.bmax.SetX( tcfg.bmax.X() + tcfg.borderSize*tcfg.cs );
	tcfg.bmax.SetZ( tcfg.bmax.Z() + tcfg.borderSize*tcfg.cs );
	
	// Allocate voxel heightfield where we rasterize our input data to.
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}
	
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!rc.triareas)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}
	
	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.bmin.X();
	tbmin[1] = tcfg.bmin.Z();
	tbmax[0] = tcfg.bmax.X();
	tbmax[1] = tcfg.bmax.Z();
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0; // empty
	}
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i*3];
		const int ntris = node.n;
		
		memset(rc.triareas, 0, ntris*sizeof(unsigned char));
		rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle,
								verts, nverts, tris, ntris, rc.triareas);
		
		rcRasterizeTriangles(ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
	}
	
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *rc.solid);
	rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
	rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *rc.solid);
	
	
	rc.chf = rcAllocCompactHeightfield();
	if (!rc.chf)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}
	
	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(ctx, tcfg.walkableRadius, *rc.chf))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}
	
	// (Optional) Mark areas.
	const ConvexVolume* vols = geom->getConvexVolumes();
	for (int i  = 0; i < geom->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(ctx, vols[i].verts, vols[i].nverts,
							 vols[i].hmin, vols[i].hmax,
							 (unsigned char)vols[i].area, *rc.chf);
	}
	
	rc.lset = rcAllocHeightfieldLayerSet();
	if (!rc.lset)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}
	
	rc.ntiles = 0;
	for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.tiles[rc.ntiles++];
		const rcHeightfieldLayer* layer = &rc.lset->layers[i];
		
		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;
		
		// Tile layer location in the navmesh.
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);
		
		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
												&tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rc.tiles[i];
		rc.tiles[i].data = 0;
		rc.tiles[i].dataSize = 0;
	}
	
	return n;
}


void drawTiles(duDebugDraw* dd, dtTileCache* tc)
{
	unsigned int fcol[6];
	dtCoordinates bmin, bmax;

	for (int i = 0; i < tc->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tc->getTile(i);
		if (!tile->header) continue;
		
		tc->calcTightTileBounds(tile->header, bmin, bmax);
		
		const unsigned int col = duIntToCol(i,64);
		duCalcBoxColors(fcol, col, col);
		duDebugDrawBox(dd, bmin.X(),bmin.Y(),bmin.Z(), bmax.X(),bmax.Y(),bmax.Z(), fcol);
	}
	
	for (int i = 0; i < tc->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tc->getTile(i);
		if (!tile->header) continue;
		
		tc->calcTightTileBounds(tile->header, bmin, bmax);
		
		const unsigned int col = duIntToCol(i,255);
		const float pad = tc->getParams()->cs * 0.1f;
		duDebugDrawBoxWire(dd, bmin.X()-pad,bmin.Y()-pad,bmin.Z()-pad,
						   bmax.X()+pad,bmax.Y()+pad,bmax.Z()+pad, col, 2.0f);
	}

}

enum DrawDetailType
{
	DRAWDETAIL_AREAS,
	DRAWDETAIL_REGIONS,
	DRAWDETAIL_CONTOURS,
	DRAWDETAIL_MESH,
};

void drawDetail(duDebugDraw* dd, dtTileCache* tc, const int tx, const int ty, int type)
{
	struct TileCacheBuildContext
	{
		inline TileCacheBuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
		inline ~TileCacheBuildContext() { purge(); }
		void purge()
		{
			dtFreeTileCacheLayer(alloc, layer);
			layer = 0;
			dtFreeTileCacheContourSet(alloc, lcset);
			lcset = 0;
			dtFreeTileCachePolyMesh(alloc, lmesh);
			lmesh = 0;
		}
		struct dtTileCacheLayer* layer;
		struct dtTileCacheContourSet* lcset;
		struct dtTileCachePolyMesh* lmesh;
		struct dtTileCacheAlloc* alloc;
	};

	dtCompressedTileRef tiles[MAX_LAYERS];
	const int ntiles = tc->getTilesAt(tx,ty,tiles,MAX_LAYERS);

	dtTileCacheAlloc* talloc = tc->getAlloc();
	dtTileCacheCompressor* tcomp = tc->getCompressor();
	const dtTileCacheParams* params = tc->getParams();

	for (int i = 0; i < ntiles; ++i)
	{
		const dtCompressedTile* tile = tc->getTileByRef(tiles[i]);

		talloc->reset();

		TileCacheBuildContext bc(talloc);
		const int walkableClimbVx = (int)(params->walkableClimb / params->ch);
		dtStatus status;
		
		// Decompress tile layer data. 
		status = dtDecompressTileCacheLayer(talloc, tcomp, tile->data, tile->dataSize, &bc.layer);
		if (dtStatusFailed(status))
			return;
		if (type == DRAWDETAIL_AREAS)
		{
			duDebugDrawTileCacheLayerAreas(dd, *bc.layer, params->cs, params->ch);
			continue;
		}

		// Build navmesh
		status = dtBuildTileCacheRegions(talloc, *bc.layer, walkableClimbVx);
		if (dtStatusFailed(status))
			return;
		if (type == DRAWDETAIL_REGIONS)
		{
			duDebugDrawTileCacheLayerRegions(dd, *bc.layer, params->cs, params->ch);
			continue;
		}
		
		bc.lcset = dtAllocTileCacheContourSet(talloc);
		if (!bc.lcset)
			return;
		status = dtBuildTileCacheContours(talloc, *bc.layer, walkableClimbVx,
										  params->maxSimplificationError, *bc.lcset);
		if (dtStatusFailed(status))
			return;
		if (type == DRAWDETAIL_CONTOURS)
		{
			duDebugDrawTileCacheContours(dd, *bc.lcset, tile->header->bmin, params->cs, params->ch);
			continue;
		}
		
		bc.lmesh = dtAllocTileCachePolyMesh(talloc);
		if (!bc.lmesh)
			return;
		status = dtBuildTileCachePolyMesh(talloc, *bc.lcset, *bc.lmesh);
		if (dtStatusFailed(status))
			return;

		if (type == DRAWDETAIL_MESH)
		{
			duDebugDrawTileCachePolyMesh(dd, *bc.lmesh, tile->header->bmin, params->cs, params->ch);
			continue;
		}

	}
}


void drawDetailOverlay(const dtTileCache* tc, const int tx, const int ty, double* proj, double* model, int* view)
{
	dtCompressedTileRef tiles[MAX_LAYERS];
	const int ntiles = tc->getTilesAt(tx,ty,tiles,MAX_LAYERS);
	if (!ntiles)
		return;
	
	const int rawSize = calcLayerBufferSize(tc->getParams()->width, tc->getParams()->height);
	
	char text[128];

	for (int i = 0; i < ntiles; ++i)
	{
		const dtCompressedTile* tile = tc->getTileByRef(tiles[i]);
		
		const dtCoordinates pos( (tile->header->bmin.X()+tile->header->bmax.X())/2.0f, tile->header->bmin.Y(), (tile->header->bmin.Z()+tile->header->bmax.Z())/2.0f );
		
		GLdouble x, y, z;
		if (gluProject((GLdouble)pos.X(), (GLdouble)pos.Y(), (GLdouble)pos.Z(),
					   model, proj, view, &x, &y, &z))
		{
			snprintf(text,128,"(%d,%d)/%d", tile->header->tx,tile->header->ty,tile->header->tlayer);
			imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
			snprintf(text,128,"Compressed: %.1f kB", tile->dataSize/1024.0f);
			imguiDrawText((int)x, (int)y-45, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,128));
			snprintf(text,128,"Raw:%.1fkB", rawSize/1024.0f);
			imguiDrawText((int)x, (int)y-65, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,128));
		}
	}
}
		
dtObstacleRef hitTestObstacle(const dtTileCache* tc, const dtCoordinates& sp, const dtCoordinates& sq)
{
	float tmin = FLT_MAX;
	const dtTileCacheObstacle* obmin = 0;
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY)
			continue;
		
		dtCoordinates bmin, bmax;
		float t0,t1;
		tc->getObstacleBounds(ob, bmin,bmax);
		
		if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
		{
			if (t0 < tmin)
			{
				tmin = t0;
				obmin = ob;
			}
		}
	}
	return tc->getObstacleRef(obmin);
}
	
void drawObstacles(duDebugDraw* dd, const dtTileCache* tc)
{
	// Draw obstacles
	for (int i = 0; i < tc->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = tc->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY) continue;
		dtCoordinates bmin, bmax;
		tc->getObstacleBounds(ob, bmin,bmax);

		unsigned int col = 0;
		if (ob->state == DT_OBSTACLE_PROCESSING)
			col = duRGBA(255,255,0,128);
		else if (ob->state == DT_OBSTACLE_PROCESSED)
			col = duRGBA(255,192,0,192);
		else if (ob->state == DT_OBSTACLE_REMOVING)
			col = duRGBA(220,0,0,128);

		duDebugDrawCylinder(dd, bmin.X(),bmin.Y(),bmin.Z(), bmax.X(),bmax.Y(),bmax.Z(), col);
		duDebugDrawCylinderWire(dd, bmin.X(),bmin.Y(),bmin.Z(), bmax.X(),bmax.Y(),bmax.Z(), duDarkenCol(col), 2);
	}
}




class TempObstacleHilightTool : public SampleTool
{
	Sample_TempObstacles* m_sample;
	dtCoordinates m_hitPos;
	bool m_hitPosSet;
	float m_agentRadius;
	int m_drawType;
	
public:

	TempObstacleHilightTool() :
		m_sample(0),
		m_hitPosSet(false),
		m_agentRadius(0),
		m_drawType(DRAWDETAIL_AREAS)
	{
	}

	virtual ~TempObstacleHilightTool()
	{
	}

	virtual int type() { return TOOL_TILE_HIGHLIGHT; }

	virtual void init(Sample* sample)
	{
		m_sample = (Sample_TempObstacles*)sample; 
	}
	
	virtual void reset() {}

	virtual void handleMenu()
	{
		imguiLabel("Highlight Tile Cache");
		imguiValue("Click LMB to highlight a tile.");
		imguiSeparator();
		if (imguiCheck("Draw Areas", m_drawType == DRAWDETAIL_AREAS))
			m_drawType = DRAWDETAIL_AREAS;
		if (imguiCheck("Draw Regions", m_drawType == DRAWDETAIL_REGIONS))
			m_drawType = DRAWDETAIL_REGIONS;
		if (imguiCheck("Draw Contours", m_drawType == DRAWDETAIL_CONTOURS))
			m_drawType = DRAWDETAIL_CONTOURS;
		if (imguiCheck("Draw Mesh", m_drawType == DRAWDETAIL_MESH))
			m_drawType = DRAWDETAIL_MESH;
	}

	virtual void handleClick(const dtCoordinates& /*s*/, const dtCoordinates& p, bool /*shift*/)
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos,p);
	}

	virtual void handleToggle() {}

	virtual void handleStep() {}

	virtual void handleUpdate(const float /*dt*/) {}
	
	virtual void handleRender()
	{
		if (m_hitPosSet)
		{
			const float s = m_sample->getAgentRadius();
			glColor4ub(0,0,0,128);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(m_hitPos.X()-s,m_hitPos.Y()+0.1f,m_hitPos.Z());
			glVertex3f(m_hitPos.X()+s,m_hitPos.Y()+0.1f,m_hitPos.Z());
			glVertex3f(m_hitPos.X(),m_hitPos.Y()-s+0.1f,m_hitPos.Z());
			glVertex3f(m_hitPos.X(),m_hitPos.Y()+s+0.1f,m_hitPos.Z());
			glVertex3f(m_hitPos.X(),m_hitPos.Y()+0.1f,m_hitPos.Z()-s);
			glVertex3f(m_hitPos.X(),m_hitPos.Y()+0.1f,m_hitPos.Z()+s);
			glEnd();
			glLineWidth(1.0f);
			
			if (m_sample)
			{
				int tx=0, ty=0;
				m_sample->getTilePos(m_hitPos, tx, ty);
				m_sample->renderCachedTile(tx,ty,m_drawType);
			}

		}
	}
	
	virtual void handleRenderOverlay(double* proj, double* model, int* view)
	{
		if (m_hitPosSet)
		{
			if (m_sample)
			{
				int tx=0, ty=0;
				m_sample->getTilePos(m_hitPos, tx, ty);
				m_sample->renderCachedTileOverlay(tx,ty,proj,model,view);
			}
		}		
	}
};


class TempObstacleCreateTool : public SampleTool
{
	Sample_TempObstacles* m_sample;
	
public:
	
	TempObstacleCreateTool()
	{
	}
	
	virtual ~TempObstacleCreateTool()
	{
	}
	
	virtual int type() { return TOOL_TEMP_OBSTACLE; }
	
	virtual void init(Sample* sample)
	{
		m_sample = (Sample_TempObstacles*)sample; 
	}
	
	virtual void reset() {}
	
	virtual void handleMenu()
	{
		imguiLabel("Create Temp Obstacles");
		
		if (imguiButton("Remove All"))
			m_sample->clearAllTempObstacles();
		
		imguiSeparator();

		imguiValue("Click LMB to create an obstacle.");
		imguiValue("Shift+LMB to remove an obstacle.");
	}
	
	virtual void handleClick(const dtCoordinates& s, const dtCoordinates& p, bool shift)
	{
		if (m_sample)
		{
			if (shift)
				m_sample->removeTempObstacle(s,p);
			else
				m_sample->addTempObstacle(p);
		}
	}
	
	virtual void handleToggle() {}
	virtual void handleStep() {}
	virtual void handleUpdate(const float /*dt*/) {}
	virtual void handleRender() {}
	virtual void handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) { }
};





Sample_TempObstacles::Sample_TempObstacles() :
	m_keepInterResults(false),
	m_tileCache(0),
	m_cacheBuildTimeMs(0),
	m_cacheCompressedSize(0),
	m_cacheRawSize(0),
	m_cacheLayerCount(0),
	m_cacheBuildMemUsage(0),
	m_drawMode(DRAWMODE_NAVMESH),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileSize(48)
{
	resetCommonSettings();
	
	m_talloc = new LinearAllocator(32000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	
	setTool(new TempObstacleCreateTool);
}

Sample_TempObstacles::~Sample_TempObstacles()
{
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
	dtFreeTileCache(m_tileCache);
}

void Sample_TempObstacles::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_tileSize, 16.0f, 128.0f, 8.0f);
	
	int gridSize = 1;
	if (m_geom)
	{
		const dtCoordinates bmin( m_geom->getMeshBoundsMin() );
		const dtCoordinates bmax( m_geom->getMeshBoundsMax() );
		char text[64];
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		const int ts = (int)m_tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
		imguiValue(text);
		gridSize = tw*th;
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}
	
	imguiSeparator();
	
	imguiLabel("Tile Cache");
	char msg[64];

	const float compressionRatio = (float)m_cacheCompressedSize / (float)(m_cacheRawSize+1);
	
	snprintf(msg, 64, "Layers  %d", m_cacheLayerCount);
	imguiValue(msg);
	snprintf(msg, 64, "Layers (per tile)  %.1f", (float)m_cacheLayerCount/(float)gridSize);
	imguiValue(msg);
	
	snprintf(msg, 64, "Memory  %.1f kB / %.1f kB (%.1f%%)", m_cacheCompressedSize/1024.0f, m_cacheRawSize/1024.0f, compressionRatio*100.0f);
	imguiValue(msg);
	snprintf(msg, 64, "Navmesh Build Time  %.1f ms", m_cacheBuildTimeMs);
	imguiValue(msg);
	snprintf(msg, 64, "Build Peak Mem Usage  %.1f kB", m_cacheBuildMemUsage/1024.0f);
	imguiValue(msg);
	
	imguiSeparator();
}

void Sample_TempObstacles::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Highlight Tile Cache", type == TOOL_TILE_HIGHLIGHT))
	{
		setTool(new TempObstacleHilightTool);
	}
	if (imguiCheck("Create Temp Obstacles", type == TOOL_TEMP_OBSTACLE))
	{
		setTool(new TempObstacleCreateTool);
	}
	if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();
}

void Sample_TempObstacles::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;
	
	if (m_geom)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_CACHE_BOUNDS] = true;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Navmesh Portals", m_drawMode == DRAWMODE_NAVMESH_PORTALS, valid[DRAWMODE_NAVMESH_PORTALS]))
		m_drawMode = DRAWMODE_NAVMESH_PORTALS;
	if (imguiCheck("Cache Bounds", m_drawMode == DRAWMODE_CACHE_BOUNDS, valid[DRAWMODE_CACHE_BOUNDS]))
		m_drawMode = DRAWMODE_CACHE_BOUNDS;
	
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}
}

void Sample_TempObstacles::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;
	
	DebugDrawGL dd;

	const float texScale = 1.0f / (m_cellSize * 10.0f);
	
	// Draw mesh
	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
								m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
								m_agentMaxSlope, texScale);

#ifndef MODIFY_OFF_MESH_CONNECTION
		m_geom->drawOffMeshConnections(&dd);
#endif // !MODIFY_OFF_MESH_CONNECTION
	}
	
	if (m_tileCache && m_drawMode == DRAWMODE_CACHE_BOUNDS)
		drawTiles(&dd, m_tileCache);
	
	if (m_tileCache)
		drawObstacles(&dd, m_tileCache);
	
	
	glDepthMask(GL_FALSE);
	
	// Draw bounds
	const dtCoordinates bmin( m_geom->getMeshBoundsMin() );
	const dtCoordinates bmax( m_geom->getMeshBoundsMax() );
	duDebugDrawBoxWire(&dd, bmin.X(),bmin.Y(),bmin.Z(), bmax.X(),bmax.Y(),bmax.Z(), duRGBA(255,255,255,128), 1.0f);
	
	// Tiling grid.
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize-1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize-1) / (int)m_tileSize;
	const float s = m_tileSize*m_cellSize;
	duDebugDrawGridXZ(&dd, bmin.X(),bmin.Y(),bmin.Z(), tw,th, s, duRGBA(0,0,0,64), 1.0f);
		
	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		 m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		 m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags/*|DU_DRAWNAVMESH_COLOR_TILES*/);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_PORTALS)
			duDebugDrawNavMeshPortals(&dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&dd, *m_navQuery);
		duDebugDrawNavMeshPolysWithFlags(&dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}
	
	
	glDepthMask(GL_TRUE);
		
	m_geom->drawConvexVolumes(&dd);
	
	if (m_tool)
		m_tool->handleRender();
	renderToolStates();
	
	glDepthMask(GL_TRUE);
}

void Sample_TempObstacles::renderCachedTile(const int tx, const int ty, const int type)
{
	DebugDrawGL dd;
	if (m_tileCache)
		drawDetail(&dd,m_tileCache,tx,ty,type);
}

void Sample_TempObstacles::renderCachedTileOverlay(const int tx, const int ty, double* proj, double* model, int* view)
{
	if (m_tileCache)
		drawDetailOverlay(m_tileCache, tx, ty, proj, model, view);
}

void Sample_TempObstacles::handleRenderOverlay(double* proj, double* model, int* view)
{	
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);

	// Stats
/*	imguiDrawRect(280,10,300,100,imguiRGBA(0,0,0,64));
	
	char text[64];
	int y = 110-30;
	
	snprintf(text,64,"Lean Data: %.1fkB", m_tileCache->getRawSize()/1024.0f);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;
	
	snprintf(text,64,"Compressed: %.1fkB (%.1f%%)", m_tileCache->getCompressedSize()/1024.0f,
			 m_tileCache->getRawSize() > 0 ? 100.0f*(float)m_tileCache->getCompressedSize()/(float)m_tileCache->getRawSize() : 0);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;

	if (m_rebuildTileCount > 0 && m_rebuildTime > 0.0f)
	{
		snprintf(text,64,"Changed obstacles, rebuild %d tiles: %.3f ms", m_rebuildTileCount, m_rebuildTime);
		imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,192,0,255));
		y -= 20;
	}
	*/
}

void Sample_TempObstacles::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	dtFreeTileCache(m_tileCache);
	m_tileCache = 0;
	
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
		m_tmproc->init(m_geom);
	}
	resetToolStates();
	initToolStates(this);
}

void Sample_TempObstacles::addTempObstacle(const dtCoordinates& pos)
{
	if (!m_tileCache)
		return;
	dtCoordinates p;
	dtVcopy(p, pos);
	p.SetY( p.Y() - 0.5f );
	m_tileCache->addObstacle(p, 1.0f, 2.0f, 0);
}

void Sample_TempObstacles::removeTempObstacle(const dtCoordinates& sp, const dtCoordinates& sq)
{
	if (!m_tileCache)
		return;
	dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
	m_tileCache->removeObstacle(ref);
}

void Sample_TempObstacles::clearAllTempObstacles()
{
	if (!m_tileCache)
		return;
	for (int i = 0; i < m_tileCache->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
		if (ob->state == DT_OBSTACLE_EMPTY) continue;
		m_tileCache->removeObstacle(m_tileCache->getObstacleRef(ob));
	}
}

bool Sample_TempObstacles::handleBuild()
{
	dtStatus status;
	
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	m_tmproc->init(m_geom);
	
	// Init cache
	const dtCoordinates bmin( m_geom->getMeshBoundsMin() );
	const dtCoordinates bmax( m_geom->getMeshBoundsMax() );
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;

	// Generation params.
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	cfg.maxSimplificationError = m_edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	cfg.tileSize = (int)m_tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize*2;
	cfg.height = cfg.tileSize + cfg.borderSize*2;
	cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);
	
	// Tile cache params.
	dtTileCacheParams tcparams;
	memset(&tcparams, 0, sizeof(tcparams));
	rcVcopy(tcparams.orig, bmin);
	tcparams.cs = m_cellSize;
	tcparams.ch = m_cellHeight;
	tcparams.width = (int)m_tileSize;
	tcparams.height = (int)m_tileSize;
	tcparams.walkableHeight = m_agentHeight;
	tcparams.walkableRadius = m_agentRadius;
	tcparams.walkableClimb = m_agentMaxClimb;
	tcparams.maxSimplificationError = m_edgeMaxError;
	tcparams.maxTiles = tw*th*EXPECTED_LAYERS_PER_TILE;
	tcparams.maxObstacles = 128;

	dtFreeTileCache(m_tileCache);
	
	m_tileCache = dtAllocTileCache();
	if (!m_tileCache)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return false;
	}
	status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return false;
	}
	
	dtFreeNavMesh(m_navMesh);
	
	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	rcVcopy(params.orig, m_geom->getMeshBoundsMin());
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;
	
	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}
	
	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}
	

	// Preprocess tiles.
	
	m_ctx->resetTimers();
	
	m_cacheLayerCount = 0;
	m_cacheCompressedSize = 0;
	m_cacheRawSize = 0;
	
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(m_ctx, m_geom, x, y, cfg, tiles, MAX_LAYERS);

			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}
				
				m_cacheLayerCount++;
				m_cacheCompressedSize += tile->dataSize;
				m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}

	// Build initial meshes
	m_ctx->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			m_tileCache->buildNavMeshTilesAt(x,y, m_navMesh);
	m_ctx->stopTimer(RC_TIMER_TOTAL);
	
	m_cacheBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
	m_cacheBuildMemUsage = m_talloc->high;
	

	const dtNavMesh* nav = m_navMesh;
	int navmeshMemUsage = 0;
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nav->getTile(i);
		if (tile->header)
			navmeshMemUsage += tile->dataSize;
	}
	printf("navmeshMemUsage = %.1f kB", navmeshMemUsage/1024.0f);
		
	
	if (m_tool)
		m_tool->init(this);
	initToolStates(this);

	return true;
}

void Sample_TempObstacles::handleUpdate(const float dt)
{
	Sample::handleUpdate(dt);
	
	if (!m_navMesh)
		return;
	if (!m_tileCache)
		return;
	
	m_tileCache->update(dt, m_navMesh);
}

void Sample_TempObstacles::getTilePos(const dtCoordinates& pos, int& tx, int& ty)
{
	if (!m_geom) return;
	
	const dtCoordinates bmin( m_geom->getMeshBoundsMin() );
	
	const float ts = m_tileSize*m_cellSize;
	tx = (int)((pos.X() - bmin.X()) / ts);
	ty = (int)((pos.Z() - bmin.Z()) / ts);
}
