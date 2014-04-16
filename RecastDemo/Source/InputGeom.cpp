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
#include <ctype.h>
#include <string.h>
#include "Recast.h"
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"

static bool intersectSegmentTriangle(const dtCoordinates& sp, const dtCoordinates& sq,
									 const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c,
									 float &t)
{
	float v, w;
	dtCoordinates ab, ac, qp, ap, norm, e;
	rcVsub(ab, b, a);
	rcVsub(ac, c, a);
	rcVsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	rcVcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = rcVdot(qp, norm);
	if (d <= 0.0f) return false;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	rcVsub(ap, sp, a);
	t = rcVdot(ap, norm);
	if (t < 0.0f) return false;
	if (t > d) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	rcVcross(e, qp, ap);
	v = rcVdot(ac, e);
	if (v < 0.0f || v > d) return false;
	w = -rcVdot(ab, e);
	if (w < 0.0f || v + w > d) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	t /= d;
	
	return true;
}

static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
			default:
				start = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	return buf;
}



InputGeom::InputGeom() :
	m_chunkyMesh(0),
	m_mesh(0),
#ifndef MODIFY_OFF_MESH_CONNECTION
	m_offMeshConCount(0),
#endif // !MODIFY_OFF_MESH_CONNECTION
	m_volumeCount(0)
#ifdef INTEGRATION_BUILD
	, m_integrationBuild( false )
#endif // INTEGRATION_BUILD
{
}

InputGeom::~InputGeom()
{
	delete m_chunkyMesh;
	delete m_mesh;
}
		
bool InputGeom::loadMesh(rcContext* ctx, const char* filepath)
{
	m_path = filepath;
	if (m_mesh)
	{
		delete m_chunkyMesh;
		m_chunkyMesh = 0;
		delete m_mesh;
		m_mesh = 0;
	}
#ifndef MODIFY_OFF_MESH_CONNECTION
	m_offMeshConCount = 0;
#endif // !MODIFY_OFF_MESH_CONNECTION
	m_volumeCount = 0;
	
	m_mesh = new rcMeshLoaderObj;
	if (!m_mesh)
	{
		ctx->log(RC_LOG_ERROR, "loadMesh: Out of memory 'm_mesh'.");
		return false;
	}
#ifdef INTEGRATION_BUILD
	m_mesh->setIntegrationBuild( m_integrationBuild );
#endif // INTEGRATION_BUILD
	if (!m_mesh->load(filepath))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath);
		return false;
	}

#ifdef MODIFY_SQUARE_SECTOR
	rcCalcBounds( m_mesh->getVerts(), m_mesh->getVertCount(), m_mesh->getSquareMin(), m_mesh->getSquareMax(), m_meshBMin, m_meshBMax );
#else // MODIFY_SQUARE_SECTOR
	rcCalcBounds(m_mesh->getVerts(), m_mesh->getVertCount(), m_meshBMin, m_meshBMax);
#endif // MODIFY_SQUARE_SECTOR

	m_chunkyMesh = new rcChunkyTriMesh;
	if (!m_chunkyMesh)
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
		return false;
	}
	if (!rcCreateChunkyTriMesh(m_mesh->getVerts(), m_mesh->getTris(), m_mesh->getTriCount(), 256, m_chunkyMesh))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh.");
		return false;
	}		

	return true;
}

bool InputGeom::load(rcContext* ctx, const char* filePath)
{
	char* buf = 0;
	FILE* fp = fopen(filePath, "rb");
	if (!fp)
		return false;
	fseek(fp, 0, SEEK_END);
	int bufSize = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}
	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);
	if (readLen != 1)
	{
		return false;
	}

#ifndef MODIFY_OFF_MESH_CONNECTION
	m_offMeshConCount = 0;
#endif // !MODIFY_OFF_MESH_CONNECTION
	m_volumeCount = 0;
	delete m_mesh;
	m_mesh = 0;

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		if (row[0] == 'f')
		{
			// File name.
			const char* name = row+1;
			// Skip white spaces
			while (*name && isspace(*name))
				name++;
			if (*name)
			{
				if (!loadMesh(ctx, name))
				{
					delete [] buf;
					return false;
				}
			}
		}
		else if (row[0] == 'c')
		{
#ifndef MODIFY_OFF_MESH_CONNECTION
			// Off-mesh connection
			if (m_offMeshConCount < MAX_OFFMESH_CONNECTIONS)
			{
				float* v = &m_offMeshConVerts[m_offMeshConCount*3*2];
				int bidir, area = 0, flags = 0;
				float rad;
				sscanf(row+1, "%f %f %f  %f %f %f %f %d %d %d",
					   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rad, &bidir, &area, &flags);
				m_offMeshConRads[m_offMeshConCount] = rad;
				m_offMeshConDirs[m_offMeshConCount] = (unsigned char)bidir;
				m_offMeshConAreas[m_offMeshConCount] = (unsigned char)area;
				m_offMeshConFlags[m_offMeshConCount] = (unsigned short)flags;
				m_offMeshConCount++;
			}
#endif // !MODIFY_OFF_MESH_CONNECTION
		}
		else if (row[0] == 'v')
		{
			// Convex volumes
			if (m_volumeCount < MAX_VOLUMES)
			{
				ConvexVolume* vol = &m_volumes[m_volumeCount++];
				sscanf(row+1, "%d %d %f %f", &vol->nverts, &vol->area, &vol->hmin, &vol->hmax);
				for (int i = 0; i < vol->nverts; ++i)
				{
					row[0] = '\0';
					src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
					float x, y, z;
					sscanf(row, "%f %f %f", &x, &y, &z);
					vol->verts[i].SetX( x );
					vol->verts[i].SetY( y );
					vol->verts[i].SetZ( z );
				}
			}
		}
	}
	
	delete [] buf;
	
	return true;
}

bool InputGeom::save(const char* filepath)
{
	if (!m_mesh) return false;
	
	FILE* fp = fopen(filepath, "w");
	if (!fp) return false;
	
	// Store mesh filename.
	fprintf(fp, "f %s\n", m_mesh->getFileName());

#ifndef MODIFY_OFF_MESH_CONNECTION
	// Store off-mesh links.
	for (int i = 0; i < m_offMeshConCount; ++i)
	{
		const float* v = &m_offMeshConVerts[i*3*2];
		const float rad = m_offMeshConRads[i];
		const int bidir = m_offMeshConDirs[i];
		const int area = m_offMeshConAreas[i];
		const int flags = m_offMeshConFlags[i];
		fprintf(fp, "c %f %f %f  %f %f %f  %f %d %d %d\n",
				v[0], v[1], v[2], v[3], v[4], v[5], rad, bidir, area, flags);
	}
#endif // !MODIFY_OFF_MESH_CONNECTION

	// Convex volumes
	for (int i = 0; i < m_volumeCount; ++i)
	{
		ConvexVolume* vol = &m_volumes[i];
		fprintf(fp, "v %d %d %f %f\n", vol->nverts, vol->area, vol->hmin, vol->hmax);
		for (int j = 0; j < vol->nverts; ++j)
			fprintf(fp, "%f %f %f\n", vol->verts[j].X(), vol->verts[j].Y(), vol->verts[j].Z());
	}
	
	fclose(fp);
	
	return true;
}

static bool isectSegAABB(const dtCoordinates& _sp, const dtCoordinates& _sq,
						 const dtCoordinates& _amin, const dtCoordinates& _amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	const dtCoordinates _d( _sq.X() - _sp.X(), _sq.Y() - _sp.Y(), _sq.Z() - _sp.Z() );
	tmin = 0.0;
	tmax = 1.0f;

	for (int i = 0; i < 3; i++)
	{
		//////////////////////////////////////////////////////////////////////////
		float d, sp, sq, amin, amax;
		switch ( i ) {
			case 0:
				d = _d.X();	sp = _sp.X();	sq = _sq.X();	amin = _amin.X();	amax = _amax.X();
				break;
			case 1:
				d = _d.Y();	sp = _sp.Y();	sq = _sq.Y();	amin = _amin.Y();	amax = _amax.Y();
				break;
			case 2:
				d = _d.Z();	sp = _sp.Z();	sq = _sq.Z();	amin = _amin.Z();	amax = _amax.Z();
				break;
			default:
				return false;
		}
		//////////////////////////////////////////////////////////////////////////
		if (fabsf(d) < EPS)
		{
			if (sp < amin || sp > amax)
				return false;
		}
		else
		{
			const float ood = 1.0f / d;
			float t1 = (amin - sp) * ood;
			float t2 = (amax - sp) * ood;
			if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}


bool InputGeom::raycastMesh(const dtCoordinates& src, const dtCoordinates& dst, float& tmin)
{
	dtCoordinates dir;
	rcVsub(dir, dst, src);

	// Prune hit ray.
	float btmin, btmax;
	if (!isectSegAABB(src, dst, m_meshBMin, m_meshBMax, btmin, btmax))
		return false;
	float p[2], q[2];
	p[0] = src.X() + (dst.X()-src.X())*btmin;
	p[1] = src.Z() + (dst.Z()-src.Z())*btmin;
	q[0] = src.X() + (dst.X()-src.X())*btmax;
	q[1] = src.Z() + (dst.Z()-src.Z())*btmax;
	
	int cid[512];
	const int ncid = rcGetChunksOverlappingSegment(m_chunkyMesh, p, q, cid, 512);
	if (!ncid)
		return false;
	
	tmin = 1.0f;
	bool hit = false;
	const dtCoordinates* verts = m_mesh->getVerts();
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = m_chunkyMesh->nodes[cid[i]];
		const int* tris = &m_chunkyMesh->tris[node.i*3];
		const int ntris = node.n;

		for (int j = 0; j < ntris*3; j += 3)
		{
			float t = 1;
			if (intersectSegmentTriangle(src, dst,
										 verts[tris[j]],
										 verts[tris[j+1]],
										 verts[tris[j+2]], t))
			{
				if (t < tmin)
					tmin = t;
				hit = true;
			}
		}
	}
	
	return hit;
}

#ifndef MODIFY_OFF_MESH_CONNECTION
void InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad,
									 unsigned char bidir, unsigned char area, unsigned short flags)
{
	if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
	float* v = &m_offMeshConVerts[m_offMeshConCount*3*2];
	m_offMeshConRads[m_offMeshConCount] = rad;
	m_offMeshConDirs[m_offMeshConCount] = bidir;
	m_offMeshConAreas[m_offMeshConCount] = area;
	m_offMeshConFlags[m_offMeshConCount] = flags;
	m_offMeshConId[m_offMeshConCount] = 1000 + m_offMeshConCount;
	rcVcopy(&v[0], spos); 
	rcVcopy(&v[3], epos);
	m_offMeshConCount++;
}

void InputGeom::deleteOffMeshConnection(int i)
{
	m_offMeshConCount--;
	float* src = &m_offMeshConVerts[m_offMeshConCount*3*2];
	float* dst = &m_offMeshConVerts[i*3*2];
	rcVcopy(&dst[0], &src[0]);
	rcVcopy(&dst[3], &src[3]);
	m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
	m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
	m_offMeshConAreas[i] = m_offMeshConAreas[m_offMeshConCount];
	m_offMeshConFlags[i] = m_offMeshConFlags[m_offMeshConCount];
}

void InputGeom::drawOffMeshConnections(duDebugDraw* dd, bool hilight)
{
	unsigned int conColor = duRGBA(192,0,128,192);
	unsigned int baseColor = duRGBA(0,0,0,64);
	dd->depthMask(false);

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_offMeshConCount; ++i)
	{
		float* v = &m_offMeshConVerts[i*3*2];

		dd->vertex(v[0],v[1],v[2], baseColor);
		dd->vertex(v[0],v[1]+0.2f,v[2], baseColor);
		
		dd->vertex(v[3],v[4],v[5], baseColor);
		dd->vertex(v[3],v[4]+0.2f,v[5], baseColor);
		
		duAppendCircle(dd, v[0],v[1]+0.1f,v[2], m_offMeshConRads[i], baseColor);
		duAppendCircle(dd, v[3],v[4]+0.1f,v[5], m_offMeshConRads[i], baseColor);

		if (hilight)
		{
			duAppendArc(dd, v[0],v[1],v[2], v[3],v[4],v[5], 0.25f,
						(m_offMeshConDirs[i]&1) ? 0.6f : 0.0f, 0.6f, conColor);
		}
	}	
	dd->end();

	dd->depthMask(true);
}
#endif // !MODIFY_OFF_MESH_CONNECTION

void InputGeom::addConvexVolume(const dtCoordinates* verts, const int nverts,
								const float minh, const float maxh, unsigned char area)
{
	if (m_volumeCount >= MAX_VOLUMES) return;
	ConvexVolume* vol = &m_volumes[m_volumeCount++];
	memset(vol, 0, sizeof(ConvexVolume));
	memcpy(vol->verts, verts, sizeof(dtCoordinates)*nverts);
	vol->hmin = minh;
	vol->hmax = maxh;
	vol->nverts = nverts;
	vol->area = area;
}

void InputGeom::deleteConvexVolume(int i)
{
	m_volumeCount--;
	m_volumes[i] = m_volumes[m_volumeCount];
}

void InputGeom::drawConvexVolumes(struct duDebugDraw* dd, bool /*hilight*/)
{
	dd->depthMask(false);

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duIntToCol(vol->area, 32);
		for (int j = 0, k = vol->nverts-1; j < vol->nverts; k = j++)
		{
			const dtCoordinates va( vol->verts[k] );
			const dtCoordinates vb( vol->verts[j] );

			dd->vertex(vol->verts[0].X(),vol->hmax,vol->verts[0].Z(), col);
			dd->vertex(vb.X(),vol->hmax,vb.Z(), col);
			dd->vertex(va.X(),vol->hmax,va.Z(), col);
			
			dd->vertex(va.X(),vol->hmin,va.Z(), duDarkenCol(col));
			dd->vertex(va.X(),vol->hmax,va.Z(), col);
			dd->vertex(vb.X(),vol->hmax,vb.Z(), col);

			dd->vertex(va.X(),vol->hmin,va.Z(), duDarkenCol(col));
			dd->vertex(vb.X(),vol->hmax,vb.Z(), col);
			dd->vertex(vb.X(),vol->hmin,vb.Z(), duDarkenCol(col));
		}
	}
	
	dd->end();

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duIntToCol(vol->area, 220);
		for (int j = 0, k = vol->nverts-1; j < vol->nverts; k = j++)
		{
			const dtCoordinates va( vol->verts[k] );
			const dtCoordinates vb( vol->verts[j] );
			dd->vertex(va.X(),vol->hmin,va.Z(), duDarkenCol(col));
			dd->vertex(vb.X(),vol->hmin,vb.Z(), duDarkenCol(col));
			dd->vertex(va.X(),vol->hmax,va.Z(), col);
			dd->vertex(vb.X(),vol->hmax,vb.Z(), col);
			dd->vertex(va.X(),vol->hmin,va.Z(), duDarkenCol(col));
			dd->vertex(va.X(),vol->hmax,va.Z(), col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duDarkenCol(duIntToCol(vol->area, 255));
		for (int j = 0; j < vol->nverts; ++j)
		{
			dd->vertex(vol->verts[j].X(),vol->verts[j].Y()+0.1f,vol->verts[j].Z(), col);
			dd->vertex(vol->verts[j].X(),vol->hmin,vol->verts[j].Z(), col);
			dd->vertex(vol->verts[j].Z(),vol->hmax,vol->verts[j].Z(), col);
		}
	}
	dd->end();
	
	
	dd->depthMask(true);
}
