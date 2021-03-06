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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "Sample.h"
#include <string>
#include <vector>
#include "..\..\Detour\Include\DetourNavMesh.h"
#include "..\..\Detour\Include\DetourCoordinates.h"

static const int MAX_CONVEXVOL_PTS = 12;
struct ConvexVolume
{
	dtCoordinates verts[MAX_CONVEXVOL_PTS];
	float hmin, hmax;
	int nverts;
	int area;
};


class InputGeom
{
	rcChunkyTriMesh* m_chunkyMesh;
	rcMeshLoaderObj* m_mesh;
	dtCoordinates m_meshBMin, m_meshBMax;
	
#ifndef MODIFY_OFF_MESH_CONNECTION
	/// @name Off-Mesh connections.
	///@{
	static const int MAX_OFFMESH_CONNECTIONS = 10000;
	dtCoordinates m_offMeshConVerts[MAX_OFFMESH_CONNECTIONS*2];
	float m_offMeshConRads[MAX_OFFMESH_CONNECTIONS];
	unsigned char m_offMeshConDirs[MAX_OFFMESH_CONNECTIONS];
	unsigned char m_offMeshConAreas[MAX_OFFMESH_CONNECTIONS];
	unsigned short m_offMeshConFlags[MAX_OFFMESH_CONNECTIONS];
	unsigned int m_offMeshConId[MAX_OFFMESH_CONNECTIONS];
	int m_offMeshConCount;
	///@}
#endif // !MODIFY_OFF_MESH_CONNECTION

	/// @name Convex Volumes.
	///@{
	static const int MAX_VOLUMES = 256;
	ConvexVolume m_volumes[MAX_VOLUMES];
	int m_volumeCount;
#ifdef INTEGRATION_BUILD
	bool m_integrationBuild;
#endif // INTEGRATION_BUILD
#ifdef VARIABLE_TILE_SIZE
	bool m_variableTileSize;
#endif // VARIABLE_TILE_SIZE
	///@}
	
public:
	std::string m_path;
	InputGeom();
	~InputGeom();
	
	bool loadMesh(class rcContext* ctx, const char* filepath);
	
	bool load(class rcContext* ctx, const char* filepath);
	bool save(const char* filepath);
	
	/// Method to return static mesh data.
	inline const rcMeshLoaderObj* getMesh() const { return m_mesh; }
	inline const dtCoordinates& getMeshBoundsMin() const { return m_meshBMin; }
	inline const dtCoordinates& getMeshBoundsMax() const { return m_meshBMax; }
	inline const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }
	bool raycastMesh(const dtCoordinates& src, const dtCoordinates& dst, float& tmin);

#ifndef MODIFY_OFF_MESH_CONNECTION
	/// @name Off-Mesh connections.
	///@{
	int getOffMeshConnectionCount() const { return m_offMeshConCount; }
	const float* getOffMeshConnectionVerts() const { return m_offMeshConVerts; }
	const float* getOffMeshConnectionRads() const { return m_offMeshConRads; }
	const unsigned char* getOffMeshConnectionDirs() const { return m_offMeshConDirs; }
	const unsigned char* getOffMeshConnectionAreas() const { return m_offMeshConAreas; }
	const unsigned short* getOffMeshConnectionFlags() const { return m_offMeshConFlags; }
	const unsigned int* getOffMeshConnectionId() const { return m_offMeshConId; }
	void addOffMeshConnection(const float* spos, const float* epos, const float rad,
							  unsigned char bidir, unsigned char area = SAMPLE_POLYAREA_JUMP, unsigned short flags = SAMPLE_POLYFLAGS_JUMP);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);
	///@}
#endif // !MODIFY_OFF_MESH_CONNECTION

	/// @name Box Volumes.
	///@{
	int getConvexVolumeCount() const { return m_volumeCount; }
	const ConvexVolume* getConvexVolumes() const { return m_volumes; }
	void addConvexVolume(const dtCoordinates* verts, const int nverts,
						 const float minh, const float maxh, unsigned char area);
	void deleteConvexVolume(int i);
	void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);
	///@}

#ifdef INTEGRATION_BUILD
	void	setIntegrationBuild( const bool integrationBuild )	{	m_integrationBuild = integrationBuild;	}
#endif // INTEGRATION_BUILD

#ifdef VARIABLE_TILE_SIZE
	void	setVariableTileSize( const bool variableTileSize )	{	m_variableTileSize = variableTileSize;	}
	bool	isVariableTileSize() const	{	return m_variableTileSize;	}
#endif // VARIABLE_TILE_SIZE

// #ifdef MODIFY_OFF_MESH_CONNECTION
	std::vector<dtJumpMeshConnection>	tableJumpMeshConnection;
	int jumpMeshConnectionCount;
// #endif // MODIFY_OFF_MESH_CONNECTION
};

#endif // INPUTGEOM_H
