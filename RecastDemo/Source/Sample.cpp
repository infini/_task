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
#include "Sample.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"
#include "DetourCommon.h"
#include "DetourCoordinates.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

Sample::Sample() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS|DU_DRAWNAVMESH_CLOSEDLIST),
	m_tool(0),
	m_ctx(0)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();

	for (int i = 0; i < MAX_TOOLS; i++)
		m_toolStates[i] = 0;
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	dtFreeCrowd(m_crowd);
	delete m_tool;
	for (int i = 0; i < MAX_TOOLS; i++)
		delete m_toolStates[i];
}

void Sample::setTool(SampleTool* tool)
{
	delete m_tool;
	m_tool = tool;
	if (tool)
		m_tool->init(this);
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender()
{
	if (!m_geom)
		return;
	
	DebugDrawGL dd;
		
	// Draw mesh
	duDebugDrawTriMesh(&dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
					   m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0, 1.0f);
	// Draw bounds
	const dtCoordinates bmin( m_geom->getMeshBoundsMin() );
	const dtCoordinates bmax( m_geom->getMeshBoundsMax() );
	duDebugDrawBoxWire(&dd, bmin.X(),bmin.Y(),bmin.Z(), bmax.X(),bmax.Y(),bmax.Z(), duRGBA(255,255,255,128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const dtCoordinates* Sample::getBoundsMin()
{
	if (!m_geom) return 0;
	return &m_geom->getMeshBoundsMin();
}

const dtCoordinates* Sample::getBoundsMax()
{
	if (!m_geom) return 0;
	return &m_geom->getMeshBoundsMax();
}

void Sample::resetCommonSettings()
{
	///*
	m_cellSize				= 0.01f;
	m_cellHeight			= 0.01f;
	m_agentHeight			= 0.4f;
	m_agentRadius			= 0.0f;
	m_agentMaxClimb			= 0.1f;
	m_agentMaxSlope			= 70.0f;
	m_regionMinSize			= 10.f;
	m_regionMergeSize		= 20.f;
	m_monotonePartitioning	= false;
	m_edgeMaxLen			= 0.0f;
	m_edgeMaxError			= 1.3f;
	m_vertsPerPoly			= 4.0f;
	m_detailSampleDist		= 20.f;
	m_detailSampleMaxError	= 1.0f;
	//*/

	/*
	m_cellSize				= 0.02f;
	m_cellHeight			= 0.01f;
	m_agentHeight			= 0.4f;
	m_agentRadius			= 0.0f;
	m_agentMaxClimb			= 0.1f;
	m_agentMaxSlope			= 70.0f;
	m_regionMinSize			= 4.0f;
	m_regionMergeSize		= 10.0f;
	m_monotonePartitioning	= false;
	m_edgeMaxLen			= 0.0f;
	m_edgeMaxError			= 1.3f;
	m_vertsPerPoly			= 4.0f;
	m_detailSampleDist		= 10.0f;
	m_detailSampleMaxError	= 1.0f;
	*/
}

void Sample::handleCommonSettings()
{
	imguiLabel("Rasterization");
	imguiSlider("Cell Size", &m_cellSize, 0.01f, 1.0f, 0.01f);
	imguiSlider("Cell Height", &m_cellHeight, 0.01f, 1.0f, 0.01f);
	
	if (m_geom)
	{
		const dtCoordinates bmin( m_geom->getMeshBoundsMin() );
		const dtCoordinates bmax( m_geom->getMeshBoundsMax() );
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		imguiValue(text);
	}
	
	imguiSeparator();
	imguiLabel("Agent");
	imguiSlider("Height", &m_agentHeight, 0.0f, 2.0f, 0.01f);
	imguiSlider("Radius", &m_agentRadius, 0.0f, 2.0f, 0.01f);
	imguiSlider("Max Climb", &m_agentMaxClimb, 0.f, 1.0f, 0.01f);
	imguiSlider("Max Slope", &m_agentMaxSlope, 0.0f, 180.0f, 1.0f);
	
	imguiSeparator();
	imguiLabel("Region");
	imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 400.0f, 2.0f);
	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 400.0f, 2.0f);
	if (imguiCheck("Monotone Partitioning", m_monotonePartitioning))
		m_monotonePartitioning = !m_monotonePartitioning;
	
	imguiSeparator();
	imguiLabel("Polygonization");
	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 100.0f, 1.0f);
	imguiSlider("Max Edge Error", &m_edgeMaxError, 0.0f, 10.0f, 0.1f);
	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 6.0f, 1.0f);

	imguiSeparator();
	imguiLabel("Detail Mesh");
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.0f, 30.0f, 1.0f);
	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 30.0f, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const dtCoordinates& s, const dtCoordinates& p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void Sample::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}


void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

#ifdef MODIFY_OFF_MESH_CONNECTION
#include <vector>
void	Sample::rcGenerateJumpableMeshConnection( const rcPolyMesh& mesh, const float walkableHeight, const float walkableClimb, InputGeom& geom )
{
	//////////////////////////////////////////////////////////////////////////
	geom.tableJumpMeshConnection.clear();
	geom.tableJumpMeshConnection.reserve( mesh.npolys*(mesh.npolys-1) );
	geom.jumpMeshConnectionCount = 0;

	struct MeshPosition
	{
		int vertCount;
		dtCoordinates pos;
		dtCoordinates bmin;
		dtCoordinates bmax;
		dtCoordinates verts[DT_VERTS_PER_POLYGON];
		MeshPosition() : vertCount( 0 ) {}
	};

	std::vector<MeshPosition>	tableMeshPosition;
	tableMeshPosition.reserve( mesh.npolys );

	for( int nth = 0; nth < mesh.npolys; ++nth ) {
		//////////////////////////////////////////////////////////////////////////
		unsigned short* p = &mesh.polys[nth*2*DT_VERTS_PER_POLYGON];
		MeshPosition meshPosition;
		for( int i = 0; i < DT_VERTS_PER_POLYGON; ++i ) {
			if( p[i] == RC_MESH_NULL_IDX ) {
				break;
			}
			++meshPosition.vertCount;
		}
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		const unsigned short* va = &mesh.verts[p[0]*3];
		meshPosition.bmin = dtCoordinates( mesh.bmin.X() + (va[0] * mesh.cs), mesh.bmin.Y() + (va[1] * mesh.ch), mesh.bmin.Z() + (va[2] * mesh.cs) );
		meshPosition.bmax = dtCoordinates( mesh.bmin.X() + (va[0] * mesh.cs), mesh.bmin.Y() + (va[1] * mesh.ch), mesh.bmin.Z() + (va[2] * mesh.cs) );
		meshPosition.verts[0] = dtCoordinates( mesh.bmin.X() + (va[0] * mesh.cs), mesh.bmin.Y() + (va[1] * mesh.ch), mesh.bmin.Z() + (va[2] * mesh.cs) );

		for( int i = 1; i < meshPosition.vertCount; ++i ) {
			va = &mesh.verts[p[i]*3];
			meshPosition.verts[i] = dtCoordinates( mesh.bmin.X() + (va[0] * mesh.cs), mesh.bmin.Y() + (va[1] * mesh.ch), mesh.bmin.Z() + (va[2] * mesh.cs) );

			meshPosition.bmin.SetX( rcMin( meshPosition.bmin.X(), meshPosition.verts[i].X() ) );
			meshPosition.bmin.SetY( rcMin( meshPosition.bmin.Y(), meshPosition.verts[i].Y() ) );
			meshPosition.bmin.SetZ( rcMin( meshPosition.bmin.Z(), meshPosition.verts[i].Z() ) );

			meshPosition.bmax.SetX( rcMax( meshPosition.bmax.X(), meshPosition.verts[i].X() ) );
			meshPosition.bmax.SetY( rcMax( meshPosition.bmax.Y(), meshPosition.verts[i].Y() ) );
			meshPosition.bmax.SetZ( rcMax( meshPosition.bmax.Z(), meshPosition.verts[i].Z() ) );
		}
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		dtCoordinates center;
		for( int i = 0; i < meshPosition.vertCount; ++i ) {
			center.SetX( center.X() + meshPosition.verts[i].X() );
			center.SetY( center.Y() + meshPosition.verts[i].Y() );
			center.SetZ( center.Z() + meshPosition.verts[i].Z() );
		}
		const float divide = 1.0f / meshPosition.vertCount;
		meshPosition.pos.SetX( center.X() * divide );
		meshPosition.pos.SetY( center.Y() * divide );
		meshPosition.pos.SetZ( center.Z() * divide );
		
		//tableMeshPosition.at( nth ) = meshPosition;
		tableMeshPosition.push_back( meshPosition );
		//////////////////////////////////////////////////////////////////////////
	}

	//////////////////////////////////////////////////////////////////////////
	for( unsigned int nth = 0; nth < tableMeshPosition.size(); ++nth ) {
		const MeshPosition src = tableMeshPosition.at( nth );
		for( unsigned i = 0; i < tableMeshPosition.size(); ++i ) {
			if( i == nth ) {
				continue;
			}
			const MeshPosition dest = tableMeshPosition.at( i );

			//////////////////////////////////////////////////////////////////////////
			// over height?
			if( rcAbs( src.pos.Y() - dest.pos.Y() ) <= walkableClimb || ( src.pos.Y() < dest.pos.Y() && walkableHeight < rcAbs( dest.pos.Y() - src.pos.Y() ) ) ) {
				continue;
			}
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			// in bound?
// 			if( dtCompleteOverlapBounds2D( src.bmin, src.bmax, dest.bmin, dest.bmax ) ) {
// 				continue;
// 			}

			bool overlap = false;
			for( int nv = 0; nv < src.vertCount; ++nv ) {
				if( rcIsOverlapBounds2D( src.verts[nv], dest.bmin, dest.bmax ) ) {
					overlap = true;
					break;
				}
			}
			if( !overlap ) {
				continue;
			}
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			// jump link?
			const bool jumpLink = rcIsLinkableMeshFlag( mesh.flags[nth], mesh.flags[i] );
			if( !jumpLink ) {
				continue;
			}
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			// is connected?
			const bool connect = rcIsConnectedPoly( src.verts, src.vertCount, dest.verts, dest.vertCount );
			if( connect ) {
				continue;
			}
			//////////////////////////////////////////////////////////////////////////
			
			dtJumpMeshConnection jp;
			jp.startPosition = src.pos;
			jp.endPosition = dest.pos;

			geom.tableJumpMeshConnection.push_back( jp );
			++geom.jumpMeshConnectionCount;
		}
	}
	//////////////////////////////////////////////////////////////////////////
}
#endif // MODIFY_OFF_MESH_CONNECTION
