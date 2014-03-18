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
	const float* bmin = m_geom->getMeshBoundsMin();
	const float* bmax = m_geom->getMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const float* Sample::getBoundsMin()
{
	if (!m_geom) return 0;
	return m_geom->getMeshBoundsMin();
}

const float* Sample::getBoundsMax()
{
	if (!m_geom) return 0;
	return m_geom->getMeshBoundsMax();
}

void Sample::resetCommonSettings()
{
	m_cellSize				= 0.02f;
	m_cellHeight			= 0.01f;
	m_agentHeight			= 0.4f;
	m_agentRadius			= 0.0f;
	m_agentMaxClimb			= 0.1f;
	m_agentMaxSlope			= 60.0f;
	m_regionMinSize			= 4.0f;//4.0f;
	m_regionMergeSize		= 100.0f;//20.0f;
	m_monotonePartitioning	= false;
	m_edgeMaxLen			= 0.0f;
	m_edgeMaxError			= 1.3f;
	m_vertsPerPoly			= 6.0f;
	m_detailSampleDist		= 8.0f;
	m_detailSampleMaxError	= 1.0f;
}

void Sample::handleCommonSettings()
{
	imguiLabel("Rasterization");
	imguiSlider("Cell Size", &m_cellSize, 0.01f, 1.0f, 0.01f);
	imguiSlider("Cell Height", &m_cellHeight, 0.01f, 1.0f, 0.01f);
	
	if (m_geom)
	{
		const float* bmin = m_geom->getMeshBoundsMin();
		const float* bmax = m_geom->getMeshBoundsMax();
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
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.0f, 16.0f, 1.0f);
	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 16.0f, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
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
void	Sample::rcGenerateOffMeshConnection( const rcPolyMesh& mesh, InputGeom& geom )
{
	//////////////////////////////////////////////////////////////////////////
	// temporary
	struct _pos
	{
		float x;
		float y;
		float z;
		_pos() : x( 0 ), y( 0 ), z( 0 )	{}
	};

	std::vector<_pos>	tablePos;
	//////////////////////////////////////////////////////////////////////////
	for( int nth = 0; nth < mesh.npolys; ++nth ) {
		unsigned short* p = &mesh.polys[nth*2*DT_VERTS_PER_POLYGON];
		int vertCount = 0;
		for( int i = 0; i < DT_VERTS_PER_POLYGON; ++i ) {
			if( p[i] == RC_MESH_NULL_IDX ) {
				break;
			}
			++vertCount;
		}

		float verts[DT_VERTS_PER_POLYGON*3], bmin[3], bmax[3];
		const unsigned short* va = &mesh.verts[p[0]*3];

		bmin[0] = bmax[0] = verts[0] = mesh.bmin[0] + (va[0] * mesh.cs);
		bmin[1] = bmax[1] = verts[1] = mesh.bmin[1] + (va[1] * mesh.ch);
		bmin[2] = bmax[2] = verts[2] = mesh.bmin[2] + (va[2] * mesh.cs);

		for( int i = 1; i < vertCount; ++i ) {
			va = &mesh.verts[p[i]*3];
			verts[i*3+0] = mesh.bmin[0] + (va[0] * mesh.cs);
			verts[i*3+1] = mesh.bmin[1] + (va[1] * mesh.ch);
			verts[i*3+2] = mesh.bmin[2] + (va[2] * mesh.cs);

			bmin[0] = rcMin( bmin[0], verts[i*3+0] );
			bmin[1] = rcMin( bmin[1], verts[i*3+1] );
			bmin[2] = rcMin( bmin[2], verts[i*3+2] );

			bmax[0] = rcMax( bmax[0], verts[i*3+0] );
			bmax[1] = rcMax( bmax[1], verts[i*3+1] );
			bmax[2] = rcMax( bmax[2], verts[i*3+2] );
		}

		//////////////////////////////////////////////////////////////////////////
		const float dist = rcSqrt( (bmax[0] - bmin[0]) + (bmax[2] - bmin[2]) );
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		float center[3] = { 0 };
		for( int i = 0; i < vertCount; ++i ) {
			center[0] += verts[i*3+0];
			center[1] += verts[i*3+1];
			center[2] += verts[i*3+2];
		}
		const float divide = 1.0f / vertCount;
		_pos pos;
		pos.x = center[0] * divide;
		pos.y = center[1] * divide;
		pos.z = center[2] * divide;
		//////////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////////////////
		// temporary
		tablePos.push_back( pos );
		//////////////////////////////////////////////////////////////////////////
	}

	//////////////////////////////////////////////////////////////////////////
	// temporary
	for( unsigned int nth = 0; nth < tablePos.size(); ++nth ) {
		const _pos src = tablePos.at( nth );
		for( unsigned i = 0; i < tablePos.size(); ++i ) {
			if( i == nth ) {
				continue;
			}
			const _pos dest = tablePos.at( i );
			if( 0.1f < rcAbs( src.y - dest.y ) ) {
				// check distance
				//////////////////////////////////////////////////////////////////////////
				float _src[3], _dest[3];
				_src[0] = src.x;
				_src[1] = src.y;
				_src[2] = src.z;

				_dest[0] = dest.x;
				_dest[1] = dest.y;
				_dest[2] = dest.z;
				//////////////////////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////////////////////
				geom.addOffMeshConnection( _src, _dest, 0.01f, 0 );
				//////////////////////////////////////////////////////////////////////////
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////
}
#endif // MODIFY_OFF_MESH_CONNECTION
