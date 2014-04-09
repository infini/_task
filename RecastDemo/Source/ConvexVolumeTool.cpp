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
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "ConvexVolumeTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Quick and dirty convex hull.

// Returns true if 'c' is left of line 'a'-'b'.
inline bool left(const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c)
{ 
	const float u1 = b.X() - a.X();
	const float v1 = b.Z() - a.Z();
	const float u2 = c.X() - a.X();
	const float v2 = c.Z() - a.Z();
	return u1 * v2 - v1 * u2 < 0;
}

// Returns true if 'a' is more lower-left than 'b'.
inline bool cmppt(const dtCoordinates& a, const dtCoordinates& b)
{
	if (a.X() < b.X()) return true;
	if (a.X() > b.X()) return false;
	if (a.Z() < b.Z()) return true;
	if (a.Z() > b.Z()) return false;
	return false;
}
// Calculates convex hull on xz-plane of points on 'pts',
// stores the indices of the resulting hull in 'out' and
// returns number of points on hull.
static int convexhull(const dtCoordinates* pts, int npts, int* out)
{
	// Find lower-leftmost point.
	int hull = 0;
	for (int i = 1; i < npts; ++i)
		if (cmppt(pts[i], pts[hull]))
			hull = i;
	// Gift wrap hull.
	int endpt = 0;
	int i = 0;
	do
	{
		out[i++] = hull;
		endpt = 0;
		for (int j = 1; j < npts; ++j)
			if (hull == endpt || left(pts[hull], pts[endpt], pts[j]))
				endpt = j;
		hull = endpt;
	}
	while (endpt != out[0]);
	
	return i;
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


ConvexVolumeTool::ConvexVolumeTool() :
	m_sample(0),
	m_areaType(SAMPLE_POLYAREA_GRASS),
	m_polyOffset(0.0f),
	m_boxHeight(6.0f),
	m_boxDescent(1.0f),
	m_npts(0),
	m_nhull(0)
{
}

ConvexVolumeTool::~ConvexVolumeTool()
{
}

void ConvexVolumeTool::init(Sample* sample)
{
	m_sample = sample;
}

void ConvexVolumeTool::reset()
{
	m_npts = 0;
	m_nhull = 0;
}

void ConvexVolumeTool::handleMenu()
{
	imguiSlider("Shape Height", &m_boxHeight, 0.1f, 20.0f, 0.1f);
	imguiSlider("Shape Descent", &m_boxDescent, 0.1f, 20.0f, 0.1f);
	imguiSlider("Poly Offset", &m_polyOffset, 0.0f, 10.0f, 0.1f);

	imguiSeparator();

	imguiLabel("Area Type");
	imguiIndent();
	if (imguiCheck("Grass", m_areaType == SAMPLE_POLYAREA_GRASS))
		m_areaType = SAMPLE_POLYAREA_GRASS;
	if (imguiCheck("Road", m_areaType == SAMPLE_POLYAREA_ROAD))
		m_areaType = SAMPLE_POLYAREA_ROAD;
	if (imguiCheck("Water", m_areaType == SAMPLE_POLYAREA_WATER))
		m_areaType = SAMPLE_POLYAREA_WATER;
	if (imguiCheck("Door", m_areaType == SAMPLE_POLYAREA_DOOR))
		m_areaType = SAMPLE_POLYAREA_DOOR;
	imguiUnindent();

	imguiSeparator();

	if (imguiButton("Clear Shape"))
	{
		m_npts = 0;
		m_nhull = 0;
	}
}

void ConvexVolumeTool::handleClick(const dtCoordinates& /*s*/, const dtCoordinates& p, bool shift)
{
	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	
	if (shift)
	{
		// Delete
		int nearestIndex = -1;
		const ConvexVolume* vols = geom->getConvexVolumes();
		for (int i = 0; i < geom->getConvexVolumeCount(); ++i)
		{
			if (pointInPoly(vols[i].nverts, vols[i].verts, p) &&
							p.Y() >= vols[i].hmin && p.Y() <= vols[i].hmax)
			{
				nearestIndex = i;
			}
		}
		// If end point close enough, delete it.
		if (nearestIndex != -1)
		{
			geom->deleteConvexVolume(nearestIndex);
		}
	}
	else
	{
		// Create

		// If clicked on that last pt, create the shape.
		if (m_npts && rcVdistSqr(p, m_pts[(m_npts-1)]) < rcSqr(0.2f))
		{
			if (m_nhull > 2)
			{
				// Create shape.
				dtCoordinates verts[MAX_PTS];
				for (int i = 0; i < m_nhull; ++i)
					rcVcopy(verts[i], m_pts[m_hull[i]]);
					
				float minh = FLT_MAX, maxh = 0;
				for (int i = 0; i < m_nhull; ++i)
					minh = rcMin(minh, verts[i].Y());
				minh -= m_boxDescent;
				maxh = minh + m_boxHeight;

				if (m_polyOffset > 0.01f)
				{
					dtCoordinates offset[MAX_PTS*2];
					int noffset = rcOffsetPoly(verts, m_nhull, m_polyOffset, offset, MAX_PTS*2);
					if (noffset > 0)
						geom->addConvexVolume(offset, noffset, minh, maxh, (unsigned char)m_areaType);
				}
				else
				{
					geom->addConvexVolume(verts, m_nhull, minh, maxh, (unsigned char)m_areaType);
				}
			}
			
			m_npts = 0;
			m_nhull = 0;
		}
		else
		{
			// Add new point 
			if (m_npts < MAX_PTS)
			{
				rcVcopy(m_pts[m_npts], p);
				m_npts++;
				// Update hull.
				if (m_npts > 1)
					m_nhull = convexhull(m_pts, m_npts, m_hull);
				else
					m_nhull = 0;
			}
		}		
	}
	
}

void ConvexVolumeTool::handleToggle()
{
}

void ConvexVolumeTool::handleStep()
{
}

void ConvexVolumeTool::handleUpdate(const float /*dt*/)
{
}

void ConvexVolumeTool::handleRender()
{
	DebugDrawGL dd;
	
	// Find height extents of the shape.
	float minh = FLT_MAX, maxh = 0;
	for (int i = 0; i < m_npts; ++i)
		minh = rcMin(minh, m_pts[i].Y());
	minh -= m_boxDescent;
	maxh = minh + m_boxHeight;

	dd.begin(DU_DRAW_POINTS, 4.0f);
	for (int i = 0; i < m_npts; ++i)
	{
		unsigned int col = duRGBA(255,255,255,255);
		if (i == m_npts-1)
			col = duRGBA(240,32,16,255);
		dd.vertex(m_pts[i].X(),m_pts[i].Y()+0.1f,m_pts[i].Z(), col);
	}
	dd.end();

	dd.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0, j = m_nhull-1; i < m_nhull; j = i++)
	{
		const dtCoordinates vi( m_pts[m_hull[j]] );
		const dtCoordinates vj( m_pts[m_hull[i]] );
		dd.vertex(vj.X(),minh,vj.Z(), duRGBA(255,255,255,64));
		dd.vertex(vi.X(),minh,vi.Z(), duRGBA(255,255,255,64));
		dd.vertex(vj.X(),maxh,vj.Z(), duRGBA(255,255,255,64));
		dd.vertex(vi.X(),maxh,vi.Z(), duRGBA(255,255,255,64));
		dd.vertex(vj.X(),minh,vj.Z(), duRGBA(255,255,255,64));
		dd.vertex(vj.X(),maxh,vj.Z(), duRGBA(255,255,255,64));
	}
	dd.end();	
}

void ConvexVolumeTool::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* view)
{
	// Tool help
	const int h = view[3];
	if (!m_npts)
	{
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Create new shape.  SHIFT+LMB: Delete existing shape (click inside a shape).", imguiRGBA(255,255,255,192));	
	}
	else
	{
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "Click LMB to add new points. Click on the red point to finish the shape.", imguiRGBA(255,255,255,192));	
		imguiDrawText(280, h-60, IMGUI_ALIGN_LEFT, "The shape will be convex hull of all added points.", imguiRGBA(255,255,255,192));	
	}
	
}
