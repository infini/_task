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

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include "TestCase.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "PerfTimer.h"

#ifdef WIN32
#define snprintf _snprintf
#endif

TestCase::TestCase() :
	m_tests(0)
{
}

TestCase::~TestCase()
{
	Test* iter = m_tests;
	while (iter)
	{
		Test* next = iter->next;
		delete iter;
		iter = next;
	}
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

static void copyName(char* dst, const char* src)
{
	// Skip white spaces
	while (*src && isspace(*src))
		src++;
	strcpy(dst, src);
}

bool TestCase::load(const char* filePath)
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

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512];
	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		if (row[0] == 's')
		{
			// Sample name.
			copyName(m_sampleName, row+1);
		}
		else if (row[0] == 'f')
		{
			// File name.
			copyName(m_geomFileName, row+1);
		}
		else if (row[0] == 'p' && row[1] == 'f')
		{
			// Pathfind test.
			Test* test = new Test;
			memset(test, 0, sizeof(Test));
			test->type = TEST_PATHFIND;
			test->expand = false;
			test->next = m_tests;
			m_tests = test;
			float spos[3], epos[3];
			sscanf(row+2, "%f %f %f %f %f %f %x %x",
				   &spos[0], &spos[1], &spos[2],
				   &epos[0], &epos[1], &epos[2],
				   &test->includeFlags, &test->excludeFlags);

			test->spos = spos;
			test->epos = epos;
		}
		else if (row[0] == 'r' && row[1] == 'c')
		{
			// Pathfind test.
			Test* test = new Test;
			memset(test, 0, sizeof(Test));
			test->type = TEST_RAYCAST;
			test->expand = false;
			test->next = m_tests;
			m_tests = test;
			float spos[3], epos[3];
			sscanf(row+2, "%f %f %f %f %f %f %x %x",
				   &spos[0], &spos[1], &spos[2],
				   &epos[0], &epos[1], &epos[2],
				   &test->includeFlags, &test->excludeFlags);

			test->spos = spos;
			test->epos = epos;
		}
	}
	
	delete [] buf;

	return true;
}
		
void TestCase::resetTimes()
{
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		iter->findNearestPolyTime = 0;
		iter->findPathTime = 0;
		iter->findStraightPathTime = 0;
	}
}

void TestCase::doTests(dtNavMesh* navmesh, dtNavMeshQuery* navquery)
{
	if (!navmesh || !navquery)
		return;
	
	resetTimes();
	
	static const int MAX_POLYS = 256;
	dtPolyRef polys[MAX_POLYS];
	dtCoordinates straight[MAX_POLYS];
	const dtCoordinates polyPickExt( 2.f,4.f,2.f );
	
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		delete [] iter->polys;
		iter->polys = 0;
		iter->npolys = 0;
		delete [] iter->straight;
		iter->straight = 0;
		iter->nstraight = 0;
		
		dtQueryFilter filter;
		filter.setIncludeFlags((unsigned short)iter->includeFlags);
		filter.setExcludeFlags((unsigned short)iter->excludeFlags);
	
		// Find start points
		TimeVal findNearestPolyStart = getPerfTime();
		
		dtPolyRef startRef, endRef;
		navquery->findNearestPoly(iter->spos, polyPickExt, &filter, &startRef, &iter->nspos);
		navquery->findNearestPoly(iter->epos, polyPickExt, &filter, &endRef, &iter->nepos);

		TimeVal findNearestPolyEnd = getPerfTime();
		iter->findNearestPolyTime += getPerfDeltaTimeUsec(findNearestPolyStart, findNearestPolyEnd);

		if (!startRef || ! endRef)
			continue;
	
		if (iter->type == TEST_PATHFIND)
		{
		// Find path
		TimeVal findPathStart = getPerfTime();

		navquery->findPath(startRef, endRef, iter->spos, iter->epos, &filter, polys, &iter->npolys, MAX_POLYS);
		
		TimeVal findPathEnd = getPerfTime();
		iter->findPathTime += getPerfDeltaTimeUsec(findPathStart, findPathEnd);
		
		// Find straight path
		if (iter->npolys)
		{
			TimeVal findStraightPathStart = getPerfTime();
			
			navquery->findStraightPath(iter->spos, iter->epos, polys, iter->npolys,
									   straight, 0, 0, &iter->nstraight, MAX_POLYS);
			TimeVal findStraightPathEnd = getPerfTime();
			iter->findStraightPathTime += getPerfDeltaTimeUsec(findStraightPathStart, findStraightPathEnd);
		}
		
		// Copy results
		if (iter->npolys)
		{
			iter->polys = new dtPolyRef[iter->npolys];
			memcpy(iter->polys, polys, sizeof(dtPolyRef)*iter->npolys);
		}
		if (iter->nstraight)
		{
			iter->straight = new dtCoordinates[iter->nstraight];
			memcpy(iter->straight, straight, sizeof(dtCoordinates)*iter->nstraight);
		}
	}
		else if (iter->type == TEST_RAYCAST)
		{
			float t = 0;
			dtCoordinates hitNormal, hitPos;
			
			iter->straight = new dtCoordinates[2];
			iter->nstraight = 2;
			
			iter->straight[0].SetX( iter->spos.X() );
			iter->straight[0].SetY( iter->spos.Y() );
			iter->straight[0].SetZ( iter->spos.X() );
			
			TimeVal findPathStart = getPerfTime();
			
			navquery->raycast(startRef, iter->spos, iter->epos, &filter, &t, hitNormal, polys, &iter->npolys, MAX_POLYS);

			TimeVal findPathEnd = getPerfTime();
			iter->findPathTime += getPerfDeltaTimeUsec(findPathStart, findPathEnd);

			if (t > 1)
			{
				// No hit
				dtVcopy(hitPos, iter->epos);
			}
			else
			{
				// Hit
				dtVlerp(hitPos, iter->spos, iter->epos, t);
			}
			// Adjust height.
			if (iter->npolys > 0)
			{
				float h = 0;
				navquery->getPolyHeight(polys[iter->npolys-1], hitPos, &h);
				hitPos.SetY( h );
			}
			dtVcopy(iter->straight[1], hitPos);

			if (iter->npolys)
			{
				iter->polys = new dtPolyRef[iter->npolys];
				memcpy(iter->polys, polys, sizeof(dtPolyRef)*iter->npolys);
			}
		}
	}


	printf("Test Results:\n");
	int n = 0;
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		const int total = iter->findNearestPolyTime + iter->findPathTime + iter->findStraightPathTime;
		printf(" - Path %02d:     %.4f ms\n", n, (float)total/1000.0f);
		printf("    - poly:     %.4f ms\n", (float)iter->findNearestPolyTime/1000.0f);
		printf("    - path:     %.4f ms\n", (float)iter->findPathTime/1000.0f);
		printf("    - straight: %.4f ms\n", (float)iter->findStraightPathTime/1000.0f);
		n++;
	}
}

void TestCase::handleRender()
{
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		dtCoordinates dir;
		dtVsub(dir, iter->epos, iter->spos);
		dtVnormalize(dir);
		glColor4ub(128,25,0,192);
		glVertex3f(iter->spos.X(),iter->spos.Y()-0.3f,iter->spos.Z());
		glVertex3f(iter->spos.X(),iter->spos.Y()+0.3f,iter->spos.Z());
		glVertex3f(iter->spos.X(),iter->spos.Y()+0.3f,iter->spos.Z());
		glVertex3f(iter->spos.X()+dir.X()*0.3f,iter->spos.Y()+0.3f+dir.Y()*0.3f,iter->spos.Z()+dir.Z()*0.3f);
		glColor4ub(51,102,0,129);
		glVertex3f(iter->epos.X(),iter->epos.Y()-0.3f,iter->epos.Z());
		glVertex3f(iter->epos.X(),iter->epos.Y()+0.3f,iter->epos.Z());

		if (iter->expand)
		{
			const float s = 0.1f;
			glColor4ub(255,32,0,128);
			glVertex3f(iter->spos.X()-s,iter->spos.Y(),iter->spos.Z());
			glVertex3f(iter->spos.X()+s,iter->spos.Y(),iter->spos.Z());
			glVertex3f(iter->spos.X(),iter->spos.Y(),iter->spos.Z()-s);
			glVertex3f(iter->spos.X(),iter->spos.Y(),iter->spos.Z()+s);
			glColor4ub(255,192,0,255);
			glVertex3f(iter->nspos.X()-s,iter->nspos.Y(),iter->nspos.Z());
			glVertex3f(iter->nspos.X()+s,iter->nspos.Y(),iter->nspos.Z());
			glVertex3f(iter->nspos.X(),iter->nspos.Y(),iter->nspos.Z()-s);
			glVertex3f(iter->nspos.X(),iter->nspos.Y(),iter->nspos.Z()+s);
			
			glColor4ub(255,32,0,128);
			glVertex3f(iter->epos.X()-s,iter->epos.Y(),iter->epos.Z());
			glVertex3f(iter->epos.X()+s,iter->epos.Y(),iter->epos.Z());
			glVertex3f(iter->epos.X(),iter->epos.Y(),iter->epos.Z()-s);
			glVertex3f(iter->epos.X(),iter->epos.Y(),iter->epos.Z()+s);
			glColor4ub(255,192,0,255);
			glVertex3f(iter->nepos.X()-s,iter->nepos.Y(),iter->nepos.Z());
			glVertex3f(iter->nepos.X()+s,iter->nepos.Y(),iter->nepos.Z());
			glVertex3f(iter->nepos.X(),iter->nepos.Y(),iter->nepos.Z()-s);
			glVertex3f(iter->nepos.X(),iter->nepos.Y(),iter->nepos.Z()+s);
		}
		
		if (iter->expand)
			glColor4ub(255,192,0,255);
		else
			glColor4ub(0,0,0,64);
			
		for (int i = 0; i < iter->nstraight-1; ++i)
		{
			glVertex3f(iter->straight[i].X(),iter->straight[i].Y()+0.3f,iter->straight[i].Z());
			glVertex3f(iter->straight[(i+1)].X(),iter->straight[(i+1)].Y()+0.3f,iter->straight[(i+1)].Z());
		}
	}
	glEnd();
	glLineWidth(1.0f);
}

bool TestCase::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	char text[64], subtext[64];
	int n = 0;

	static const float LABEL_DIST = 1.0f;

	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		dtCoordinates pt, dir;
		if (iter->nstraight)
		{
			dtVcopy(pt, iter->straight[1]);
			if (dtVdist(pt, iter->spos) > LABEL_DIST)
			{
				dtVsub(dir, pt, iter->spos);
				dtVnormalize(dir);
				dtVmad(pt, iter->spos, dir, LABEL_DIST);
			}
			pt.SetY( pt.Y()+0.5f );
		}
		else
		{
			dtVsub(dir, iter->epos, iter->spos);
			dtVnormalize(dir);
			dtVmad(pt, iter->spos, dir, LABEL_DIST);
			pt.SetY( pt.Y()+0.5f );
		}
		
		if (gluProject((GLdouble)pt.X(), (GLdouble)pt.Y(), (GLdouble)pt.Z(),
					   model, proj, view, &x, &y, &z))
		{
			snprintf(text, 64, "Path %d\n", n);
			unsigned int col = imguiRGBA(0,0,0,128);
			if (iter->expand)
				col = imguiRGBA(255,192,0,220);
			imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, text, col);
		}
		n++;
	}
	
	static int resScroll = 0;
	bool mouseOverMenu = imguiBeginScrollArea("Test Results", 10, view[3] - 10 - 350, 200, 350, &resScroll);
//		mouseOverMenu = true;
		
	n = 0;
	for (Test* iter = m_tests; iter; iter = iter->next)
	{
		const int total = iter->findNearestPolyTime + iter->findPathTime + iter->findStraightPathTime;
		snprintf(subtext, 64, "%.4f ms", (float)total/1000.0f);
		snprintf(text, 64, "Path %d", n);
		
		if (imguiCollapse(text, subtext, iter->expand))
			iter->expand = !iter->expand;
		if (iter->expand)
		{
			snprintf(text, 64, "Poly: %.4f ms", (float)iter->findNearestPolyTime/1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Path: %.4f ms", (float)iter->findPathTime/1000.0f);
			imguiValue(text);

			snprintf(text, 64, "Straight: %.4f ms", (float)iter->findStraightPathTime/1000.0f);
			imguiValue(text);
			
			imguiSeparator();
		}
		
		n++;
	}

	imguiEndScrollArea();
	
	return mouseOverMenu;
}
