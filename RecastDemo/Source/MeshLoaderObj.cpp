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

#include "MeshLoaderObj.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>

rcMeshLoaderObj::rcMeshLoaderObj() :
	m_scale(1.0f),
	m_verts(0),
	m_tris(0),
	m_normals(0),
	m_vertCount(0),
	m_triCount(0)
{
#ifdef INTEGRATION_BUILD
	m_integrationBuild = false;
	m_correctVertsCount = 0;
#endif // INTEGRATION_BUILD
#ifdef VARIABLE_TILE_SIZE
	m_countVariableHeight = 0;
#endif // VARIABLE_TILE_SIZE
#ifdef MODIFY_SQUARE_SECTOR
	m_square_min.SetX( FLT_MAX );
	m_square_min.SetZ( FLT_MAX );
	m_square_max.SetX( -FLT_MAX );
	m_square_max.SetZ( -FLT_MAX );
#endif // MODIFY_SQUARE_SECTOR
	m_terrain_count = 0;
}

rcMeshLoaderObj::~rcMeshLoaderObj()
{
	delete [] m_verts;
	delete [] m_normals;
	delete [] m_tris;
}

void rcMeshLoaderObj::addVertex(float x, float y, float z, int& cap)
{
	if( m_vertCount < RC_MAX_GROUND_FLOOR_VERTICES ) {
#ifdef MODIFY_SQUARE_SECTOR
		m_square_min.SetX( rcMin( x, m_square_min.X() ) );
		m_square_min.SetZ( rcMin( z, m_square_min.Z() ) );
		m_square_max.SetX( rcMax( x, m_square_max.X() ) );
		m_square_max.SetZ( rcMax( z, m_square_max.Z() ) );
#endif // MODIFY_SQUARE_SECTOR

#ifdef VARIABLE_TILE_SIZE
		static const float f = 0.001f;
		const float prevHeight = m_vertCount > 0 ? m_verts[(m_vertCount-1)].Y() : y;
		if( f < rcAbs( prevHeight - y ) ) {
			++m_countVariableHeight;
		}
#endif // VARIABLE_TILE_SIZE
	}

#ifdef LOAD_ONLY_OBJECT
	++m_terrain_count;
	if( m_terrain_count <= RC_MAX_GROUND_FLOOR_VERTICES ) {
		return;
	}
#endif // LOAD_ONLY_OBJECT
#ifdef LOAD_ONLY_TERRAIN
	++m_terrain_count;
	if( RC_MAX_GROUND_FLOOR_VERTICES < m_terrain_count ) {
		return;
	}
#endif // LOAD_ONLY_TERRAIN
	if (m_vertCount+1 > cap)
	{
		cap = !cap ? 8 : cap*2;
		dtCoordinates* nv = new dtCoordinates[cap];
		if (m_vertCount)
			memcpy(nv, m_verts, m_vertCount*sizeof(dtCoordinates));
		delete [] m_verts;
		m_verts = nv;
	}
	
	m_verts[m_vertCount].SetX( x*m_scale );
	m_verts[m_vertCount].SetY( y*m_scale );
	m_verts[m_vertCount].SetZ( z*m_scale );
	m_vertCount++;
}

void rcMeshLoaderObj::addTriangle(int a, int b, int c, int& cap)
{
#ifdef INTEGRATION_BUILD
	a += m_correctVertsCount;
	b += m_correctVertsCount;
	c += m_correctVertsCount;
#endif // INTEGRATION_BUILD

	if (m_triCount+1 > cap)
	{
		cap = !cap ? 8 : cap*2;
		int* nv = new int[cap*3];
		if (m_triCount)
			memcpy(nv, m_tris, m_triCount*3*sizeof(int));
		delete [] m_tris;
		m_tris = nv;
	}
	int* dst = &m_tris[m_triCount*3];
	*dst++ = a;
	*dst++ = b;
	*dst++ = c;
	m_triCount++;
}

static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool cont = false;
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
			case '\\':
				cont = true; // multirow
				break;
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
				cont = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	return buf;
}

static int parseFace(char* row, int* data, int n, int vcnt)
{
	int j = 0;
	while (*row != '\0')
	{
		// Skip initial white space
		while (*row != '\0' && (*row == ' ' || *row == '\t'))
			row++;
		char* s = row;
		// Find vertex delimiter and terminated the string there for conversion.
		while (*row != '\0' && *row != ' ' && *row != '\t')
		{
			if (*row == '/') *row = '\0';
			row++;
		}
		if (*s == '\0')
			continue;
		int vi = atoi(s);
		data[j++] = vi < 0 ? vi+vcnt : vi-1;
		if (j >= n) return j;
	}
	return j;
}

#include "Filelist.h"
bool rcMeshLoaderObj::load(const char* filename)
{
#ifdef INTEGRATION_BUILD
	if( m_integrationBuild ) {
#else // INTEGRATION_BUILD
	if( false ) {
#endif // INTEGRATION_BUILD
		char drive[256] = { 0 };
		char dir[256] = { 0 };
		char name[256] = { 0 };
		char ext[256] = { 0 };
		::_splitpath_s( filename, drive, dir, name, ext );

		FileList meshFiles;
		scanDirectory( dir, ".obj", meshFiles );

		int vcap = 0;
		int tcap = 0;

		for( int nth = 0; nth < meshFiles.size; ++nth ) {
			char path[256] = { 0 };
			strcpy( path, dir );
			strcat( path, meshFiles.files[nth] );
			//////////////////////////////////////////////////////////////////////////
			char* buf = 0;
			FILE* fp = fopen(path/*filename*/, "rb");
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
			fread(buf, bufSize, 1, fp);
			fclose(fp);

			char* src = buf;
			char* srcEnd = buf + bufSize;
			char row[512];
			int face[32];
			float x,y,z;
			int nv;
			// 	int vcap = 0;
			// 	int tcap = 0;

			while (src < srcEnd)
			{
				// Parse one row
				row[0] = '\0';
				src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
				// Skip comments
				if (row[0] == '#') continue;
				if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
				{
					// Vertex pos
					sscanf(row+1, "%f %f %f", &x, &y, &z);
					addVertex(x, y, z, vcap);
				}
				if (row[0] == 'f')
				{
					// Faces
					nv = parseFace(row+1, face, 32, m_vertCount);
					for (int i = 2; i < nv; ++i)
					{
						/*const*/ int a = face[0];
						/*const*/ int b = face[i-1];
						/*const*/ int c = face[i];
#ifdef LOAD_ONLY_OBJECT
						a -= RC_MAX_GROUND_FLOOR_VERTICES;
						b -= RC_MAX_GROUND_FLOOR_VERTICES;
						c -= RC_MAX_GROUND_FLOOR_VERTICES;
						if( a < 0 || b < 0 || c < 0 ) {
							continue;
						}
#endif // LOAD_ONLY_OBJECT
#ifdef LOAD_ONLY_TERRAIN
						if( RC_MAX_GROUND_FLOOR_VERTICES <= a || RC_MAX_GROUND_FLOOR_VERTICES <= b || RC_MAX_GROUND_FLOOR_VERTICES <= c ) {
							continue;
						}
#endif // LOAD_ONLY_TERRAIN
						if (a < 0 || a >= m_vertCount || b < 0 || b >= m_vertCount || c < 0 || c >= m_vertCount)
							continue;
						addTriangle(a, b, c, tcap);
					}
				}
			}

#ifdef INTEGRATION_BUILD
			m_correctVertsCount = m_vertCount;
#endif // INTEGRATION_BUILD
			delete [] buf;
		}
	}
	else {
		//////////////////////////////////////////////////////////////////////////
		char* buf = 0;
		FILE* fp = fopen(filename, "rb");
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
		fread(buf, bufSize, 1, fp);
		fclose(fp);

		char* src = buf;
		char* srcEnd = buf + bufSize;
		char row[512];
		int face[32];
		float x,y,z;
		int nv;
		int vcap = 0;
		int tcap = 0;

		while (src < srcEnd)
		{
			// Parse one row
			row[0] = '\0';
			src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
			// Skip comments
			if (row[0] == '#') continue;
			if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
			{
				// Vertex pos
				sscanf(row+1, "%f %f %f", &x, &y, &z);
				addVertex(x, y, z, vcap);
			}
			if (row[0] == 'f')
			{
				// Faces
				nv = parseFace(row+1, face, 32, m_vertCount);
				for (int i = 2; i < nv; ++i)
				{
					/*const*/ int a = face[0];
					/*const*/ int b = face[i-1];
					/*const*/ int c = face[i];
#ifdef LOAD_ONLY_OBJECT
					a -= RC_MAX_GROUND_FLOOR_VERTICES;
					b -= RC_MAX_GROUND_FLOOR_VERTICES;
					c -= RC_MAX_GROUND_FLOOR_VERTICES;
					if( a < 0 || b < 0 || c < 0 ) {
						continue;
					}
#endif // LOAD_ONLY_OBJECT
#ifdef LOAD_ONLY_TERRAIN
					if( RC_MAX_GROUND_FLOOR_VERTICES <= a || RC_MAX_GROUND_FLOOR_VERTICES <= b || RC_MAX_GROUND_FLOOR_VERTICES <= c ) {
						continue;
					}
#endif // LOAD_ONLY_TERRAIN
					if (a < 0 || a >= m_vertCount || b < 0 || b >= m_vertCount || c < 0 || c >= m_vertCount)
						continue;
					addTriangle(a, b, c, tcap);
				}
			}
		}

		delete [] buf;
	}
	//////////////////////////////////////////////////////////////////////////
	

	// Calculate normals.
	m_normals = new dtCoordinates[m_triCount];
	int ni = 0;
	for (int i = 0; i < m_triCount*3; i += 3)
	{
		const dtCoordinates v0( m_verts[m_tris[i]] );
		const dtCoordinates v1( m_verts[m_tris[i+1]] );
		const dtCoordinates v2( m_verts[m_tris[i+2]] );
		dtCoordinates e0, e1;
		e0.SetX( v1.X() - v0.X() );
		e0.SetY( v1.Y() - v0.Y() );
		e0.SetZ( v1.Z() - v0.Z() );

		e1.SetX( v2.X() - v0.X() );
		e1.SetY( v2.Y() - v0.Y() );
		e1.SetZ( v2.Z() - v0.Z() );

		dtCoordinates* n = &m_normals[ni];
		n->SetX( e0.Y()*e1.Z() - e0.Z()*e1.Y() );
		n->SetY( e0.Z()*e1.X() - e0.X()*e1.Z() );
		n->SetZ( e0.X()*e1.Y() - e0.Y()*e1.X() );
		float d = sqrtf(n->X()*n->X() + n->Y()*n->Y() + n->Z()*n->Z());
		if (d > 0)
		{
			d = 1.0f/d;
			n->SetX( n->X() * d );
			n->SetY( n->Y() * d );
			n->SetZ( n->Z() * d );
		}
		++ni;
	}
	
	strncpy(m_filename, filename, sizeof(m_filename));
	m_filename[sizeof(m_filename)-1] = '\0';

	return true;
}
