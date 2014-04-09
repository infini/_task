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

#ifndef MESHLOADER_OBJ
#define MESHLOADER_OBJ

#include <Recast.h>
#include <DetourCoordinates.h>


class rcMeshLoaderObj
{
public:
	rcMeshLoaderObj();
	~rcMeshLoaderObj();
	
	bool load(const char* fileName);

	inline const dtCoordinates* getVerts() const { return m_verts; }
	inline const dtCoordinates* getNormals() const { return m_normals; }
	inline const int* getTris() const { return m_tris; }
	inline int getVertCount() const { return m_vertCount; }
	inline int getTriCount() const { return m_triCount; }
	inline const char* getFileName() const { return m_filename; }

#ifdef MODIFY_SQUARE_SECTOR
	inline const dtCoordinates& getSquareMin() const { return m_square_min; }
	inline const dtCoordinates& getSquareMax() const { return m_square_max; }
#endif // MODIFY_SQUARE_SECTOR
#ifdef INTEGRATION_BUILD
	inline void			setIntegrationBuild( const bool integrationBuild )	{	m_integrationBuild = integrationBuild;	}
#endif // INTEGRATION_BUILD
#ifdef VARIABLE_TILE_SIZE
	inline int			getVariableHeightCount() const	{	return m_countVariableHeight;	}
#endif // VARIABLE_TILE_SIZE

private:
	
	void addVertex(float x, float y, float z, int& cap);
	void addTriangle(int a, int b, int c, int& cap);
	
	char m_filename[260];
	float m_scale;	
	dtCoordinates* m_verts;
	int* m_tris;
	dtCoordinates* m_normals;
	int m_vertCount;
	int m_triCount;

#ifdef INTEGRATION_BUILD
	bool	m_integrationBuild;
	int	m_correctVertsCount;
#endif // INTEGRATION_BUILD
#ifdef VARIABLE_TILE_SIZE
	int m_countVariableHeight;
#endif // VARIABLE_TILE_SIZE
#ifdef MODIFY_SQUARE_SECTOR
	dtCoordinates m_square_min;
	dtCoordinates m_square_max;
#endif // MODIFY_SQUARE_SECTOR
	int m_terrain_count;
};

#endif // MESHLOADER_OBJ
