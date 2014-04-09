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

#ifndef RECASTSAMPLEDEBUG_H
#define RECASTSAMPLEDEBUG_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include <DetourCoordinates.h>

/// Sample used for random debugging.
class Sample_Debug : public Sample
{
protected:
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_pmesh;

	dtCoordinates m_ext;
	dtCoordinates m_center;
	dtCoordinates m_bmin, m_bmax;
	dtPolyRef m_ref;
	
public:
	Sample_Debug();
	virtual ~Sample_Debug();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const dtCoordinates& s, const dtCoordinates& p, bool shift);
	virtual void handleToggle();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();

	virtual const dtCoordinates* getBoundsMin();
	virtual const dtCoordinates* getBoundsMax();
};


#endif // RECASTSAMPLE_H
