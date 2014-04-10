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

#include "DetourCommon.h"
#include "DetourMath.h"

//////////////////////////////////////////////////////////////////////////////////////////

float dtSqrt(float x)
{
	return dtMathSqrtf(x);
}

void dtClosestPtPointTriangle(dtCoordinates& closest, const dtCoordinates& p,
							  const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c)
{
	// Check if P in vertex region outside A
	dtCoordinates ab, ac, ap;
	dtVsub(ab, b, a);
	dtVsub(ac, c, a);
	dtVsub(ap, p, a);
	float d1 = dtVdot(ab, ap);
	float d2 = dtVdot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		dtVcopy(closest, a);
		return;
	}
	
	// Check if P in vertex region outside B
	dtCoordinates bp;
	dtVsub(bp, p, b);
	float d3 = dtVdot(ab, bp);
	float d4 = dtVdot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		dtVcopy(closest, b);
		return;
	}
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		float v = d1 / (d1 - d3);
		closest.SetX( a.X() + v * ab.X() );
		closest.SetY( a.Y() + v * ab.Y() );
		closest.SetZ( a.Z() + v * ab.Z() );
		return;
	}
	
	// Check if P in vertex region outside C
	dtCoordinates cp;
	dtVsub(cp, p, c);
	float d5 = dtVdot(ab, cp);
	float d6 = dtVdot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		dtVcopy(closest, c);
		return;
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		float w = d2 / (d2 - d6);
		closest.SetX( a.X() + w * ac.X() );
		closest.SetY( a.Y() + w * ac.Y() );
		closest.SetZ( a.Z() + w * ac.Z() );
		return;
	}
	
	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest.SetX( b.X() + w * (c.X() - b.X()) );
		closest.SetY( b.Y() + w * (c.Y() - b.Y()) );
		closest.SetZ( b.Z() + w * (c.Z() - b.Z()) );
		return;
	}
	
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	closest.SetX( a.X() + ab.X() * v + ac.X() * w );
	closest.SetY( a.Y() + ab.Y() * v + ac.Y() * w );
	closest.SetZ( a.Z() + ab.Z() * v + ac.Z() * w );
}

bool dtIntersectSegmentPoly2D(const dtCoordinates& p0, const dtCoordinates& p1,
							  const dtCoordinates* verts, int nverts,
							  float& tmin, float& tmax,
							  int& segMin, int& segMax)
{
	static const float EPS = 0.00000001f;
	
	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;
	
	dtCoordinates dir;
	dtVsub(dir, p1, p0);
	
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		dtCoordinates edge, diff;
		dtVsub(edge, verts[i], verts[j]);
		dtVsub(diff, p0, verts[j]);
		const float n = dtVperp2D(edge, diff);
		const float d = dtVperp2D(dir, edge);
		if (fabsf(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < 0)
				return false;
			else
				continue;
		}
		const float t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}
	
	return true;
}

float dtDistancePtSegSqr2D(const dtCoordinates& pt, const dtCoordinates& p, const dtCoordinates& q, float& t)
{
	const float pqx = q.X() - p.X();
	const float pqz = q.Z() - p.Z();
	float dx = pt.X() - p.X();
	float dz = pt.Z() - p.Z();
	float d = pqx*pqx + pqz*pqz;
	t = pqx*dx + pqz*dz;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dx = p.X() + t*pqx - pt.X();
	dz = p.Z() + t*pqz - pt.Z();
	return dx*dx + dz*dz;
}

void dtCalcPolyCenter(dtCoordinates& tc, const unsigned short* idx, int nidx, const dtCoordinates* verts)
{
	tc = dtCoordinates();
	for (int j = 0; j < nidx; ++j)
	{
		const dtCoordinates v( verts[idx[j]] );
		dtVadd( tc, tc, v );
	}
	const float s = 1.0f / nidx;
	dtVscale( tc, tc, s );
}

bool dtClosestHeightPointTriangle(const dtCoordinates& p, const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c, float& h)
{
	dtCoordinates v0, v1, v2;
	dtVsub(v0, c,a);
	dtVsub(v1, b,a);
	dtVsub(v2, p,a);
	
	const float dot00 = dtVdot2D(v0, v0);
	const float dot01 = dtVdot2D(v0, v1);
	const float dot02 = dtVdot2D(v0, v2);
	const float dot11 = dtVdot2D(v1, v1);
	const float dot12 = dtVdot2D(v1, v2);
	
	// Compute barycentric coordinates
	const float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
	const float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	const float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.
	static const float EPS = 1e-4f;
	
	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= -EPS && v >= -EPS && (u+v) <= 1+EPS)
	{
		h = a.Y() + v0.Y()*u + v1.Y()*v;
		return true;
	}
	
	return false;
}

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
bool dtPointInPolygon(const dtCoordinates& pt, const dtCoordinates* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool in = false;
	bool edge = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const dtCoordinates vi( verts[i] );
		const dtCoordinates vj( verts[j] );
		if( ( vi.X() == pt.X() && vi.Z() == pt.Z() ) || ( vj.X() == pt.X() && vj.Z() == pt.Z() ) ) {
			return true;
		}

		const float t0 = (vj.X()-vi.X()) * (pt.Z()-vi.Z());
		const float t1 = (vj.Z()-vi.Z());
		const float t = (t0 != 0.f && t1 != 0.f ) ? (t0 / t1 + vi.X()) : vi.X();

		if( ( (pt.Z() < vi.Z()) != (pt.Z() < vj.Z()) ) && (pt.X() < t) ) {
			in = !in;
		}
		else {
			if( (pt.X() == t) && ( (pt.Z() < vi.Z()) != (pt.Z() < vj.Z()) ) ) {
				edge = !edge;
			}
			else if( ( (pt.Z() == vi.Z()) && (pt.Z() == vj.Z()) ) && (pt.X() < t) ) {
				edge = !edge;
			}
		}
	}
	return in || edge;
}

bool dtDistancePtPolyEdgesSqr(const dtCoordinates& pt, const dtCoordinates* verts, const int nverts,
							  float* ed, float* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const dtCoordinates vi( verts[i] );
		const dtCoordinates vj( verts[j] );
		if (((vi.Z() > pt.Z()) != (vj.Z() > pt.Z())) &&
			(pt.X() < (vj.X()-vi.X()) * (pt.Z()-vi.Z()) / (vj.Z()-vi.Z()) + vi.X()) )
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

static void projectPoly(const dtCoordinates& axis, const dtCoordinates* poly, const int npoly,
						float& rmin, float& rmax)
{
	rmin = rmax = dtVdot2D(axis, poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const float d = dtVdot2D(axis, poly[i]);
		rmin = dtMin(rmin, d);
		rmax = dtMax(rmax, d);
	}
}

inline bool overlapRange(const float amin, const float amax,
						 const float bmin, const float bmax,
						 const float eps)
{
	return ((amin+eps) > bmax || (amax-eps) < bmin) ? false : true;
}

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
bool dtOverlapPolyPoly2D(const dtCoordinates* polya, const int npolya,
						 const dtCoordinates* polyb, const int npolyb)
{
	const float eps = 1e-4f;
	
	for (int i = 0, j = npolya-1; i < npolya; j=i++)
	{
		const dtCoordinates va( polya[j] );
		const dtCoordinates vb( polya[i] );
		const dtCoordinates n( vb.Z()-va.Z(), 0, -(vb.X()-va.X()) );
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb-1; i < npolyb; j=i++)
	{
		const dtCoordinates va( polyb[j] );
		const dtCoordinates vb( polyb[i] );
		const dtCoordinates n( vb.Z()-va.Z(), 0, -(vb.X()-va.X()) );
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
void dtRandomPointInConvexPoly(const dtCoordinates* pts, const int npts, float* areas,
							   const float s, const float t, dtCoordinates& out)
{
	// Calc triangle araes
	float areasum = 0.0f;
	for (int i = 2; i < npts; i++) {
		areas[i] = dtTriArea2D(pts[0], pts[(i-1)], pts[i]);
		areasum += dtMax(0.001f, areas[i]);
	}
	// Find sub triangle weighted by area.
	const float thr = s*areasum;
	float acc = 0.0f;
	float u = 0.0f;
	int tri = 0;
	for (int i = 2; i < npts; i++) {
		const float dacc = areas[i];
		if (thr >= acc && thr < (acc+dacc))
		{
			u = (thr - acc) / dacc;
			tri = i;
			break;
		}
		acc += dacc;
	}
	
	float v = dtSqrt(t);
	
	const float a = 1 - v;
	const float b = (1 - u) * v;
	const float c = u * v;
	const dtCoordinates pa( pts[0] );
	const dtCoordinates pb( pts[(tri-1)] );
	const dtCoordinates pc( pts[tri] );
	
	out.SetX( a*pa.X() + b*pb.X() + c*pc.X() );
	out.SetY( a*pa.Y() + b*pb.Y() + c*pc.Y() );
	out.SetZ( a*pa.Z() + b*pb.Z() + c*pc.Z() );
}

inline float vperpXZ(const dtCoordinates& a, const dtCoordinates& b) { return a.X()*b.Z() - a.Z()*b.X(); }

bool dtIntersectSegSeg2D(const dtCoordinates& ap, const dtCoordinates& aq,
						 const dtCoordinates& bp, const dtCoordinates& bq,
						 float& s, float& t)
{
	dtCoordinates u, v, w;
	dtVsub(u,aq,ap);
	dtVsub(v,bq,bp);
	dtVsub(w,ap,bp);
	float d = vperpXZ(u,v);
	if (fabsf(d) < 1e-6f) return false;
	s = vperpXZ(v,w) / d;
	t = vperpXZ(u,w) / d;
	return true;
}

//////////////////////////////////////////////////////////////////////////
// MIRCHANG
float	dtCorrectHeightPointTriangle( const dtCoordinates& pos, const dtCoordinates* triangle )
{
	dtCoordinates v0, v1, v2;
	dtVsub(v0, triangle[2],triangle[0]);
	dtVsub(v1, triangle[1],triangle[0]);
	dtVsub(v2, pos,triangle[0]);

	const float dot00 = dtVdot2D(v0, v0);
	const float dot01 = dtVdot2D(v0, v1);
	const float dot02 = dtVdot2D(v0, v2);
	const float dot11 = dtVdot2D(v1, v1);
	const float dot12 = dtVdot2D(v1, v2);

	const float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
	const float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	const float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	return triangle[0].Y() + v0.Y()*u + v1.Y()*v;
}

//////////////////////////////////////////////////////////////////////////
bool	_dtIntersectSegmentPoly2D( const dtCoordinates& p0, const dtCoordinates& p1, const dtCoordinates* verts, const int nverts, float& tmin, float& tmax, int& segMin, int& segMax, dtCoordinates& resultPosition )
{
	static const float EPS = 0.00000001f;
	static const float gap = 0.00001f;

	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;
	dtCoordinates dir;
	dtVsub( dir, p1, p0 );

	for( int i = 0, j = nverts-1; i < nverts; j=i++ ) {
		const dtCoordinates v0( verts[i] );
		const dtCoordinates v1( verts[j] );

		dtCoordinates edge, diff;
		dtVsub(edge, v0, v1);
		dtVsub(diff, p0, v1);
		const float n = dtVperp2D(edge, diff);
		const float d = dtVperp2D(dir, edge);
		if( fabsf(d) < EPS ) {
			if( n < 0 ) {
				return false;
			}
			else {
				continue;
			}
		}
		const float t = n / d;
		if( d < 0 ) {
			if( tmin < t ) {
				if( t < gap ) {
					continue;
				}
				if( tmax < tmin ) {
					//return false;
					continue;
				}
				tmin = t;
				segMin = j;
			}
		}
		else {
			if( t < tmax ) {
				if( t < gap ) {
					continue;
				}
				if( tmax < tmin ) {
					//return false;
					continue;
				}
				tmax = t;
				segMax = j;
				//////////////////////////////////////////////////////////////////////////
				// intersect point
// 				const float t1 = ((p0.Z() - v0.Z()) * (v1.X() - v0.X())) - ((p0.X() - v0.X()) * (v1.Z() - v0.Z()));
// 				const float t2 = ((p1.X() - p0.X()) * (v1.Z() - v0.Z())) - ((p1.Z() - p0.Z()) * (v1.X() - v0.X()));
// 				float t0 = t1 / t2;
// 				t0 = 0.002f < t0 ? t0 - 0.002f : 0.0f;

				float t0 ( tmax );
				t0 = 0.002f < t0 ? t0 - 0.002f : 0.0f;
				dtVlerp( resultPosition, p0, p1, t0 );
				//////////////////////////////////////////////////////////////////////////
			}
		}
	}

	//return true;
	return segMin != -1 || segMax != -1;

// 	segMin = -1;
// 	segMax = -1;
// 	tmin = 0;
// 	tmax = 0;
// 	float t = 0;
// 	bool intersect = false;
// 
// 	if( dtPointInPolygon( p0, verts, nverts ) && dtPointInPolygon( p1, verts, nverts ) ) {
// 		dtVcopy( resultPosition, p1 );
// 		return true;
// 	}
// 
// 	for( int i = 0, j = nverts-1; i < nverts; j=i++ ) {
// 		const float* v0 = &verts[i*3];
// 		const float* v1 = &verts[j*3];
// 
// 		const float t0 = ((v0[0] - v1[0]) * (p0[2] - v0[2])) + ((v0[2] - v1[2]) * (v0[0] - p0[0]));
// 		const float t1 = ((v0[0] - v1[0]) * (p1[2] - v0[2])) + ((v0[2] - v1[2]) * (v0[0] - p1[0]));
// 
// 		if( t0 * t1 < 0 ) {
// 			intersect = true;
// 			if( t0 < 0 ) {
// 				// max
// 				tmax = tmax == 0 ? (t0 + t1) : tmax;
// 				if( tmax <= (t0 + t1) ) {
// 					segMax = j;
// 					tmax = (t0 + t1);
// 				}
// 				else {
// 					continue;
// 				}
// 			}
// 			else {
// 				// min
// 				tmin = tmin == 0 ? (t0 + t1) : tmin;
// 				if( (t0 + t1) <= tmin ) {
// 					segMin = j;
// 					tmin = (t0 + t1);
// 				}
// 				else {
// 					continue;
// 				}
// 			}
// 			//////////////////////////////////////////////////////////////////////////
// 			// intersect point
// 			const float t3 = ((p0[2] - v0[2]) * (v1[0] - v0[0])) - ((p0[0] - v0[0]) * (v1[2] - v0[2]));
// 			const float t4 = ((p1[0] - p0[0]) * (v1[2] - v0[2])) - ((p1[2] - p0[2]) * (v1[0] - v0[0]));
// 			if( t4 == 0.0f ) {
// 				continue;
// 			}
// 			t = t3 / t4;
// 			t *= 0.95f;
// 			dtVlerp( resultPosition, p0, p1, t );
// 			//////////////////////////////////////////////////////////////////////////
// 		}
// 	}
// 
// 	if( intersect ) {
// 		if( dtPointInPolygon( p1, verts, nverts ) ) {
// 			dtVcopy( resultPosition, p1 );
// 		}
// 		if( !dtPointInPolygon( resultPosition, verts, nverts ) ) {
// 			return false;
// 		}
// 	}
// 
// 	return intersect;
}
//////////////////////////////////////////////////////////////////////////
// MIRCHANG
//////////////////////////////////////////////////////////////////////////
