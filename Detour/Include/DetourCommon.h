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

#ifndef DETOURCOMMON_H
#define DETOURCOMMON_H

#include "DetourCoordinates.h"

/**
@defgroup detour Detour

Members in this module are used to create, manipulate, and query navigation 
meshes.

@note This is a summary list of members.  Use the index or search 
feature to find minor members.
*/

/// @name General helper functions
/// @{

/// Used to ignore a function parameter.  VS complains about unused parameters
/// and this silences the warning.
///  @param [in] _ Unused parameter
template<class T> void dtIgnoreUnused(const T&) { }

/// Swaps the values of the two parameters.
///  @param[in,out]	a	Value A
///  @param[in,out]	b	Value B
template<class T> inline void dtSwap(T& a, T& b) { T t = a; a = b; b = t; }

/// Returns the minimum of two values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The minimum of the two values.
template<class T> inline T dtMin(T a, T b) { return a < b ? a : b; }

/// Returns the maximum of two values.
///  @param[in]		a	Value A
///  @param[in]		b	Value B
///  @return The maximum of the two values.
template<class T> inline T dtMax(T a, T b) { return a > b ? a : b; }

/// Returns the absolute value.
///  @param[in]		a	The value.
///  @return The absolute value of the specified value.
template<class T> inline T dtAbs(T a) { return a < 0 ? -a : a; }

/// Returns the square of the value.
///  @param[in]		a	The value.
///  @return The square of the value.
template<class T> inline T dtSqr(T a) { return a*a; }

/// Clamps the value to the specified range.
///  @param[in]		v	The value to clamp.
///  @param[in]		mn	The minimum permitted return value.
///  @param[in]		mx	The maximum permitted return value.
///  @return The value, clamped to the specified range.
template<class T> inline T dtClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

/// Returns the square root of the value.
///  @param[in]		x	The value.
///  @return The square root of the vlaue.
float dtSqrt(float x);

/// @}
/// @name Vector helper functions.
/// @{

/// Derives the cross product of two vectors. (@p v1 x @p v2)
///  @param[out]	dest	The cross product. [(x, y, z)]
///  @param[in]		v1		A Vector [(x, y, z)]
///  @param[in]		v2		A vector [(x, y, z)]
inline void dtVcross(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2)
{
	dest.SetX( v1.Y()*v2.Z() - v1.Z()*v2.Y() );
	dest.SetY( v1.Z()*v2.X() - v1.X()*v2.Z() );
	dest.SetZ( v1.X()*v2.Y() - v1.Y()*v2.X() );
}

/// Derives the dot product of two vectors. (@p v1 . @p v2)
///  @param[in]		v1	A Vector [(x, y, z)]
///  @param[in]		v2	A vector [(x, y, z)]
/// @return The dot product.
inline float dtVdot(const dtCoordinates& v1, const dtCoordinates& v2)
{
	return v1.X()*v2.X() + v1.Y()*v2.Y() + v1.Z()*v2.Z();
}

/// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
///  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
inline void dtVmad(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2, const float s)
{
	dest.SetX( v1.X()+v2.X()*s );
	dest.SetY( v1.Y()+v2.Y()*s );
	dest.SetZ( v1.Z()+v2.Z()*s );
}

/// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
///  @param[out]	dest	The result vector. [(x, y, x)]
///  @param[in]		v1		The starting vector.
///  @param[in]		v2		The destination vector.
///	 @param[in]		t		The interpolation factor. [Limits: 0 <= value <= 1.0]
inline void dtVlerp(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2, const float t)
{
	dest.SetX( v1.X()+(v2.X()-v1.X())*t );
	dest.SetY( v1.Y()+(v2.Y()-v1.Y())*t );
	dest.SetZ( v1.Z()+(v2.Z()-v1.Z())*t );
}

/// Performs a vector addition. (@p v1 + @p v2)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
inline void dtVadd(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2)
{
	dest.SetX( v1.X()+v2.X() );
	dest.SetY( v1.Y()+v2.Y() );
	dest.SetZ( v1.Z()+v2.Z() );
}

/// Performs a vector subtraction. (@p v1 - @p v2)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v1		The base vector. [(x, y, z)]
///  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
inline void dtVsub(dtCoordinates& dest, const dtCoordinates& v1, const dtCoordinates& v2)
{
	dest.SetX( v1.X()-v2.X() );
	dest.SetY( v1.Y()-v2.Y() );
	dest.SetZ( v1.Z()-v2.Z() );
}

/// Scales the vector by the specified value. (@p v * @p t)
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		v		The vector to scale. [(x, y, z)]
///  @param[in]		t		The scaling factor.
inline void dtVscale(dtCoordinates& dest, const dtCoordinates& v, const float t)
{
	dest.SetX( v.X()*t );
	dest.SetY( v.Y()*t );
	dest.SetZ( v.Z()*t );
}

/// Selects the minimum value of each element from the specified vectors.
///  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]	v	A vector. [(x, y, z)]
inline void dtVmin(dtCoordinates& mn, const dtCoordinates& v)
{
	mn.SetX( dtMin(mn.X(), v.X()) );
	mn.SetY( dtMin(mn.Y(), v.Y()) );
	mn.SetZ( dtMin(mn.Z(), v.Z()) );
}

/// Selects the maximum value of each element from the specified vectors.
///  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
///  @param[in]		v	A vector. [(x, y, z)]
inline void dtVmax(dtCoordinates& mx, const dtCoordinates& v)
{
	mx.SetX( dtMax(mx.X(), v.X()) );
	mx.SetY( dtMax(mx.Y(), v.Y()) );
	mx.SetZ( dtMax(mx.Z(), v.Z()) );
}

/// Sets the vector elements to the specified values.
///  @param[out]	dest	The result vector. [(x, y, z)]
///  @param[in]		x		The x-value of the vector.
///  @param[in]		y		The y-value of the vector.
///  @param[in]		z		The z-value of the vector.
inline void dtVset(dtCoordinates& dest, const float x, const float y, const float z)
{
	dest.SetX( x );
	dest.SetY( y );
	dest.SetZ( z );
}

/// Performs a vector copy.
///  @param[out]	dest	The result. [(x, y, z)]
///  @param[in]		a		The vector to copy. [(x, y, z)]
inline void dtVcopy(dtCoordinates& dest, const dtCoordinates& a)
{
	dest = a;
}

/// Derives the scalar length of the vector.
///  @param[in]		v The vector. [(x, y, z)]
/// @return The scalar length of the vector.
inline float dtVlen(const dtCoordinates& v)
{
	return dtSqrt(v.X()*v.X() + v.Y()*v.Y() + v.Z()*v.Z());
}

/// Derives the square of the scalar length of the vector. (len * len)
///  @param[in]		v The vector. [(x, y, z)]
/// @return The square of the scalar length of the vector.
inline float dtVlenSqr(const dtCoordinates& v)
{
	return v.X()*v.X() + v.Y()*v.Y() + v.Z()*v.Z();
}

/// Returns the distance between two points.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The distance between the two points.
inline float dtVdist(const dtCoordinates& v1, const dtCoordinates& v2)
{
	const float dx = v2.X() - v1.X();
	const float dy = v2.Y() - v1.Y();
	const float dz = v2.Z() - v1.Z();
	return dtSqrt(dx*dx + dy*dy + dz*dz);
}

/// Returns the square of the distance between two points.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The square of the distance between the two points.
inline float dtVdistSqr(const dtCoordinates& v1, const dtCoordinates& v2)
{
	const float dx = v2.X() - v1.X();
	const float dy = v2.Y() - v1.Y();
	const float dz = v2.Z() - v1.Z();
	return dx*dx + dy*dy + dz*dz;
}

/// Derives the distance between the specified points on the xz-plane.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The distance between the point on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float dtVdist2D(const dtCoordinates& v1, const dtCoordinates& v2)
{
	const float dx = v2.X() - v1.X();
	const float dz = v2.Z() - v1.Z();
	return dtSqrt(dx*dx + dz*dz);
}

/// Derives the square of the distance between the specified points on the xz-plane.
///  @param[in]		v1	A point. [(x, y, z)]
///  @param[in]		v2	A point. [(x, y, z)]
/// @return The square of the distance between the point on the xz-plane.
inline float dtVdist2DSqr(const dtCoordinates& v1, const dtCoordinates& v2)
{
	const float dx = v2.X() - v1.X();
	const float dz = v2.Z() - v1.Z();
	return dx*dx + dz*dz;
}

/// Normalizes the vector.
///  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void dtVnormalize(dtCoordinates& v)
{
	float d = 1.0f / dtSqrt(dtSqr(v.X()) + dtSqr(v.Y()) + dtSqr(v.Z()));
	v.SetX( v.X()*d );
	v.SetY( v.Y()*d );
	v.SetZ( v.Z()*d );
}

/// Performs a 'sloppy' colocation check of the specified points.
///  @param[in]		p0	A point. [(x, y, z)]
///  @param[in]		p1	A point. [(x, y, z)]
/// @return True if the points are considered to be at the same location.
///
/// Basically, this function will return true if the specified points are 
/// close enough to eachother to be considered colocated.
inline bool dtVequal(const dtCoordinates& p0, const dtCoordinates& p1)
{
	static const float thr = dtSqr(1.0f/16384.0f);
	const float d = dtVdistSqr(p0, p1);
	return d < thr;
}

/// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
///  @param[in]		u		A vector [(x, y, z)]
///  @param[in]		v		A vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float dtVdot2D(const dtCoordinates& u, const dtCoordinates& v)
{
	return u.X()*v.X() + u.Z()*v.Z();
}

/// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
///  @param[in]		u		The LHV vector [(x, y, z)]
///  @param[in]		v		The RHV vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float dtVperp2D(const dtCoordinates& u, const dtCoordinates& v)
{
	return u.Z()*v.X() - u.X()*v.Z();
}

/// @}
/// @name Computational geometry helper functions.
/// @{

/// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
///  @param[in]		a		Vertex A. [(x, y, z)]
///  @param[in]		b		Vertex B. [(x, y, z)]
///  @param[in]		c		Vertex C. [(x, y, z)]
/// @return The signed xz-plane area of the triangle.
inline float dtTriArea2D(const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c)
{
	const float abx = b.X() - a.X();
	const float abz = b.Z() - a.Z();
	const float acx = c.X() - a.X();
	const float acz = c.Z() - a.Z();
	return acx*abz - abx*acz;
}

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapBounds
inline bool dtOverlapQuantBounds(const unsigned short amin[3], const unsigned short amax[3],
								 const unsigned short bmin[3], const unsigned short bmax[3])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

/// Determines if two axis-aligned bounding boxes overlap.
///  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
///  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
///  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
///  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
/// @return True if the two AABB's overlap.
/// @see dtOverlapQuantBounds
inline bool dtOverlapBounds(const dtCoordinates& amin, const dtCoordinates& amax,
							const dtCoordinates& bmin, const dtCoordinates& bmax)
{
	bool overlap = true;
	overlap = (amin.X() > bmax.X() || amax.X() < bmin.X()) ? false : overlap;
	overlap = (amin.Y() > bmax.Y() || amax.Y() < bmin.Y()) ? false : overlap;
	overlap = (amin.Z() > bmax.Z() || amax.Z() < bmin.Z()) ? false : overlap;
	return overlap;
}

/// Derives the closest point on a triangle from the specified reference point.
///  @param[out]	closest	The closest point on the triangle.	
///  @param[in]		p		The reference point from which to test. [(x, y, z)]
///  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
void dtClosestPtPointTriangle(dtCoordinates& closest, const dtCoordinates& p,
							  const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c);

/// Derives the y-axis height of the closest point on the triangle from the specified reference point.
///  @param[in]		p		The reference point from which to test. [(x, y, z)]
///  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
///  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
///  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
///  @param[out]	h		The resulting height.
bool dtClosestHeightPointTriangle(const dtCoordinates& p, const dtCoordinates& a, const dtCoordinates& b, const dtCoordinates& c, float& h);

bool dtIntersectSegmentPoly2D(const dtCoordinates& p0, const dtCoordinates& p1,
							  const dtCoordinates* verts, int nverts,
							  float& tmin, float& tmax,
							  int& segMin, int& segMax);

bool dtIntersectSegSeg2D(const dtCoordinates& ap, const dtCoordinates& aq,
						 const dtCoordinates& bp, const dtCoordinates& bq,
						 float& s, float& t);

/// Determines if the specified point is inside the convex polygon on the xz-plane.
///  @param[in]		pt		The point to check. [(x, y, z)]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * @p nverts]
///  @param[in]		nverts	The number of vertices. [Limit: >= 3]
/// @return True if the point is inside the polygon.
bool dtPointInPolygon(const dtCoordinates& pt, const dtCoordinates* verts, const int nverts);

bool dtDistancePtPolyEdgesSqr(const dtCoordinates& pt, const dtCoordinates* verts, const int nverts,
							float* ed, float* et);

float dtDistancePtSegSqr2D(const dtCoordinates& pt, const dtCoordinates& p, const dtCoordinates& q, float& t);

/// Derives the centroid of a convex polygon.
///  @param[out]	tc		The centroid of the polgyon. [(x, y, z)]
///  @param[in]		idx		The polygon indices. [(vertIndex) * @p nidx]
///  @param[in]		nidx	The number of indices in the polygon. [Limit: >= 3]
///  @param[in]		verts	The polygon vertices. [(x, y, z) * vertCount]
void dtCalcPolyCenter(dtCoordinates& tc, const unsigned short* idx, int nidx, const dtCoordinates* verts);

/// Determines if the two convex polygons overlap on the xz-plane.
///  @param[in]		polya		Polygon A vertices.	[(x, y, z) * @p npolya]
///  @param[in]		npolya		The number of vertices in polygon A.
///  @param[in]		polyb		Polygon B vertices.	[(x, y, z) * @p npolyb]
///  @param[in]		npolyb		The number of vertices in polygon B.
/// @return True if the two polygons overlap.
bool dtOverlapPolyPoly2D(const dtCoordinates* polya, const int npolya,
						 const dtCoordinates* polyb, const int npolyb);

/// @}
/// @name Miscellanious functions.
/// @{

inline unsigned int dtNextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int dtIlog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

inline int dtAlign4(int x) { return (x+3) & ~3; }

inline int dtOppositeTile(int side) { return (side+4) & 0x7; }

inline void dtSwapByte(unsigned char* a, unsigned char* b)
{
	unsigned char tmp = *a;
	*a = *b;
	*b = tmp;
}

inline void dtSwapEndian(unsigned short* v)
{
	unsigned char* x = (unsigned char*)v;
	dtSwapByte(x+0, x+1);
}

inline void dtSwapEndian(short* v)
{
	unsigned char* x = (unsigned char*)v;
	dtSwapByte(x+0, x+1);
}

inline void dtSwapEndian(unsigned int* v)
{
	unsigned char* x = (unsigned char*)v;
	dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
}

inline void dtSwapEndian(int* v)
{
	unsigned char* x = (unsigned char*)v;
	dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
}

inline void dtSwapEndian(float* v)
{
	unsigned char* x = (unsigned char*)v;
	dtSwapByte(x+0, x+3); dtSwapByte(x+1, x+2);
}

void dtRandomPointInConvexPoly(const dtCoordinates* pts, const int npts, float* areas,
							   const float s, const float t, dtCoordinates& out);

//////////////////////////////////////////////////////////////////////////
// MIRCHANG
inline bool dtOverlapQuantBounds2D(const unsigned short amin[3], const unsigned short amax[3],
								   const unsigned short bmin[3], const unsigned short bmax[3])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

inline bool dtOverlapBounds2D(const dtCoordinates& amin, const dtCoordinates& amax,
							  const dtCoordinates& bmin, const dtCoordinates& bmax)
{
	bool overlap = true;
	overlap = (amin.X() > bmax.X() || amax.X() < bmin.X()) ? false : overlap;
	overlap = (amin.Z() > bmax.Z() || amax.Z() < bmin.Z()) ? false : overlap;
	return overlap;
}

inline bool dtCompleteOverlapBounds2D(const dtCoordinates& amin, const dtCoordinates& amax, const dtCoordinates& bmin, const dtCoordinates& bmax)
{
	if( amin.X() <= bmin.X() && bmax.X() <= amax.X() && amin.Z() <= bmin.Z() && bmax.Z() <= amax.Z() ) {
		return true;
	}
	if( bmin.X() <= amin.X() && amax.X() <= bmax.X() && bmin.Z() <= amin.Z() && amax.Z() <= bmax.Z() ) {
		return true;
	}
	return false;
}

float	dtCorrectHeightPointTriangle( const dtCoordinates& pos, const dtCoordinates* triangle );

bool	_dtIntersectSegmentPoly2D( const dtCoordinates& p0, const dtCoordinates& p1, const dtCoordinates* verts, const int nverts, float& tmin, float& tmax, int& segMin, int& segMax, dtCoordinates& resultPosition );
// MIRCHANG
//////////////////////////////////////////////////////////////////////////

/// @}

#endif // DETOURCOMMON_H

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@fn float dtTriArea2D(const float* a, const float* b, const float* c)
@par

The vertices are projected onto the xz-plane, so the y-values are ignored.

This is a low cost function than can be used for various purposes.  Its main purpose
is for point/line relationship testing.

In all cases: A value of zero indicates that all vertices are collinear or represent the same point.
(On the xz-plane.)

When used for point/line relationship tests, AB usually represents a line against which
the C point is to be tested.  In this case:

A positive value indicates that point C is to the left of line AB, looking from A toward B.<br/>
A negative value indicates that point C is to the right of lineAB, looking from A toward B.

When used for evaluating a triangle:

The absolute value of the return value is two times the area of the triangle when it is
projected onto the xz-plane.

A positive return value indicates:

<ul>
<li>The vertices are wrapped in the normal Detour wrap direction.</li>
<li>The triangle's 3D face normal is in the general up direction.</li>
</ul>

A negative return value indicates:

<ul>
<li>The vertices are reverse wrapped. (Wrapped opposite the normal Detour wrap direction.)</li>
<li>The triangle's 3D face normal is in the general down direction.</li>
</ul>

*/
