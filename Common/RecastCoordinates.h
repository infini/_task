
#pragma once

#include <Point3.h>

class RecastCoordinates
{
public:
	explicit RecastCoordinates( const efd::Point3& position );
	RecastCoordinates( const float x, const float y, const float z );
	~RecastCoordinates();

	bool	getCoordinates( OUT efd::Point3& position ) const;
	bool	getCoordinates( OUT float* position ) const;

private:
	float x;
	float y;
	float z;
};
