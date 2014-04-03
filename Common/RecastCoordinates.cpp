
#include "RecastCoordinates.h"

RecastCoordinates::RecastCoordinates( const efd::Point3& position )
: x( position.y )
, y( position.z )
, z( position.x )
{
}

RecastCoordinates::RecastCoordinates( const float x, const float y, const float z )
: x( x )
, y( y )
, z( z )
{
}

RecastCoordinates::~RecastCoordinates()
{
}

bool	RecastCoordinates::getCoordinates( OUT efd::Point3& position ) const
{
	position.x = z;
	position.y = x;
	position.z = y;

	return true;
}

bool	RecastCoordinates::getCoordinates( OUT float* position ) const
{
	position[0] = x;
	position[1] = y;
	position[2] = z;
	
	return true;
}
