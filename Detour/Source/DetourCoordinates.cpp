
#include "DetourCoordinates.h"

dtCoordinates::dtCoordinates()
: m_X( 0 )
, m_Y( 0 )
, m_Z( 0 )
{
}

dtCoordinates::dtCoordinates( const float x, const float y, const float z )
: m_X( x )
, m_Y( y )
, m_Z( z )
{
}

dtCoordinates::dtCoordinates( const dtCoordinates& rhs )
: m_X( rhs.X() )
, m_Y( rhs.Y() )
, m_Z( rhs.Z() )
{
}

dtCoordinates::dtCoordinates( const efd::Point3& rhs )
: m_X( rhs.y )
, m_Y( rhs.z )
, m_Z( rhs.x )
{
}

dtCoordinates::dtCoordinates( const float* rhs )
: m_X( rhs[0] )
, m_Y( rhs[1] )
, m_Z( rhs[2] )
{
}

dtCoordinates::~dtCoordinates()
{
}

dtCoordinates&	dtCoordinates::operator = ( const dtCoordinates& rhs )
{
	m_X = rhs.X();
	m_Y = rhs.Y();
	m_Z = rhs.Z();

	return *this;
}

dtCoordinates&	dtCoordinates::operator = ( const efd::Point3& rhs )
{
	m_X = rhs.y;
	m_Y = rhs.z;
	m_Z = rhs.x;
	
	return *this;
}

dtCoordinates&	dtCoordinates::operator = ( const float* rhs )
{
	m_X = rhs[0];
	m_Y = rhs[1];
	m_Z = rhs[2];

	return *this;
}

void	dtCoordinates::getCoordinates( OUT efd::Point3& position ) const
{
	position.x = m_Z;
	position.y = m_X;
	position.z = m_Y;
}

void	dtCoordinates::getCoordinates( OUT float* position ) const
{
	position[0] = m_X;
	position[1] = m_Y;
	position[2] = m_Z;
}

namespace TransformCoordinates
{
	void	transform( const dtCoordinates& src, float* dest )
	{
		dest[0] = src.X();
		dest[1] = src.Y();
		dest[2] = src.Z();
	}
}
