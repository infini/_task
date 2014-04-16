
#include "DetourCoordinates.h"

dtCoordinates::dtCoordinates()
: m_X( 0 )
, m_Y( 0 )
, m_Z( 0 )
{
}

dtCoordinates::dtCoordinates( const dtCoordinates& rhs )
: m_X( rhs.X() )
, m_Y( rhs.Y() )
, m_Z( rhs.Z() )
{
}

dtCoordinates::dtCoordinates( const float* rhs )
: m_X( rhs[0] )
, m_Y( rhs[1] )
, m_Z( rhs[2] )
{
}

dtCoordinates::dtCoordinates( const float x, const float y, const float z )
: m_X( x )
, m_Y( y )
, m_Z( z )
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

dtCoordinates&	dtCoordinates::operator = ( const float* rhs )
{
	m_X = rhs[0];
	m_Y = rhs[1];
	m_Z = rhs[2];

	return *this;
}

dtCoordinates	dtCoordinates::operator +( const dtCoordinates& rhs ) const
{
	return dtCoordinates( m_X + rhs.X(), m_Y + rhs.Y(), m_Z + rhs.Z() );
}

dtCoordinates	dtCoordinates::operator -( const dtCoordinates& rhs ) const
{
	return dtCoordinates( m_X - rhs.X(), m_Y - rhs.Y(), m_Z - rhs.Z() );
}

dtCoordinates	dtCoordinates::operator *( const dtCoordinates& rhs ) const
{
	return dtCoordinates( m_X * rhs.X(), m_Y * rhs.Y(), m_Z * rhs.Z() );
}

dtCoordinates	dtCoordinates::operator /( const dtCoordinates& rhs ) const
{
	return dtCoordinates( m_X / rhs.X(), m_Y / rhs.Y(), m_Z / rhs.Z() );
}
