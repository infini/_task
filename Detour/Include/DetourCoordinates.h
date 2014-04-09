
#pragma once

#include <efd/Point3.h>

class dtCoordinates
{
public:
	dtCoordinates();
	dtCoordinates( const float x, const float y, const float z );
	explicit dtCoordinates( const dtCoordinates& rhs );
	explicit dtCoordinates( const efd::Point3& rhs );
	explicit dtCoordinates( const float* rhs );
	~dtCoordinates();

	dtCoordinates&	operator = ( const dtCoordinates& rhs );
	dtCoordinates&	operator = ( const efd::Point3& rhs );
	dtCoordinates&	operator = ( const float* rhs );

	float	X() const	{	return m_X;	}
	float	Y() const	{	return m_Y;	}
	float	Z() const	{	return m_Z;	}

	void	SetX( const float x )	{	m_X = x;	}
	void	SetY( const float y )	{	m_Y = y;	}
	void	SetZ( const float z )	{	m_Z = z;	}

	void	getCoordinates( OUT efd::Point3& position ) const;
	void	getCoordinates( OUT float* position ) const;

private:
	float	m_X;
	float	m_Y;
	float	m_Z;
};

namespace TransformCoordinates
{
	void	transform( const dtCoordinates& src, float* dest );
}
