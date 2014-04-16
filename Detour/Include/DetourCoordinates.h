
#pragma once

class dtCoordinates
{
public:
	dtCoordinates();
	dtCoordinates( const dtCoordinates& rhs );
	dtCoordinates( const float* rhs );
	dtCoordinates( const float x, const float y, const float z );
	~dtCoordinates();

	dtCoordinates&	operator = ( const dtCoordinates& rhs );
	dtCoordinates&	operator = ( const float* rhs );

	dtCoordinates	operator +( const dtCoordinates& rhs ) const;
	dtCoordinates	operator -( const dtCoordinates& rhs ) const;
	dtCoordinates	operator *( const dtCoordinates& rhs ) const;
	dtCoordinates	operator /( const dtCoordinates& rhs ) const;

	float	X() const	{	return m_X;	}
	float	Y() const	{	return m_Y;	}
	float	Z() const	{	return m_Z;	}

	void	SetX( const float x )	{	m_X = x;	}
	void	SetY( const float y )	{	m_Y = y;	}
	void	SetZ( const float z )	{	m_Z = z;	}

private:
	float	m_X;
	float	m_Y;
	float	m_Z;
};
