#pragma once

#include "AC_Math.hpp"

/*
	三维向量类
	20190909：朱文杰
*/

template<typename T>
class complex
{
	private:
		
	public:
		//x+yi
		T x , y;
	
		/*构造函数
		*/
			inline complex(){this->x = this->y = 0;}
			inline complex( T real , T imag )
			{
				this->x = real;
				this->y = imag;
			}
		/*构造函数*/
		
		//向量清零
		inline void zero()
		{
			x=0;	y=0;
		}
		
		//等号运算符重载
		inline complex<T>& operator =( const complex<T>& b )
		{
			x=b.x;	y=b.y;
			return *this;
		}
		
		/*复数基本运算符重载
		*/
			inline complex<T> operator +( const complex<T>& b ) const
			{
				return complex<T>( x+b.x , y+b.y );
			}
			inline complex<T> operator -( const complex<T>& b ) const
			{
				return complex<T>( x-b.x , y-b.y );
			}
			inline complex<T> operator *( const T b ) const
			{
				return complex<T>( x*b , y*b );
			}
			inline complex<T> operator *( const complex<T>& b )const
			{
				return complex<T>( x*b.x-y*b.y , y*b.x+x*b.y );
			}
			inline complex<T> operator /( const T b ) const
			{
				return complex<T>( x/b , y/b );
			}
			inline complex<T> operator -(void) const
			{
				return complex<T>( -x , -y );
			}
			
			inline complex<T> operator +=( const complex<T>& b )
			{
				x += b.x;
				y += b.y;
				return *this;
			}
			
			inline complex<T> operator -=( const complex<T>& b )
			{
				this->x -= b.x;
				this->y -= b.y;
				return *this;
			}
			inline complex<T> operator *=( T b )
			{
				this->x *= b;
				this->y *= b;
				return *this;
			}
			
			inline bool operator ==(const complex<T> &v) const
			{
				return x==v.x && y==v.y;
			}
			
			inline bool operator !=(const complex<T> &v) const
			{
				return x!=v.x || y!=v.y;
			}
		/*复数基本运算符重载*/
		
		//计算向量平方
		inline T get_square() const
		{
			return this->x*this->x + this->y*this->y;
		}
			
		inline void constrain( const T max_length );
		inline void normalize();
};

/*浮点向量等号运算特例*/
	template<>
	inline bool complex<float>::operator ==(const complex<float> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y));
	}
	template<>
	inline bool complex<double>::operator ==(const complex<double> &v) const
	{
			return (is_equal(x,v.x) && is_equal(y,v.y));
	}
	
	template<>
	inline bool complex<float>::operator !=(const complex<float> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y));
	}
	template<>
	inline bool complex<double>::operator !=(const complex<double> &v) const
	{
			return (!is_equal(x,v.x) || !is_equal(y,v.y));
	}
/*浮点向量等号运算特例*/
	
/*浮点向量归一化特例*/
	template<>
	inline void complex<float>::normalize()
	{
		float length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0f / length;
		*this *= length;
	}
	template<>
	inline void complex<double>::normalize()
	{
		double length = safe_sqrt( get_square() );
		if( is_zero( length ) )
			return;
		
		length = 1.0 / length;
		*this *= length;
	}
/*浮点向量归一化特例*/