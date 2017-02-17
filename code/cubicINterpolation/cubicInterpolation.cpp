#include "cubicInterpolation.h"

double cubicInterpolation::getTime()
{
	return time;
}

double cubicInterpolation::x(double time)
{
	search(time);
	double tt = time - t(m);
	return a(m)+b(m)*tt+c(m)*tt*tt+d(m)*tt*tt*tt;
}

double cubicInterpolation::v(double time)
{
	search(time);
	double tt = time - t(m);
	return b(m)+2*c(m)*tt+3*d(m)*tt*tt;
}

double cubicInterpolation::aa(double time)
{
	search(time);
	double tt = time - t(m);
	return 2*c(m)+6*d(m)*tt;
}

void cubicInterpolation::search(double time)
{
	// 搜尋現在時間在哪一個區間
	for( int j = 0; j < n; j++)
		if( time <= t(j+1) && time >= t(j) )
			m = j;	
}

void cubicInterpolation::interpolate()
{
	n = vecData.size()-1;
	
	a.resize(n+1);
	c.resize(n+1);
   	t.resize(n+1);
	b.resize(n);
	d.resize(n);
	h.resize(n);
	A.resize(n+1, n+1);
	B.resize(n+1);
	
	// t, a
	double tmp = 0;
	for( int i = 0; i < n+1; i++) 
	{
		t(i) = vecData[i].first + tmp;
		a(i) = vecData[i].second;
		tmp = t(i);
	}
	
	time = t(n);
	
	std::cout<<"total time:"<<time<<std::endl;
	// h
	for( int i = 0; i < n; i++)
		h(i) = vecData[i+1].first;	
	//	h(i) = vecData[i+1].first - vecData[i].first;

	// Matrix A
	A(0, 0) = 2*h(0);
	A(0, 1) = h(0);
	for( int i = 1; i < n; i++)
	{	
		A( i, i-1) = h(i-1);
		A( i, i  ) = 2*(h(i-1)+h(i));
		A( i, i+1) = h(i);
	}
	A(n, n-1) = h(n-1);
	A(n, n) = 2*h(n-1);

	// Matrix B
	B(0) = 3.0/h(0)*(a(1)-a(0));
	for( int i = 1; i < n; i++)
		B(i) = 3.0/h(i)*(a(i+1)-a(i)) - 3.0/h(i-1)*(a(i)-a(i-1));
	B(n) = -3.0/h(n-1)*(a(n)-a(n-1));

	// Should use LU decomposition instead, but there is compile error here.
	c = A.inverse()*B;

	// get b, d from c
	for( int i = 0; i < n; i++)
	{
		b(i) = 1.0/h(i)*(a(i+1)-a(i)) - h(i)/3.0*(2*c(i)+c(i+1));
		d(i) = (c(i+1)-c(i))/3.0/h(i);
	}

}

void cubicInterpolation::addData( std::pair<float, float> data)
{
	vecData.push_back( data);
}

void cubicInterpolation::clear()
{
	vecData.clear();
}
