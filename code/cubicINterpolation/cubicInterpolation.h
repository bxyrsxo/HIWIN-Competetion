#ifndef _CUBICINTERPOLATION_H
#define _CUBICINTERPOLATION_H


#include <Eigen/Core>
#include <Eigen/LU>

#include <iostream>
#include <vector>

class cubicInterpolation
{
private:
	std::vector< std::pair< float, int> > vecData;
	Eigen::VectorXd a, b, c, d, h, t, B;
	Eigen::MatrixXd A;
	int m;				// in which region of time
	int n;              // number of cubic polynomials
	double time;		// total time

public:
	double getTime();
	void addData( std::pair<float, float> );  // time, angle with degree
	void clear();
	// step: interpolate() -> search() -> x(), v(), a()
	void interpolate();
	void search(double time);
	double x (double time);
	double v (double time);
	double aa(double time);
};

#endif
