#ifndef _TRAJECTORY_GENERATOR_H
#define _TRAJECTORY_GENERATOR_H

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <Trajectory/cubicInterpolation.h>

class Tcp6D
{
public:
	double x, y, z, a, b, c;
	Tcp6D(double x_pos=0, double y_pos=0, double z_pos=0, double a_ori=0, double b_ori=0, double c_ori=0):
		x(x_pos), y(y_pos), z(z_pos), a(a_ori), b(b_ori), c(c_ori){}

	void input( double x_pos, double y_pos, double z_pos, double a_ori, double b_ori, double c_ori)
	{
		x = x_pos;
		y = y_pos;
		z = z_pos;
		a = a_ori;
		b = b_ori;
		c = c_ori;
	}
};

class Trajectory_Generator
{
private:
	boost::shared_ptr< rl::kin::Kinematics > kin;
	Eigen::MatrixXd jointData;  
	int dof;
	int ndata;
	
public:
	std::vector< std::pair< double, Tcp6D> >  trajData;
	cubicInterpolation* cubic;
	double tracking_time;						// total time in second

	Trajectory_Generator( boost::shared_ptr< rl::kin::Kinematics> kinematics);
	~Trajectory_Generator();
	void addData( double, Tcp6D data);   // time and tcp point
	void clear();

	bool generate( float joint[]);
};




#endif
