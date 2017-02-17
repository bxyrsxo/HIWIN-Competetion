#include "Trajectory_Generator.h"

Trajectory_Generator::Trajectory_Generator( boost::shared_ptr< rl::kin::Kinematics> kinematics)
{
	kin = kinematics;
	dof = kin->getDof();
	// trajecotry interpolating here
	cubic = new cubicInterpolation [dof];
}

Trajectory_Generator::~Trajectory_Generator()
{
	delete [] cubic;
}

void Trajectory_Generator::addData( double time, Tcp6D tcp)
{
	trajData.push_back( std::make_pair( time, tcp));
}

void Trajectory_Generator::clear()
{
	trajData.clear();
	jointData.resize(0,0);

	for( int i = 0 ; i < dof; i++)
		cubic[i].clear();
}

bool Trajectory_Generator::generate( float joint[])
{
	ndata = trajData.size();
	jointData.resize( dof, ndata);

	// 解ik，儲存各軸q值
	rl::math::Transform transform;

	//內插次數設定
	const float interpolate_times = 10;
	
	// Set initial guess of IK
	rl::math::Vector q(dof);
	for( int j = 0; j < dof; j++)
		q(j) = joint[j]*rl::math::DEG2RAD;
	kin->setPosition(q);
	kin->updateFrames();	
	
	// 目前end-effector的位置
	const rl::math::Transform::ConstTranslationPart& position = kin->forwardPosition().translation();
	float xi = position.x();
	float yi = position.y();
	float zi = position.z();

	
	for( int i = 0; i < ndata; i++)
	{
		// get the rotation matrix
		double a, b, c, x, y, z;
		x = trajData[i].second.x;
		y = trajData[i].second.y;
		z = trajData[i].second.z;
		a = trajData[i].second.a * rl::math::DEG2RAD;
		b = trajData[i].second.b * rl::math::DEG2RAD;
		c = trajData[i].second.c * rl::math::DEG2RAD;

		Eigen::Matrix3d Rz, Ry, Rx, R;
		Rz << cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1;
		Ry << cos(b), 0, sin(b), 0, 1, 0, -sin(b), 0, cos(b);
		Rx << 1, 0, 0, 0, cos(c), -sin(c), 0, sin(c), cos(c);
		R = Rz*Ry*Rx;

		// cannot input as the MatrixXd type, to find a clear and elegant way to input the data
		transform(0,0) = R(0,0);
		transform(1,0) = R(1,0);
		transform(2,0) = R(2,0);
		transform(0,1) = R(0,1);
		transform(1,1) = R(1,1);
		transform(2,1) = R(2,1);
		transform(0,2) = R(0,2);
		transform(1,2) = R(1,2);
		transform(2,2) = R(2,2);
						
		// 內插的每個step
		float step[3];
		step[0] = (x - xi) / interpolate_times;
		step[1] = (y - yi) / interpolate_times;
		step[2] = (z - zi) / interpolate_times;
		
		for( int j = 0; j < interpolate_times; j++)
		{
			float xp = xi + step[0]*(j+1);
			float yp = yi + step[1]*(j+1);
			float zp = zi + step[2]*(j+1);
			
			transform(0,3) = xp;
			transform(1,3) = yp;
			transform(2,3) = zp;
			//反向運動學計算	
			if (!kin->inversePosition(transform, q))
			{
				std::cout << "out of reach" << std::endl;
				return 0;
			}
		}

		// 儲存結果
		for( int j = 0; j < dof; j++)
			jointData(j, i) = q(j)*rl::math::RAD2DEG;
	}

	std::cout<<jointData<<std::endl;

	// set the first point of the cubic spline
	// time = 0.0, and initial joint angle
	for( int i = 0; i < dof; i++)
		cubic[i].addData( std::make_pair( 0.0, joint[i]));

	// input the IK data and time of each via point
	for( int i = 0; i < dof; i++)
		for( int j = 0; j < ndata; j++)
			cubic[i].addData( std::make_pair( trajData[j].first, jointData(i, j)));

	// cubic interpolation
	for( int i = 0; i < dof; i++)
		cubic[i].interpolate();

	tracking_time = cubic[0].getTime();

	return 1;
}

