#include <iostream>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>

int
main (int argc, char** argv)
{

	if( argc != 4)
	{
		std::cout<<"hint: .exe x y z"<<std::endl;
		return 0;
	}

	double tx, ty, tz;
	tx = atof(argv[1]);
	ty = atof(argv[2]);
	tz = atof(argv[3]);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);

	// predefine parameters (known parameters)
	const int point_num = 10;
	// depends on the setting of the timer
	double sampling_time = 0.01;
	// sphere's radius
	const double radius = 0.5;	
	
	// generate the point cloud--------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	
	for (float z(-radius); z <= radius; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 10.0)
		{
			double r = sqrt(radius*radius-z*z);
			pcl::PointXYZ point;
			point.x = cosf (pcl::deg2rad(angle))*r;
			point.y = sinf (pcl::deg2rad(angle))*r;
			point.z = z;
			cloud_ptr->points.push_back(point);
		}
	}

	for( int i = 0 ; i < cloud_ptr->size(); i++)
	{
		cloud_ptr->points[i].x += tx;  
		cloud_ptr->points[i].y += ty;  
		cloud_ptr->points[i].z += tz;  
	}

	viewer->addPointCloud(cloud_ptr);

	// theta (estimated parameter)
	Eigen::VectorXd theta(4);
	Eigen::VectorXd B(point_num);
	Eigen::MatrixXd A(point_num, 4);


	clock_t start_time, end_time;
	srand((unsigned)time(NULL));
	double total_time;
	double a, b, c;

	start_time = clock();
	int cloud_size = cloud_ptr->size();
	
	for( int i = 0 ; i < point_num; i++)
	{
		double random = (double)(rand()) / (RAND_MAX + 1.0);
		unsigned int index = static_cast<unsigned int>(random*cloud_size);	
		double x, y, z;
		x = cloud_ptr->points[index].x;
		y = cloud_ptr->points[index].y;
		z = cloud_ptr->points[index].z;
		A(i,0) = x;
		A(i,1) = y;
		A(i,2) = z;
		A(i,3) = 1;
		B(i)   = radius*radius - x*x - y*y - z*z;
	}

	// least square 
	theta = (A.transpose()*A).inverse()*A.transpose()*B;

	// calculate the center of sphere (a,b,c)	
	a = -0.5*theta(0);
	b = -0.5*theta(1);
	c = -0.5*theta(2);

	// debug information
	std::cout<<"th0:"<<theta(0)<<"  th1:"<<theta(1)<<"  th2:"<<theta(2)<<std::endl;
	std::cout<<"a:"<<a<<" b:"<<b<<" c:"<<c<<std::endl;

	end_time = clock();
	total_time = (double)(end_time - start_time)/CLOCKS_PER_SEC;
	std::cout<<total_time<<std::endl;

	// put the sphere model on the visualizer-------------------------------------
	pcl::ModelCoefficients sphere_coeff;
	sphere_coeff.values.push_back(a);
	sphere_coeff.values.push_back(b);
	sphere_coeff.values.push_back(c);
	sphere_coeff.values.push_back(radius);
	
	viewer->addSphere(sphere_coeff);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere");

// display---------------------------------------------------------------------
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}
