/* \author Geoffrey Biggs */

#include <iostream>
#include <vector>
#include <fstream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>

const float radius = 200;

typedef struct Point
{
	float x, y;
}Point;

Point _Point( float x, float y)
{
	Point pt;
	pt.x = x;	pt.y = y;
	return pt;
}

typedef struct Search
{
	int count;
	int nearest_index;
	float nearest_dis;
	int color;
	int shape;
	int size;
	Point centroid;
	float orientation;
}Search;

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer ()
	{
		interface = new pcl::OpenNIGrabber();
		cloud = *(new pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr(new pcl::PointCloud<pcl::PointXYZRGBA>));
	}

	void grab_cloud( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &inputCloud)
	{
			cloud = inputCloud;
	}

	void grab()
	{
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
          boost::bind (&SimpleOpenNIViewer::grab_cloud, this, _1);

		interface->registerCallback (f);
	    interface->start ();
		Sleep(50);
		interface->stop ();  
	 }

	 pcl::Grabber* interface;
	 pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud; 
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> SimpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);
void ShowCloud( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
void PassThroughFilter( pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr inCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outCloud,
	float x1, float x2, float y1, float y2, float z1, float z2);
void VoxelFilter( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_output, float size);
void PlaneSegmentation( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered, pcl::ModelCoefficients::Ptr coefficients);
void RemovePlane( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remove_plane);
void ProjectedXY( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remove_plane, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr project_plane);
void ExtractClusters( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices);
void ShowResult( pcl::PointXY centroid, int color, int shape);
pcl::PointXY Centroid( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster);
int Color(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster);
int Shape( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster);
float Orientation( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXY centroid);
Search SearchRadius( Point pt[], int index, int size);

//-----Main-----
int
main (int argc, char** argv)
{
	// Grab a frame Data(RGBD) from Kinect sensor			 
	SimpleOpenNIViewer v;
	v.grab();

	// 抓取區
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr CatchRegion( new pcl::PointCloud<pcl::PointXYZRGBA>);
	PassThroughFilter( v.cloud, CatchRegion, -0.6, 0.5, 0.02, 0.37, 1.1, 1.5 );

	// Plane Model Segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PlaneSegmentation( CatchRegion, coefficients);

	// Remove table plane
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BallCube( new pcl::PointCloud<pcl::PointXYZRGBA>);
	RemovePlane( CatchRegion, coefficients, BallCube);

	// 將所有點投影至xy平面
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projected (new pcl::PointCloud<pcl::PointXYZRGBA>);
	ProjectedXY( BallCube, projected);

	// Voxel Filter for Down Sampling
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel( new pcl::PointCloud<pcl::PointXYZRGBA>);
	VoxelFilter( projected, voxel, 0.001);
	
	// 分群
	// Creating the KdTree object for the search method of the extraction
	std::vector<pcl::PointIndices> indices;
	ExtractClusters( voxel, indices);

	cout<<indices.size()<<endl;
	
	// Visulize the Point Cloud by Visulizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//ShowCloud( viewer, CatchRegion);
	
	// Search data structure
	std::vector<Search> vecSearch;
	for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cluster->points.push_back (voxel->points[*pit]); //*
		cluster->width = cluster->points.size ();
		cluster->height = 1;
		cluster->is_dense = true;

		Search data;
		// 計算形狀中心、顏色、形狀
		pcl::PointXY centroid = Centroid( cluster);
		int color = Color( cluster);
		int shape = Shape( cluster);

		// 顯示判斷結果
		cout<<"size:"<<cluster->points.size()<<endl;
		ShowResult( centroid, color, shape);
		if( !shape)
		{
			float orientation = Orientation( cluster, centroid);	
			data.orientation = orientation;
		}
		else
			data.orientation = 0;

		data.color = color;
		data.shape = shape;
		data.centroid.x = centroid.x;
		data.centroid.y = centroid.y;
		data.size = cluster->points.size();

		vecSearch.push_back( data);

		//ShowCloud( viewer, cluster);
	}

	// 產生形心的資料給SearchRadius函式
	int len = vecSearch.size();
	Point* centroid_arr = new Point [len];
	for( int i = 0; i != len; i++)
	{
		centroid_arr[i].x = vecSearch[i].centroid.x;
		centroid_arr[i].y = vecSearch[i].centroid.y;
	}

	// 執行SearchRadius，更新Search資料結構內，count, nearest_dis, nearest_index
	for( int i = 0; i != len; i++)
	{
		Search tmp = SearchRadius( centroid_arr, 1, len);
		vecSearch[i].count = tmp.count;
		vecSearch[i].nearest_dis = tmp.nearest_dis;
		vecSearch[i].nearest_index = tmp.nearest_index;
	}

	delete [] centroid_arr;

	// Save to file
	ofstream out("data.txt");
	out<<len<<endl;
	for( int i = 0; i != len; i++)
	{
		out<<vecSearch[i].centroid.x<<" ";
		out<<vecSearch[i].centroid.y<<" ";
		out<<vecSearch[i].color<<" ";
		out<<vecSearch[i].shape<<" ";
		out<<vecSearch[i].size<<" ";
		out<<vecSearch[i].count<<" ";
		out<<vecSearch[i].nearest_dis<<" ";
		out<<vecSearch[i].nearest_index<<" ";
		out<<vecSearch[i].orientation<<endl;
	}
	

	return 0;
}

void PlaneSegmentation( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::ModelCoefficients::Ptr coef)
{
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud ( cloud->makeShared ());
	seg.segment (*inliers, *coef);

	if (inliers->indices.size () == 0)
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");

	//std::cerr << "Model coefficients: " << coef->values[0] << " " 
	//	                                << coef->values[1] << " "
	//		                            << coef->values[2] << " " 
	//			                        << coef->values[3] << std::endl;

	//std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
}

void RemovePlane( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::ModelCoefficients::Ptr coef, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remove)
{
	float a = coef->values[0];
	float b = coef->values[1];
	float c = coef->values[2];
	float d = coef->values[3];

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane( new pcl::PointCloud<pcl::PointXYZRGBA>);
	for( int i = 0; i != cloud->points.size(); i++)
	{
		pcl::PointXYZRGBA pt;
		pt.x = cloud->points[i].x;
		pt.y = cloud->points[i].y;
		pt.z = cloud->points[i].z;
		pt.r = cloud->points[i].r;
		pt.g = cloud->points[i].g;
		pt.b = cloud->points[i].b;

		if( fabs( a*pt.x + b*pt.y + c*pt.z + d) > 0.02 )
			remove->push_back(pt);
		else
			plane->push_back(pt);
	}

	float ymax = 0, xmin = 0;
	float xmax = 0;
	
	for( int i = 0; i != plane->points.size(); i++)
	{
		if( plane->points[i].y > ymax)
			ymax = plane->points[i].y;
		if( plane->points[i].x < xmin)
			xmin = plane->points[i].x;	
	}

	cout<<"("<<xmin<<","<<ymax<<")"<<endl;

}

void ProjectedXY( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projected)
{
	// 將所有點投影至xy平面
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
	coef->values.resize (4);
	coef->values[0] = coef->values[1] = coef->values[3] = 0;
	coef->values[2] = 1.0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (coef);
	proj.filter (*projected);
}

void PassThroughFilter( pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr inCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outCloud,
	float x1, float x2, float y1, float y2, float z1, float z2)
{
	pcl::PointXYZRGBA pt;
	for( int i = 0 ; i != inCloud->points.size(); i++)
	{
		pt.x = inCloud->points[i].x;
		pt.y = inCloud->points[i].y;
		pt.z = inCloud->points[i].z;	
	
		if( pt.z < z2 && pt.z > z1)
			if( pt.y < y2 && pt.y > y1)
				if( pt.x < x2 && pt.x > x1)
				{
					pt.r = inCloud->points[i].r;
					pt.g = inCloud->points[i].g;
					pt.b = inCloud->points[i].b;
					outCloud->push_back( pt);
				}
	}
}

void VoxelFilter( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_output, float size)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	sor.setInputCloud ( cloud);
	sor.setLeafSize (size, size, size);
	sor.filter (*cloud_output);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> SimpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);   // xyz rgba information
	//pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZRGBA> rgba(cloud);	 // xyz information
	viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	return (viewer);
}

void ShowCloud( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	viewer = SimpleVis( cloud);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void ExtractClusters( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices)
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud (cloud);
		
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance (0.005); // 2cm
	ec.setMinClusterSize (300);
	ec.setMaxClusterSize (10000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
}

void ShowResult( pcl::PointXY centroid, int color, int shape)
{
	cout<<"形心位置:"<<"("<<centroid.x*100<<","<<centroid.y*100<<")"<<endl;

	switch( color)
	{
	case 0:
		if( shape)
			cout<<"紅色，圓形"<<endl;
		else
			cout<<"紅色，方形"<<endl;
		break;
	case 1:
		if( shape)
			cout<<"綠色，圓形"<<endl;
		else
			cout<<"綠色，方形"<<endl;
		break;
	case 2:
		if( shape)
			cout<<"藍色，圓形"<<endl;
		else
			cout<<"藍色，方形"<<endl;
		break;
	case 3:
		if( shape)
			cout<<"黃色，圓形"<<endl;
		else
			cout<<"黃色，方形"<<endl;
		break;
	default:
		break;
	}
	cout<<endl;
}

pcl::PointXY Centroid( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster)
{
	pcl::PointXY centroid;
	centroid.x = 0;
	centroid.y = 0;
	for( int i = 0 ; i != cloud_cluster->points.size(); i++)
	{
		centroid.x += cloud_cluster->points[i].x;
		centroid.y += cloud_cluster->points[i].y;
	}
	centroid.x /= cloud_cluster->points.size();
	centroid.y /= cloud_cluster->points.size();
	
	return centroid;
}

int Color(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster)
{
	// 判斷顏色
	float avg_r = 0, avg_g = 0, avg_b = 0;
	for( int i = 0 ; i != cloud_cluster->points.size(); i++)
	{
		float tmp_r, tmp_g, tmp_b;
		tmp_r = cloud_cluster->points[i].r / 255.0;
		tmp_g = cloud_cluster->points[i].g / 255.0;
		tmp_b = cloud_cluster->points[i].b / 255.0;
		avg_r += tmp_r;
		avg_g += tmp_g;
		avg_b += tmp_b;
	}

	avg_r /= cloud_cluster->points.size();
	avg_g /= cloud_cluster->points.size();
	avg_b /= cloud_cluster->points.size();

	cout<<"r:"<<avg_r<<"  g:"<<avg_g<<"  b:"<<avg_b<<endl;

	// default: red ball
	int color = 0;


	// color judgement
	// color: 0->red, 1->green, 2->blue, 3->yellow, 4->undefined
	// convert to ycbcr space
	float y, cb, cr;
	y = 16 + 65.738*avg_r + 129.057*avg_g + 25.064*avg_b;
	cb = 128 - 37.945*avg_r - 74.494*avg_g + 112.439*avg_b;
	cr = 128 + 112.439*avg_r - 94.154*avg_g - 18.285*avg_b;
/*	y  = 16 + 0.2568*avg_r + 0.504*avg_g + 0.0979*avg_b;
	cb = 128 - 0.1482*avg_r - 0.291*avg_g + 0.4392*avg_b;
	cr = 128 + 0.4392*avg_r - 0.3678*avg_g - 0.0714*avg_b*/;

	cout<<"y:"<<y<<endl;
	cout<<"cb:"<<cb<<endl;
	cout<<"cr:"<<cr<<endl;

	
	float ymin, ymax, crmin, crmax, cbmin, cbmax;
	fstream fin;
	fin.open("red.txt", ios::in);
	// red color
	fin>>ymin;
	fin>>ymax;
	fin>>crmin;
	fin>>crmax;
	fin>>cbmin;
	fin>>cbmax;
	fin.close();

	if( y > ymin && y < ymax)
		if( cr > crmin && cr < crmax)
			if( cb > cbmin && cb < cbmax)
				color = 0;
	fin.open("yellow.txt", ios::in);
	// yellow color
	fin>>ymin;
	fin>>ymax;
	fin>>crmin;
	fin>>crmax;
	fin>>cbmin;
	fin>>cbmax;
	fin.close();

	if( y > ymin && y < ymax)
		if( cr > crmin && cr < crmax)
			if( cb > cbmin && cb < cbmax)
				color = 3;
	fin.open("blue.txt", ios::in);
	// blue color
	fin>>ymin;
	fin>>ymax;
	fin>>crmin;
	fin>>crmax;
	fin>>cbmin;
	fin>>cbmax;
	fin.close();

	if( y > ymin && y < ymax)
		if( cr > crmin && cr < crmax)
			if( cb > cbmin && cb < cbmax)
				color = 2;
	
	fin.open("green.txt", ios::in);
	// green color
	fin>>ymin;
	fin>>ymax;
	fin>>crmin;
	fin>>crmax;
	fin>>cbmin;
	fin>>cbmax;
	fin.close();
	if( y > ymin && y < ymax)
		if( cr > crmin && cr < crmax)
			if( cb > cbmin && cb < cbmax)
				color = 1;


	/*
	if( avg_r > 0.3 && avg_g < 0.3 && avg_b < 0.3)
		color = 0;
	else if( avg_r > 0.4 && avg_g > 0.3 && avg_b < 0.3)
		color = 3;
	else if( avg_r < 0.3 && avg_g > 0.2 && avg_b < 0.3)
		color = 1;
	else 
		color = 2;
*/
	return color;
}

int Shape( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster)
{
	int shape;
	// shape: 1->circle, 0->rectangle
	if( cloud_cluster->points.size() > 620)
		shape = 1;
	else
		shape = 0;
	return shape;
}

float Orientation( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointXY centroid)
{
	float x, y;
	float a = 0, b = 0, c = 0;
	float long_axis_angle;

	for( int i = 0; i != cloud->points.size(); i++)
	{
		x = cloud->points[i].x - centroid.x;
        y = cloud->points[i].y - centroid.y;
        a += x*x;
        b += 2*x*y;
        c += y*y;

		if( (a-c) == 0 )
			return 0;    
		else
		{
			// two times derivative of the x^2
			// if > 0, have the maximum
			// if < 0, have the minimum 
			long_axis_angle = atan( (a-c)/b );
			// restriction: angle domain in [-90,90]
			if(long_axis_angle > 0)
				long_axis_angle = 0.5*atan(b/(a-c))/M_PI*180; 
			if(long_axis_angle < 0)
			{
				long_axis_angle = 0.5*atan(b/(a-c))/M_PI*180; 
				if( long_axis_angle > 0)
					long_axis_angle -= 90;
				else
					long_axis_angle += 90;
			}
		}  
	}

	return long_axis_angle; 
}

Search SearchRadius( Point pt[], int index, int size)
{
	Search data;
	data.count = 0;
	data.nearest_dis = 0;
	data.nearest_index = 0;
	
	std::vector< pair< Point, int> > vec;
	float x, y, squrt; //暫存
	
	// 半徑範圍內點數計算
	for( int i = 0; i < size; i++)
	{
		if( i == index)
			continue;
		else
		{
			x = pt[i].x-pt[index].x;
			y = pt[i].y-pt[index].y;
			if( x*x + y*y < radius*radius)
			{
				vec.push_back( make_pair( pt[i], i)  );
				data.count++;
			}
		}
	}

	// 最近點和最近距離計算	
	if( vec.size() )
	{
		x = vec[0].first.x - pt[index].x;
		y = vec[0].first.y - pt[index].y;
		data.nearest_dis = sqrt( x*x + y*y);
		data.nearest_index = vec[0].second;

		for( int i = 1; i != vec.size(); i++)
		{
			x = vec[i].first.x-pt[index].x;
			y = vec[i].first.y-pt[index].y;
			squrt = sqrt(x*x + y*y);
			if(  squrt < data.nearest_dis )
			{
				data.nearest_dis = squrt;
				data.nearest_index = vec[i].second;
			}
		}
	}
	
	cout<<endl;
	cout<<"count:"<<data.count<<endl;
	cout<<"nearest_dis:"<<data.nearest_dis<<endl;
	cout<<"nearest_index:"<<data.nearest_index<<endl;
	
	return data;
}