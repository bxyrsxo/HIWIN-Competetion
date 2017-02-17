#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <ctime>
#include <vector>
#include <cmath>

using namespace std;
const float radius = 150;

typedef struct Point
{
	float x;
	float y;
}Point;

Point _Point( float x, float y)
{
	Point pt;
	pt.x = x;
	pt.y = y;
	return pt;
}

typedef struct Search
{
	Point centroid;
	int count;
	int nearest_index;
	float nearest_dis;
	int color;
	int shape;
	int size;
	float orientation;
}Search;

void SearchRadius( vector<Search> &vecSearch, int index)
{
	//半徑內計數、最近距離、最近點的索引
	vecSearch[index].count = 0;
	vecSearch[index].nearest_dis = 0;
	vecSearch[index].nearest_index = 0;
	
	// pair存 點坐標(x,y)和index
	std::vector< pair< Point, int> > vec;
	float x, y, squrt; //暫存
	
	//半徑範圍內點數計算  count計算
	int size = vecSearch.size();
	for( int i = 0; i < size; i++)
	{
		if( i == index)
			continue;
		else
		{
			x = vecSearch[i].centroid.x-vecSearch[index].centroid.x;
			y = vecSearch[i].centroid.y-vecSearch[index].centroid.y;
			if( x*x + y*y < radius*radius)
			{
				vec.push_back( make_pair( vecSearch[i].centroid, i)  );
				vecSearch[index].count++;
			}
		}
	}

	// 最近點和最近距離計算	nearest_dis計算
	if( vec.size() )
	{
		x = vec[0].first.x - vecSearch[index].centroid.x;
		y = vec[0].first.y - vecSearch[index].centroid.y;
		vecSearch[index].nearest_dis = sqrt( x*x + y*y);
		vecSearch[index].nearest_index = vec[0].second;

		for( int i = 1; i != vec.size(); i++)
		{
			x = vec[i].first.x-vecSearch[index].centroid.x;
			y = vec[i].first.y-vecSearch[index].centroid.y;
			squrt = sqrt(x*x + y*y);
			if(  squrt < vecSearch[index].nearest_dis )
			{
				vecSearch[index].nearest_dis = squrt;
				vecSearch[index].nearest_index = vec[i].second;
			}
		}
	}
}

Search FindMinCount( vector<Search>& vecSearch)
{
	// 找出最小count的物件
	int count = vecSearch[0].count;
	for( int i = 1; i != vecSearch.size(); i++)
  		if( vecSearch[i].count < count)
			count = vecSearch[i].count;

	//回傳最小count物件的index
	Search data;
	for( int i = 0; i != vecSearch.size(); i++)
		if( vecSearch[i].count == count)
		{
			data = vecSearch[i];
			break;
		}
	return data;
}

//float GraspOrientation( Search& data, vector<Search>& vecSearch)
//{
//
//
//
//}
int MinCountMaxDis( vector<Search> &vecSearch)
{
	//  找出最小count的值
	int index;
	int count = 10000;   // 天文數字
	for( int i = 0; i != vecSearch.size(); i++)
	{
		if( vecSearch[i].count < count)
		{
			count = vecSearch[i].count;
			index = i;
		}
	}

	// 找到相同count的index
	vector<int> minCount;
	for( int i = 0; i != vecSearch.size(); i++)
		if( vecSearch[i].count == count)
			minCount.push_back(i);

	// find the minimum count and maximum nearest_dis
	if( minCount.size())
	{
		index = minCount[0];
		for( int i = 1; i != minCount.size(); i++)
			if( vecSearch[ index ].nearest_dis < vecSearch[ minCount[i] ].nearest_dis)
				index = i;
	}

	return index;
}

float Orientation( vector<Search> vecSearch, int index)
{
	float x,  y;
	int nearest_index = vecSearch[index].nearest_index;

	x = vecSearch[index].centroid.x - vecSearch[nearest_index].centroid.x;
	y = vecSearch[index].centroid.y - vecSearch[nearest_index].centroid.y;

	float th = acos( x / sqrt( x*x + y*y) ) * 57.2958 - 90;  // 徑度轉角度
	// arc cos值域在0~180度
	// 角度需落在-90~90度之間
	// 夾取方位角與兩物體形心的連線垂直，故再減去90度，恰好落在-90~90度之間

	return th;
}




int main()
{
	IplImage* src = cvCreateImage( cvSize( 600, 600), 8, 3);

	/* initialize random seed: */
	srand ( time(NULL) );

//模擬輸入
	vector<Search> vecSearch;
	for( int i = 0; i < 10; i++)
	{
		Search input;
		input.centroid.x = rand()%600;
		input.centroid.y = rand()%600;
		vecSearch.push_back( input);
	}

// 搜尋抓取選擇	
for( int k = 0; k < 10; k++)
{
	
// vecSearch裝一堆物件的坐標值(x,y)
	for( int i = 0; i != vecSearch.size(); i++)
		SearchRadius( vecSearch, i);
// SearchRadius，得到半徑內計數、最近距離、最近點的索引
// count, nearest_dis, nearest_index

//show模擬結果
	for( int i = 0; i != vecSearch.size(); i++)
	{
		cvZero( src);
		cout<<"i:"<<i<<endl;
		cout<<"centroid:("<<vecSearch[i].centroid.x<<","<<vecSearch[i].centroid.y<<")"<<endl;
		cout<<"count:"<<vecSearch[i].count<<endl;
		cout<<"nearest_dis:"<<vecSearch[i].nearest_dis<<endl;
		cout<<"nearest_index:"<<vecSearch[i].nearest_index<<endl;
		cout<<endl;
		
		for( int j = 0; j != vecSearch.size(); j++)
		{
			cvCircle( src, cvPoint( vecSearch[j].centroid.x, vecSearch[j].centroid.y), 4, CV_RGB(255, 255, 255), -1);
			char buf[5];
			sprintf( buf, "%d", j);
			
			cvPutText( src, buf, cvPoint( vecSearch[j].centroid.x+5, vecSearch[j].centroid.y-5), &cvFont(1, 1), CV_RGB( 200, 100, 150));
		}

	}
	
// 半徑內最少count數和最大nearest_dis者先抓
	int index =	MinCountMaxDis( vecSearch);
	cout<<"抓你:"<<index<<endl;

	cvCircle( src, cvPoint( vecSearch[index].centroid.x, vecSearch[index].centroid.y), radius, CV_RGB(255, 255, 0), 3);
	cvShowImage( "src", src);
	cvWaitKey();

// 已抓取者先踢除
	// Remove the element from vector
	vecSearch.erase( vecSearch.begin()+index, vecSearch.begin()+index+1);
	
}

	cvReleaseImage(&src);

	return 0;
}