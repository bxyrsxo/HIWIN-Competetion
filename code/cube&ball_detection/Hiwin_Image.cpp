#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <cmath>
#define M_PI 3.14159
#define  ALL  1
using namespace std;

typedef struct SecMoment
{
     double xx;
     double yy;
     double xy;   
}SecMoment;

// initial the Secmoment member
inline SecMoment secMoment( double xx, double yy, double xy)
{
       SecMoment moment;
       moment.xx = xx;
       moment.yy = yy;
       moment.xy = xy;
       return moment;
}

//---------------------------------------------------------------------------

// Count non zero pixel number
int rsCountNonZero(IplImage* src)
{
     int i, j;       // for loop
     int count = 0;
     for( i = 0; i < src->height; i++)
          for( j = 0; j < src->width; j++)
               if( ((uchar*)(src->imageData + src->widthStep*i))[j])
                     count++; 
     return count;
}

// Restriction: Input Image must be a binary image.
// Centroid
CvPoint2D32f rsCentroid(IplImage* src)
{
    int i, j;  
    CvPoint2D32f pt = cvPoint2D32f(0, 0);
    int pt_num = rsCountNonZero(src);
    for( i = 0 ; i < src->height; i++)
         for( j = 0; j < src->width; j++)
         {
              if( ((uchar*)(src->imageData + src->widthStep*i))[j])
              {
                     pt.x += j; 
                     pt.y += i;
              }              
         }         
    pt.x /= pt_num;                
    pt.y /= pt_num;        
    
    return pt; 
}

// Calculate the second moment 
SecMoment rsSecondMoment(IplImage* src)
{
    int i, j;  
    SecMoment moment = secMoment(0, 0, 0);
    int pt_num = rsCountNonZero(src);
    for( i = 0 ; i < src->height; i++)
         for( j = 0; j < src->width; j++)
         {
              if( ((uchar*)(src->imageData + src->widthStep*i))[j])
              {
                     moment.xx += j*j; 
                     moment.yy += i*i;
                     moment.xy += i*j;
              }              
         }         

    moment.xx /= pt_num; 
    moment.yy /= pt_num;
    moment.xy /= pt_num;

    return moment; 
}

// Calulate the component orientation
// return the long axis angle
double rsOrientation( IplImage* src, CvPoint2D32f centroid)
{
    int i, j;
    double x, y;
    double a = 0, b = 0, c = 0;
    double long_axis_angle;

    for( i = 0 ; i < src->height; i++)
         for( j = 0; j < src->width; j++)
         {
              if( ((uchar*)(src->imageData + src->widthStep*i))[j])
              {
                    x = j - centroid.x;
                    y = i - centroid.y;
                    a += x*x;
                    b += 2*x*y;
                    c += y*y;
              }              
         }
    // Judge the long or short axis
    // if a-c = 0, the component is symmety of both x and y axes
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

    return long_axis_angle; 
}

// Calulate the component elongation
double rsElongation( IplImage* src, CvPoint2D32f centroid)
{
    int i, j;
    double x, y;
    double a = 0, b = 0, c = 0;
    double long_angle, short_angle;
    double long_axis, short_axis;
    double max, min;

    for( i = 0 ; i < src->height; i++)
         for( j = 0; j < src->width; j++)
         {
              if( ((uchar*)(src->imageData + src->widthStep*i))[j])
              {
                    x = j - centroid.x;
                    y = i - centroid.y;
                    a += x*x;
                    b += 2*x*y;
                    c += y*y;
              }              
         }
    if( (a-c) == 0 )
    {
        long_angle = 0;
        short_angle = -90;    
    }
    else
    {
        // Calculate the two times derivative of the X^2
        long_angle = atan( (a-c)/b );
        // if > 0, have the maximum value
        // if < 0, have the minimun value
        if(long_angle > 0)
        {
            long_angle = 0.5*atan(b/(a-c))/M_PI*180; 
            short_angle = long_angle - 90;
        }
        if(long_angle < 0)
        {
            short_angle = 0.5*atan(b/(a-c))/M_PI*180; 
            long_angle = short_angle + 90;
        }
    }               
    
    long_axis  = a*cos(long_angle)*cos(long_angle) 
                 + b*sin(long_angle)*cos(long_angle)
                 + c*sin(long_angle)*sin(long_angle); 
    short_axis = a*cos(short_angle)*cos(short_angle) 
                 + b*sin(short_angle)*cos(short_angle)
                 + c*sin(short_angle)*sin(short_angle); 

    if( long_axis > short_axis)
    {
        max = long_axis;
        min = short_axis;
    }
    else
    {
        max = short_axis;
        min = long_axis;
    }
    return sqrt(max)/sqrt(min);
}


// Return the contour's perimeter
// Definition : The shortest (Pixels)
int rsPerimeter(CvSeq* contour)
{
    return contour->total;
}

int rsCircleDetect( IplImage* src)
{
	CvMemStorage* storage = cvCreateMemStorage(0);
//	cvSmooth( src, dst, CV_GAUSSIAN, 5, 5 );  //降噪  
    CvSeq* results = cvHoughCircles(  //cvHoughCircles函数需要估计每一个像素梯度的方向，  
                                      //因此会在内部自动调用cvSobel,而二值边缘图像的处理是比较难的  
        src,  
        storage,  
        CV_HOUGH_GRADIENT,  
        3,  //累加器图像的分辨率  
        src->width/10  
        );  

	// return value: 1->circle; 0->rectangle
	if( results->total)
		return 1;
	else
		return 0;
}

int rsColorDetect( IplImage* src, IplImage* mask)
{
	int r, g, b;
	int count = 0;
	float avg_r = 0, avg_g = 0, avg_b = 0;
	for( int i = 0; i < src->height; i++)
		for( int j = 0; j < src->width; j++)
		{
			if( ((unsigned char)mask->imageData[i*mask->widthStep+j] ))	
			{
				float tmp_r, tmp_g, tmp_b;
				tmp_r = ((unsigned char)src->imageData[i*src->widthStep+3*j+2])/255.0;
				tmp_g = ((unsigned char)src->imageData[i*src->widthStep+3*j+1])/255.0;
				tmp_b = ((unsigned char)src->imageData[i*src->widthStep+3*j  ])/255.0;

				avg_r += tmp_r;
				avg_g += tmp_g;
				avg_b += tmp_b;
				count++;
			}
		}
	avg_r /= count;
	avg_g /= count;
	avg_b /= count;

	//cout<<"r:"<<avg_r<<endl;
	//cout<<"g:"<<avg_g<<endl;
	//cout<<"b:"<<avg_b<<endl;

	if( avg_r < 0.1)
		r = 0;
	else
		r = 1;
	if( avg_g < 0.1)
		g = 0;
	else 
		g = 1;
	if( avg_b < 0.1)
		b = 0;
	else
		b = 1;

	// return value: 0->red, 1->grenn, 2->blue, 3->yellow, 4->undefined
	if( r == 0 && b == 0)
		return 1;
	else if( g == 0 && b == 0)
		return 0;
	else if( !b)
		return 3;
	else if( !r)
		return 2;
	else
		return 4;
}

// Calculate Each Component Property
void EachComponent(IplImage* src, IplImage* dst, CvSeq* contour)
{
    int i;          // for loop
    cvNamedWindow("Component", 1);
    cvZero(dst);

    CvPoint *Point = new CvPoint [contour->total];
    CvSeqReader reader;
    cvStartReadSeq( contour, &reader, 0 );

    for( i = 0; i < contour->total; i++ )
    {
        CV_READ_SEQ_ELEM( Point[i], reader);
        ((uchar*)(dst->imageData + dst->widthStep*Point[i].y))[Point[i].x] = 255;       
    }

    static int num = 1;   
    int count;
	int shape;
	int color;
    double orientation, elongation;
    CvPoint2D32f pt;
    SecMoment moment;
    // fill the color in the contour
    cvFloodFill( dst, cvPoint( Point[0].x+2 , Point[0].y+2), cvScalarAll(255), cvRealScalar(0), cvRealScalar(0), NULL, 4 );
    count = rsCountNonZero(dst);
    pt    = rsCentroid(dst);
    moment= rsSecondMoment(dst);
    orientation = rsOrientation(dst, pt);
    elongation  = rsElongation(dst, pt);
	shape = rsCircleDetect( dst);
	color = rsColorDetect( src, dst);

	// Display the Component property
    cout<<"Component "<<num<<endl;
	// cout<<"Size:"<<count<<endl;
    cout<<"Centroid: ("<<pt.x<<","<<pt.y<<")"<<endl;

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


    // cout<<"Perimeter:"<<rsPerimeter(contour)<<endl;
    // cout<<"Ixx:"<<moment.xx<<endl;
    // cout<<"Iyy:"<<moment.yy<<endl;
    // cout<<"Ixy:"<<moment.xy<<endl;
    // cout<<"Orientation:"<<orientation<<endl;
    // cout<<"Elongation:"<<elongation<<endl;

	// 繪圖
	IplImage* show = cvCreateImage( cvGetSize( src), 8, 3);
	for( int i = 0; i < src->height; i++)
		for( int j = 0; j < src->width; j++)
		{
			if( ((unsigned char) dst->imageData[i*dst->widthStep+j]))
			{
				show->imageData[i*show->widthStep+3*j  ] = src->imageData[i*src->widthStep+3*j  ];			
				show->imageData[i*show->widthStep+3*j+1] = src->imageData[i*src->widthStep+3*j+1];
				show->imageData[i*show->widthStep+3*j+2] = src->imageData[i*src->widthStep+3*j+2];
			}
			else
			{
				show->imageData[i*show->widthStep+3*j  ] = 0;			
				show->imageData[i*show->widthStep+3*j+1] = 0;
				show->imageData[i*show->widthStep+3*j+2] = 0;
			}
		}

	// 標示形心
	cvCircle( show, cvPoint( (int)pt.x , (int)pt.y), 5, CV_RGB(255, 255, 255), -1 );
	

	cvShowImage("Component", show);
    num++;
    cvWaitKey();
    delete [] Point;    
	cvReleaseImage( &show);
}

int main()
{
	IplImage* src = cvLoadImage( "Test1.jpg", 1);
	cvShowImage( "win", src);
	cvWaitKey();

	IplImage* gray = cvCreateImage( cvGetSize(src), 8, 1);
	cvCvtColor( src, gray, CV_RGB2GRAY);

	IplImage* red = cvCreateImage( cvGetSize(src), 8, 1);
	IplImage* blue = cvCreateImage( cvGetSize(src), 8, 1);
	IplImage* green = cvCreateImage( cvGetSize(src), 8, 1);

	IplImage* blue_filtered = cvCreateImage( cvGetSize( src), 8, 1);
	IplImage* red_filtered  = cvCreateImage( cvGetSize( src), 8, 1);
	IplImage* green_filtered  = cvCreateImage( cvGetSize( src), 8, 1);

	cvSplit( src, blue, green, red, NULL);

	// 藍色資訊
	for( int i = 0; i < blue->height; i++)
		for( int j = 0; j < blue-> width ; j++)
		{
			if( ((unsigned char)blue->imageData[i*blue->widthStep+j]) < 50)
				blue_filtered->imageData[i*blue_filtered->widthStep+j] = 255;
			else
				blue_filtered->imageData[i*blue_filtered->widthStep+j] = 0;
		}

	// 紅色資訊
	for( int i = 0; i < red->height; i++)
		for( int j = 0; j < red-> width ; j++)
		{
			if( ((unsigned char)red->imageData[i*red->widthStep+j]) < 60)
				red_filtered->imageData[i*red_filtered->widthStep+j] = 255;
			else
				red_filtered->imageData[i*red_filtered->widthStep+j] = 0;
		}


	// 綠色資訊
	for( int i = 0; i < green->height; i++)
		for( int j = 0; j < green-> width ; j++)
		{
			if( ((unsigned char)green->imageData[i*green->widthStep+j]) < 120)
				green_filtered->imageData[i*green_filtered->widthStep+j] = 255;
			else
				green_filtered->imageData[i*green_filtered->widthStep+j] = 0;
		}
		
	// 合成結果
	IplImage* result = cvCreateImage( cvGetSize( src), 8, 1);

	cvOr( red_filtered, blue_filtered, result);
	cvWaitKey();

	cvShowImage("result", result);
	cvWaitKey();

	// 形態學處理
	cvDilate( result, result, NULL, 3);
	cvErode( result, result, NULL, 3);

	cvShowImage("result", result);
	cvWaitKey();

	// 尋找contour
	CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* contour;
    cvFindContours(result, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
//    IplImage* dst = cvCreateImage(cvSize(result->width,result->height), IPL_DEPTH_8U, 3); 
//    cvCvtColor( result, dst, CV_GRAY2BGR);


	IplImage* img = cvCreateImage( cvSize(src->width, src->height), 8, 1); 
    EachComponent( src, img, contour);

#ifdef ALL
    while(contour->h_next)
    {
          contour = contour->h_next;                
          EachComponent( src, img, contour);
    }
#endif


	cvReleaseImage( &src);
	cvReleaseImage( &img);
	cvReleaseImage( &result);
	cvReleaseImage( &gray);
	cvReleaseImage( &red);
	cvReleaseImage( &blue);
	cvReleaseImage( &green);
	cvReleaseImage( &blue_filtered);
	cvReleaseImage( &red_filtered);
	cvReleaseImage( &green_filtered);
	cvReleaseImage( &result);


	return 0;
}