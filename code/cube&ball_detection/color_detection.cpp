#include "stdafx.h"
#include "cv.h"
#include <XnOpenNI.h>
#include "highgui.h"
#include <cxcore.h>
#include <stdio.h>
#include <math.h>
#include <conio.h>
#include <windows.h>
#include<cstdlib>
#include <process.h>
#include <time.h>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
using namespace std;
using namespace cv;
#define PI 3.1415926535898

#define NI_CHECK_ERROR(status , message) \
	if (status != XN_STATUS_OK) { \
		xnPrintError(status, message); \
		exit(EXIT_FAILURE); \
		}

int width;
int height;
int Ymin;
int Ymax;
int Crmin;
int Crmax;
int Cbmin;
int Cbmax;

int file_idx = 0;
char filename[256];
char fileidx[10];


void onYmin(int position)
{   
    Ymin=position;
}
void onYmax(int position)
{
   Ymax=position;
}
void onCrmin(int position)
{	
    Crmin=position;
}
void onCrmax(int position)
{
     Crmax=position;
}
void onCbmin(int position)
{
    Cbmin=position;
}
void onCbmax(int position)
{
   
    Cbmax=position;
}
void ColorFilterYCrCb(IplImage *_frame_copy, IplImage* binary)
{
	
	cvCvtColor(_frame_copy,_frame_copy,CV_BGR2YCrCb);

	uchar* data=(uchar*)_frame_copy->imageData;
	uchar* data2=(uchar*)binary->imageData;
	
	//HUV
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
				
			    float Y,Cr,Cb;
				Y=(data[i*3*width+3*j]);
				Cr=(data[i*3*width+3*j+1]);
				Cb=(data[i*3*width+3*j+2]);
				
				
			if(Y>Ymin &&  Y<Ymax )
			{
				 if(Cr>Crmin && Cr<Crmax)
				 {
						if(Cb>Cbmin && Cb<Cbmax)
						{
							data2[i*width+j]=255;
						}
						else
						{
							data2[i*width+j]=0;
						}
				 }
				 else
				 {
					data2[i*width+j]=0;		
				 }
			}
				
			else
			{
				data2[i*width+j]=0;
			}

		
		}
	}
	
}

int main(int, char**)
{
	// initial kinect
	/* context initialisation */
	XnStatus status;
	XnContext *context;
	status = xnInit(&context);
	NI_CHECK_ERROR(status, "OpenNI Initialisation ");    

	/* creation of a context of a depth images generator and a context of a color images generator */
	XnNodeHandle depthGenerator, imageGenerator;
	status = xnCreateDepthGenerator(context, &depthGenerator, NULL, NULL);
	NI_CHECK_ERROR(status, "Creation of a context of a depth images generator");
	status = xnCreateImageGenerator(context, &imageGenerator, NULL, NULL);
	NI_CHECK_ERROR(status, "Creation of a context of a color images generator");

	/* fit the depth image with the color image */
	if (! xnIsCapabilitySupported(depthGenerator, XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
	{
		fprintf(stderr, "Can not change the depth images point of view \n");
		exit(EXIT_FAILURE);
	}
	else
		xnSetViewPoint(depthGenerator, imageGenerator);
	
	/* starting generators */
	xnStartGeneratingAll(context);
	
	/* change OpenNI images to OpenCV images */
	XnImageMetaData *imageMetaData = xnAllocateImageMetaData();
	XnDepthMetaData *depthMetaData = xnAllocateDepthMetaData();
	xnGetImageMetaData(imageGenerator, imageMetaData);
	IplImage *depth = cvCreateImage(cvSize(depthMetaData->pMap->FullRes.X, depthMetaData->pMap->FullRes.Y), IPL_DEPTH_16U, 1);
	
	IplImage* image = cvCreateImage( cvSize(imageMetaData->pMap->FullRes.X, imageMetaData->pMap->FullRes.Y),IPL_DEPTH_8U, 3);
	IplImage* gray  = cvCreateImage( cvSize( image->width, image->height), 8, 1);
	//uint16_t maxDepth = depthMetaData->nZRes;
	xnFreeDepthMetaData (depthMetaData);
	xnFreeImageMetaData (imageMetaData);

	int key;
	fstream file_red, file_yellow, file_blue, file_green;
	
	IplImage* image_copy = cvCreateImage( cvSize(image->width,image->height),IPL_DEPTH_8U,image->nChannels );
	IplImage* binary = cvCreateImage( cvSize(image->width,image->height),IPL_DEPTH_8U,1);

	while(1)
	{
		// query a frame from kinect
		const XnRGB24Pixel* pImageMap = xnGetRGB24ImageMap( imageGenerator);
		int total_pixel = image->height*image->width;
		for( int j = 0; j < total_pixel; j++)
		{
			image->imageData[3*j  ] = pImageMap[j].nBlue;
			image->imageData[3*j+1] = pImageMap[j].nGreen;
			image->imageData[3*j+2] = pImageMap[j].nRed;
		}

		key = cvWaitKey( 30 );

		cvCopy( image, image_copy, 0 );
	
		cvCreateTrackbar("Y min","result",&Ymin,255,onYmin);
		cvCreateTrackbar("Y max","result",&Ymax,255,onYmax);
		cvCreateTrackbar("Cr min","result",&Crmin,255,onCrmin);
		cvCreateTrackbar("Cr max","result",&Crmax,255,onCrmax);
		cvCreateTrackbar("Cb min","result",&Cbmin,255,onCbmin);
		cvCreateTrackbar("Cb max","result",&Cbmax,255,onCbmax);

		height = image->height;
		width  = image->width;
		ColorFilterYCrCb( image, binary);
		cvShowImage( "result", binary);
		//cvShowImage( "original", image);
		cvShowImage( "ycbcr", image_copy);
		if(key == 27) break;
			 
		if(key =='q' || key=='Q' )
		{  
			file_red.open("red.txt", ios::out | ios::trunc);
			file_red<<Ymin<<endl;
			file_red<<Ymax<<endl;
			file_red<<Crmin<<endl;
			file_red<<Crmax<<endl;
			file_red<<Cbmin<<endl;
			file_red<<Cbmax<<endl;
			file_red.close();
		}
	
		if(key =='w' || key=='W' )
		{ 
			file_green.open("green.txt", ios::out | ios::trunc);
			file_green<<Ymin<<endl;
			file_green<<Ymax<<endl;
			file_green<<Crmin<<endl;
			file_green<<Crmax<<endl;
			file_green<<Cbmin<<endl;
			file_green<<Cbmax<<endl;
			file_green.close();
		}

		if(key =='e' || key=='E' )
		{
			file_blue.open("blue.txt", ios::out | ios::trunc);
			file_blue<<Ymin<<endl;
			file_blue<<Ymax<<endl;
			file_blue<<Crmin<<endl;
			file_blue<<Crmax<<endl;
			file_blue<<Cbmin<<endl;
			file_blue<<Cbmax<<endl;
			file_blue.close();
		}

		if(key =='r' || key=='R' )
		{  
			file_yellow.open("yellow.txt", ios::out | ios::trunc);
			file_yellow<<Ymin<<endl;
			file_yellow<<Ymax<<endl;
			file_yellow<<Crmin<<endl;
			file_yellow<<Crmax<<endl;
			file_yellow<<Cbmin<<endl;
			file_yellow<<Cbmax<<endl;
			file_yellow.close();
		}

		if(key == '1')
		{
			file_red.open("red.txt", ios::in);
			file_red>>Ymin;
			file_red>>Ymax;
			file_red>>Crmin;
			file_red>>Crmax;
			file_red>>Cbmin;
			file_red>>Cbmax;
			file_red.close();
		}

		if(key == '2')
		{
			file_green.open("green.txt", ios::in);
			file_green>>Ymin;
			file_green>>Ymax;
			file_green>>Crmin;
			file_green>>Crmax;
			file_green>>Cbmin;
			file_green>>Cbmax;
			file_green.close();
		}

		if(key == '3')
		{
			file_blue.open("blue.txt", ios::in);
			file_blue>>Ymin;
			file_blue>>Ymax;
			file_blue>>Crmin;
			file_blue>>Crmax;
			file_blue>>Cbmin;
			file_blue>>Cbmax;
			file_blue.close();
		}

		if(key == '4')
		{
			file_yellow.open("yellow.txt", ios::in);
			file_yellow>>Ymin;
			file_yellow>>Ymax;
			file_yellow>>Crmin;
			file_yellow>>Crmax;
			file_yellow>>Cbmin;
			file_yellow>>Cbmax;
			file_yellow.close();
		}

             
	}
	cvReleaseImage( &image_copy );
	cvReleaseImage( &binary);
	cvReleaseImage( &image );
	cvDestroyAllWindows();
	xnContextRelease(context);
	
	return 0;
}