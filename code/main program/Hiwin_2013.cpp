#include "IMPCard/motorDefine.h"
#include "Robotics/robotics.h"
#include "IMPCard/IMPCard.h"
#include "Control/Control.h"
#include "Grasp/qqveiGrasp.h"
#include "RTX/RtxApp2.h"
#include "Trajectory/Trajectory_Generator.h"
#include "Box/box.h"


#include <conio.h>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

#define SAMPLING_TIME 1;// UNIT: ms

#define HIWIN_3 1

//function declaration
//計時器中斷
void RTFCNDCL TimerHandler();


//Global Variable
float Joint_Factor[USAGE_CHANNELS] = {0.0};
float Motor_Factor[USAGE_CHANNELS] = {0.0};
float TorqueDAC_Factor[USAGE_CHANNELS] = {0.0};
long  ENCounter[USAGE_CHANNELS] = {0};
long  ENCounter_Last[USAGE_CHANNELS] = {0};
float ENCounter_diff[USAGE_CHANNELS] = {0.0};
float JointAngle[USAGE_CHANNELS] = {0.0};
float Torque_damp[USAGE_CHANNELS] = {0.0};
float Torque_ref[USAGE_CHANNELS] = {0.0};
float outV[USAGE_CHANNELS] = {0.0};

float V_th_eps[USAGE_CHANNELS] = {0.0};
float k_DZ[USAGE_CHANNELS] = {0.0};

long ENCounter_HOLDING[USAGE_CHANNELS] = {0 , -(long)(MT1_GEAR_RATIO*MT1_ENCODER_4X / 4) , 0 , (long)(MT3_GEAR_RATIO*MT3_ENCODER_4X / 2) , 0 , 0 , 0};

// 2013.8.8 新增
Trajectory_Generator* ptraj;

const float initial_Pose[7] = { 0, 20, 0, 90, 0, 70, 0};

int COMMAND = 0;
int tracking_count;
float TRACKING_TIME;		// 單位：秒

typedef enum{ HOLDING, TRACKING, TRACKING_new, QUIT = 8}status;

//各關節內插軌跡
rl::math::Cubic< rl::math::Real > interpolator[7];

void holding();
bool tracking(  boost::shared_ptr< rl::kin::Kinematics> kinematics, float x, float y, float z, float th, float time = 5.0);
bool tracking2(  boost::shared_ptr< rl::kin::Kinematics> kinematics, float x, float y, float z, float time = 5.0);
bool tracking3();

void initialing();
void initialing2();
void initialing3();
void byebye();
void homing();
void BallCubeJudgement();


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

typedef struct Box
{
	float x, y, z;
}Box;

Box _Box( float x, float y, float z)
{
	Box box;
	box.x = x;
	box.y = y;
	box.z = z;
	return box;
}

void BoxPosDefinition(Box box[2][4]);

void main()
{

	// RTX periodic timer code
    LARGE_INTEGER  liPeriod;   // timer period
    HANDLE         hTimer  = RtCreateTimer(
								NULL,            // security
								0,               // stack size - 0 uses default
								TimerHandler,    // timer handler
								NULL,            // NULL context (argument to handler)
								RT_PRIORITY_MAX, // priority
								CLOCK_FASTEST) ;     // timer handle
	
		//liPeriod.QuadPart = Tsample*1000*10000;//   10000 = 1ms
	const unsigned int ms = SAMPLING_TIME;
	liPeriod.QuadPart = ms*10000;

	
	if (! (hTimer = RtCreateTimer(
			NULL,            // security
			0,               // stack size - 0 uses default
			TimerHandler,    // timer handler
			NULL,            // NULL context (argument to handler)
			RT_PRIORITY_MAX, // priority
			CLOCK_FASTEST) ))      // RTX HAL timer
	{
			ExitProcess(1);
	}

	Init_IMPCard();

	//夾抓com port設定
	HANDLE _hComm;
	// construct an object of gripper
	int port = 5;
	grasp luoLitaII(port);
	luoLitaII.addObjectInfo("ball",63);
	luoLitaII.addObjectInfo("cube",50.5);
	luoLitaII.addObjectInfo("widerLoose",75);
	

	//讀取手臂組態.xml檔
	boost::shared_ptr< rl::kin::Kinematics > kinematics(rl::kin::Kinematics::create("hiwin6.xml"));

	//定義Base和Table座標系的差
	const float X_t2b = 0.2155f;
	const float Y_t2b = 0.4892f - 0.010;
	const float Z_t2b = -0.218f;

	//定義Kinect和Table座標系的差
	const float X_k2t = 0.327 + 0.05;
	const float Y_k2t = 0.600 ;
	const float Z_k2t = 1.254;

	//定義箱子的位置
	Box box[2][4]; 
	BoxPosDefinition(box);

	// Timer Start
	RtSetTimerRelative(hTimer,&liPeriod,&liPeriod);

	system("pause");

	initialing3();

	// 按鍵開始
	// system("pause");

	//讀取Kinect資料
	ifstream fin("data.txt");
	int size;
	fin>>size;
	Search* pSearch = new Search [size];
	for( int i = 0; i < size; i++)
	{
		fin>>pSearch[i].centroid.x;	
		fin>>pSearch[i].centroid.y;
		fin>>pSearch[i].color;
		fin>>pSearch[i].shape;
		fin>>pSearch[i].size;
		fin>>pSearch[i].count;
		fin>>pSearch[i].nearest_dis;
		fin>>pSearch[i].nearest_index;
		fin>>pSearch[i].orientation;
	}
	
	float th;
	float goal_x, goal_y, goal_z;
	Trajectory_Generator traj(kinematics);
	ptraj = &traj;


#ifdef HIWIN_3
	for( int i = 0; i < size; i++)
	{
		// 點數大於1000，有鄰接，先不考慮
		if( pSearch[i].size > 1000)
			continue;

		// 顏色、形狀
		int color = pSearch[i].color;
		int shape = pSearch[i].shape;

		// 給定xy座標
		float x, y;
		x = pSearch[i].centroid.x;
		y = pSearch[i].centroid.y;


		// 計算角度
		if( !shape)
			th = pSearch[i].orientation;
		else
		{
			th = 0;
		}
		
		goal_x = -y + X_t2b + X_k2t;  //-y + X_t2b + X_k2t;
		goal_y = -x + Y_t2b - Y_k2t;  // x + Y_t2b - Y_k2t;

		ptraj->clear();
		ptraj->addData( 1.5, Tcp6D( goal_x, goal_y, -0.05, -th, 180, 0));
		ptraj->addData( 0.7, Tcp6D( goal_x, goal_y, -0.1425, -th, 180, 0));
		float joint[] = { JointAngle[0], JointAngle[1], JointAngle[3], JointAngle[4], JointAngle[5], JointAngle[6]};

		cout<<"goal_x:"<<goal_x<<" goal_y:"<<goal_y<<endl;

		if( traj.generate(joint))
		{
			cout<<"generate success, satrt tracking"<<endl;
			tracking3();
		}

		// test 
		// system("pause");


		if( shape)
		{
			luoLitaII.graspObject("ball");
			Sleep(800);
		}
		else
		{
			luoLitaII.graspObject("cube");
			Sleep(1000);
		}

		// go to box 
		if( shape)
		{
			ptraj->clear();
			ptraj->addData( 0.4, Tcp6D( goal_x, goal_y, -0.08, 0, 180, 0));
			ptraj->addData( 0.5, Tcp6D( goal_x, goal_y, 0.12, 0, 180, 0));
			
			ptraj->addData( 1.5, Tcp6D( box[shape][color].x, box[shape][color].y, 0.12, 0, 180, 0));
			ptraj->addData( 0.4, Tcp6D( box[shape][color].x, box[shape][color].y, box[shape][color].z, 0, 180, 0));
			joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
			joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];

			if( traj.generate(joint))
			{
				cout<<"generate success, satrt tracking"<<endl;
				tracking3();
			}
			else
			{
				// 無法一段解出，分兩段軌跡
				ptraj->clear();
				ptraj->addData( 0.4, Tcp6D( goal_x, goal_y, -0.08, 0, 180, 0));
				ptraj->addData( 0.5, Tcp6D( goal_x, goal_y, 0.12, 0, 180, 0));
				ptraj->addData( 1.0, Tcp6D( 0.5*(box[shape][color].x+goal_x), 0.5*(box[shape][color].y+goal_y), 0.15, 0, 180, 0));
				joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
				joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];
				if( traj.generate(joint))
				{
					cout<<"generate success, satrt tracking"<<endl;
					tracking3();
				}

				ptraj->clear();
				ptraj->addData( 1.0, Tcp6D( box[shape][color].x, box[shape][color].y, 0.12, 0, 180, 0));
				ptraj->addData( 0.4, Tcp6D( box[shape][color].x, box[shape][color].y, box[shape][color].z, 0, 180, 0));
				joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
				joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];
				if( traj.generate(joint))
				{
					cout<<"generate success, satrt tracking"<<endl;
					tracking3();
				}
			}
		}
		else
		{
			ptraj->clear();
			ptraj->addData( 0.4, Tcp6D( goal_x, goal_y, -0.08, -th, 180, 0));
			ptraj->addData( 0.5, Tcp6D( goal_x, goal_y, 0.12, -th, 180, 0));
			ptraj->addData( 1.2, Tcp6D( 0.4, 0, 0.2, 0, 180, 0));
			joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
			joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];
			
			if( traj.generate(joint))
			{
				cout<<"generate success, satrt tracking"<<endl;
				tracking3();
			}

			ptraj->clear();
			ptraj->addData( 1.2, Tcp6D( 0.4, 0, 0.2, 0, 135, 0));
			ptraj->addData( 1.2, Tcp6D( 0.4, 0, 0.2, 0, 90, 0));
			ptraj->addData( 1.0, Tcp6D( box[shape][color].x-0.05, box[shape][color].y, box[shape][color].z, 0, 90, 0));
			ptraj->addData( 0.8, Tcp6D( box[shape][color].x, box[shape][color].y, box[shape][color].z, 0, 90, 0));
			
			joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
			joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];

			if( traj.generate(joint))
			{	
				cout<<"generate success, satrt tracking"<<endl;
				tracking3();
			}
		}

		// open the gripper
		luoLitaII.looseObject("widerLoose");
		Sleep(800); 

		// 方塊，多一段軌跡，不然ik會有問題
		if( !shape)
		{
			ptraj->clear();
			ptraj->addData( 1.0, Tcp6D( 0.4, 0, 0.2, 0, 90, 0));
			joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
			joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];		
			if( traj.generate(joint))
			{	
				cout<<"generate success, satrt tracking"<<endl;
				tracking3();
			}
		}

		// 回initial pose
		ptraj->clear();
		if( !shape)
			ptraj->addData( 1.2, Tcp6D( 0.4, 0, 0.2, 0, 135, 0));
		ptraj->addData( 1.2, Tcp6D( 0.4, 0, 0.2, 0, 180, 0));
		joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
		joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];	
		if( traj.generate(joint))
		{	
			cout<<"generate success, satrt tracking"<<endl;
			tracking3();
		}

	}

#else
	cout<<"go to box position"<<endl;
	// go to each hole of ball 

	for( int i = 0; i < 4; i++)
	{
			ptraj->clear();
			ptraj->addData( 3.0, Tcp6D( 0.4, 0, 0.2, 0, 180, 0));
			float joint[] = { JointAngle[0], JointAngle[1], JointAngle[3], JointAngle[4], JointAngle[5], JointAngle[6]};
			ptraj->addData( 3.0, Tcp6D( box[1][i].x, box[1][i].y, 0.12, 0, 180, 0));
			ptraj->addData( 1.0, Tcp6D( box[1][i].x, box[1][i].y, box[1][i].z, 0, 180, 0));
			
			// 夾夾夾
			luoLitaII.graspObject("ball");
			Sleep(800);
			
			if( traj.generate(joint))
			{
				cout<<"generate success, satrt tracking"<<endl;
				tracking3();
			}
			
			// 放開
			luoLitaII.looseObject("widerLoose");
			Sleep(1000); 
			system("pause");
	}

	// go to each hole of cube
	ptraj->clear();
	ptraj->addData( 4.0, Tcp6D( 0.4, 0, 0.2, 0, 135, 0));
	ptraj->addData( 2.0, Tcp6D( 0.4, 0, 0.2, 0, 90, 0));
	float joint[] = { JointAngle[0], JointAngle[1], JointAngle[3], JointAngle[4], JointAngle[5], JointAngle[6]};
	if( traj.generate(joint))
	{	
		cout<<"generate success, satrt tracking"<<endl;
		tracking3();
	}

	for( int i = 0; i < 4; i++)
	{
		ptraj->clear();
		ptraj->addData( 2.0, Tcp6D( 0.6, 0, 0.2, 0, 90, 0));
		ptraj->addData( 2.2, Tcp6D( box[0][i].x-0.05, box[0][i].y, box[0][i].z, 0, 90, 0));
		ptraj->addData( 0.8, Tcp6D( box[0][i].x, box[0][i].y, box[0][i].z, 0, 90, 0));
		joint[0] = JointAngle[0];		joint[1] = JointAngle[1];		joint[2] = JointAngle[3];
		joint[3] = JointAngle[4];		joint[4] = JointAngle[5];		joint[5] = JointAngle[6];	
		
		// 夾夾夾
		luoLitaII.graspObject("cube");
		Sleep(800);
				
		if( traj.generate(joint))
		{	
			cout<<"generate success, satrt tracking"<<endl;
			tracking3();
		}

		// 放開
		luoLitaII.looseObject("widerLoose");
		Sleep(1000); 
		system("pause");
	
	}


#endif

	//cout<<"start your test"<<endl;
	//char kbCmd = '~';

	//while(1)
	//{
	//	if(_kbhit())
	//		kbCmd = _getche();
	//	if(kbCmd == 27)	// 27 is 'ESC'
	//		break;

	//	switch(kbCmd)
	//	{
	//	case 'r':
	//		cout<<"ball grasp"<<endl;
	//		luoLitaII.graspObject("ball");
	//		Sleep(1000);
	//		kbCmd = '~';
	//		break;
	//	case 't':
	//		cout<<"ball grasp loose"<<endl;
	//		luoLitaII.looseObject("ball");
	//		kbCmd = '~';
	//		break;
	//	case 'y':
	//		cout<<"ball grasp wide loose"<<endl;
	//		luoLitaII.looseObject("widerLoose");
	//		 kbCmd = '~';
	//		break;


	//	case 'f':
	//		cout<<"cube grasp"<<endl;
	//		luoLitaII.graspObject("cube");
	//		kbCmd = '~';
	//		break;
	//	case 'g':
	//		cout<<"cube grasp loose"<<endl;
	//		luoLitaII.looseObject("cube");
	//		kbCmd = '~';
	//		break;

	//	
	//	}
	//}




	
	


	system("pause");
	


	
	luoLitaII.gripperHoming();


	initialing2();
	byebye();


	RtDeleteTimer( hTimer );

	Close_IMPCard();
	ExitProcess(1);

}

// Timer Interrupt
void RTFCNDCL TimerHandler(   PVOID    context)
{
	// Read Encoder //
	Read_Encoder( ENCounter, JointAngle);
	Differetiator( ENCounter, ENCounter_Last, ENCounter_diff);
	for (int i = 0; i < USAGE_CHANNELS; i++)
	    ENCounter_Last[i] = ENCounter[i];

	// Generate Trajectory
	float time;									
	long ENCounter_ref[USAGE_CHANNELS];			// 期望軌跡命令
	switch(COMMAND)
	{
		// holding，期望軌跡=ENCouter_HOLDING
		case HOLDING:
			for( int i = 0; i < USAGE_CHANNELS; i++)
				ENCounter_ref[i] = ENCounter_HOLDING[i];
			break;
		// tracking，期望軌跡命令與interpolateor和time相關
		case TRACKING:
			time = tracking_count++ / 1000.0 * SAMPLING_TIME;
			for( int i = 0; i < USAGE_CHANNELS; i++)
				ENCounter_ref[i] = interpolator[i].x(time);
			// tracking_count:1000個count/sec
			// 軌跡追蹤結束後holding
			if( tracking_count == (int)(TRACKING_TIME * 1000.0) )
			{
				holding();
				COMMAND = HOLDING;
			}
			break;
		case TRACKING_new:
			time = tracking_count++ / 1000.0 * SAMPLING_TIME;
			ENCounter_ref[0] = ptraj->cubic[0].x(time)*MT0_GEAR_RATIO * MT0_ENCODER_4X / 360.0;
			ENCounter_ref[1] = ptraj->cubic[1].x(time)*MT1_GEAR_RATIO * MT1_ENCODER_4X / 360.0;;
			ENCounter_ref[2] = 0;
			ENCounter_ref[3] = ptraj->cubic[2].x(time)*MT3_GEAR_RATIO * MT3_ENCODER_4X / 360.0;;
			ENCounter_ref[4] = ptraj->cubic[3].x(time)*MT4_GEAR_RATIO * MT4_ENCODER_4X / 360.0;;
			ENCounter_ref[5] = ptraj->cubic[4].x(time)*MT5_GEAR_RATIO * MT5_ENCODER_4X / 360.0;;
			ENCounter_ref[6] = ptraj->cubic[5].x(time)*MT6_GEAR_RATIO * MT6_ENCODER_4X / 360.0;;
			// 軌跡追蹤結束後holding
			if( tracking_count == (int)(TRACKING_TIME * 1000.0) )
			{
				holding();
				COMMAND = HOLDING;
			}
			break;
		
	}

	// Control Law //
	ControlLaw( ENCounter_ref, ENCounter, ENCounter_diff);

	// Output Command //
	Torque_Voltage( Torque_ref, outV);
	OutputDAC( outV);

}

void holding()
{
	// 將目前encoder的counter值存入ENCounter_HOLDING中
	for( int i = 0; i < USAGE_CHANNELS; i++)
		ENCounter_HOLDING[i] = ENCounter[i];
}

bool tracking3()
{
	tracking_count = 1;
	TRACKING_TIME = ptraj->tracking_time;
	COMMAND = TRACKING_new;
	Sleep(TRACKING_TIME*1000);	
	return 1;
}

void homing()
{
	// 回home第一段軌跡
	tracking_count = 1;
	// 到初始位置的時間
	TRACKING_TIME = 4.0f;

	// tacking各關節的count目標值
	int jointTarget[7] = {0};

	// 第二段軌跡 第2軸抬起到45度
	jointTarget[0] = 0;
	jointTarget[1] = (45-90)*rl::math::DEG2RAD*Joint_Factor[1];
	jointTarget[2] = 0;   // 固定軸
	jointTarget[3] = (180-30)*rl::math::DEG2RAD*Joint_Factor[3];
	jointTarget[4] = 0;
	jointTarget[5] = 15*rl::math::DEG2RAD*Joint_Factor[5];
	jointTarget[6] = 0;

	for( int i = 0; i < USAGE_CHANNELS; i++)
 		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);

	// 開始tracking
	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);

	// 回home第二段軌跡
	tracking_count = 1;
	// 到初始位置的時間
	TRACKING_TIME = 5.0f;

	// 第2軸抬起到-90+0.5度，第4軸轉動到180-0.12度
	jointTarget[0] = 0;
	jointTarget[1] = (0.5-90)*rl::math::DEG2RAD*Joint_Factor[1];
	jointTarget[2] = 0;   // 固定軸
	jointTarget[3] = (180-0.12)*rl::math::DEG2RAD*Joint_Factor[3];
	jointTarget[4] = 0;
	jointTarget[5] = 0;
	jointTarget[6] = 0;

	for( int i = 0; i < USAGE_CHANNELS; i++)
 		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);
	
	// 開始tracking
	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);
}

// initialing2 + initialing3:
// go to initial pose, just like the following figure
//     /\
//    /  \
//   /  |-|
// _/   [ ]
//| |
// go to the following angle :
// q_ready1 << 0.0f, -pi/4.0f,  0.0f,  5.0f*pi/6.0f,  0.0f, -pi/12.0f,  0.0f;
void initialing2()
{
	tracking_count = 1;
	// 到初始位置的時間
	TRACKING_TIME = 4.0f;

	// tacking各關節的count目標值
	int jointTarget[7] = {0};

	// 第二段軌跡 第2軸抬起到45度
	jointTarget[0] = 0;
	jointTarget[1] = (45-90)*rl::math::DEG2RAD*Joint_Factor[1];
	jointTarget[2] = 0;   // 固定軸
	jointTarget[3] = (180-30)*rl::math::DEG2RAD*Joint_Factor[3];
	jointTarget[4] = 0;
	jointTarget[5] = 15*rl::math::DEG2RAD*Joint_Factor[5];
	jointTarget[6] = 0;

	for( int i = 0; i < USAGE_CHANNELS; i++)
 		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);

	// 開始tracking
	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);
}



// go to the following angle :
// const float initial_Pose[7] = { 0, 20, 0, 90, 0, 70, 0};
void initialing3()
{
	tracking_count = 1;
	// 到初始位置的時間
	TRACKING_TIME = 5.0f;

	// tacking各關節的count目標值
	int jointTarget[7] = {0};

	// 第三段軌跡 第2軸抬起到45度 第4軸到-90度 第6軸到70度
	jointTarget[0] = 0;
	jointTarget[1] = 20.0f*rl::math::DEG2RAD*Joint_Factor[1];
	jointTarget[2] = 0;   // 固定軸
	jointTarget[3] = 90.0f*rl::math::DEG2RAD*Joint_Factor[3];
	jointTarget[4] = 0;
	jointTarget[5] = 70.0f*rl::math::DEG2RAD*Joint_Factor[5];
	jointTarget[6] = 0;

	for( int i = 0; i < USAGE_CHANNELS; i++)
 		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);

	// 開始tracking
	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);
}

// this joint position is the same as the program written by yhtsai
void byebye()
{
	tracking_count = 1;
	// 到初始位置的時間
	TRACKING_TIME = 5.0f;

	// tacking各關節的count目標值
	int jointTarget[7] = {0};

	// 第2軸抬起到-90+0.5度，第4軸轉動到180-0.12度
	jointTarget[0] = 0;
	jointTarget[1] = (0.5-90)*rl::math::DEG2RAD*Joint_Factor[1];
	jointTarget[2] = 0;   // 固定軸
	jointTarget[3] = (180-0.12)*rl::math::DEG2RAD*Joint_Factor[3];
	jointTarget[4] = 0;
	jointTarget[5] = 0;
	jointTarget[6] = 0;

	for( int i = 0; i < USAGE_CHANNELS; i++)
 		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);
	
	// 開始tracking
	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);
}

bool tracking2(  boost::shared_ptr< rl::kin::Kinematics> kinematics, float x, float y, float z, float time)
{
	tracking_count = 1;
	TRACKING_TIME = time;

	//內插次數設定
	const float interpolate_times = 100;
	
	// update kinemctics 角度資訊，單位：rad
	rl::math::Vector q(kinematics->getDof());
	q(0) = ENCounter[0] * Motor_Factor[0];
	q(1) = ENCounter[1] * Motor_Factor[1];
	q(2) = ENCounter[3] * Motor_Factor[3];
	q(3) = ENCounter[4] * Motor_Factor[4];
	q(4) = ENCounter[5] * Motor_Factor[5];
	q(5) = ENCounter[6] * Motor_Factor[6];

	//正向運動學計算
	kinematics->setPosition(q);
	kinematics->updateFrames();	
	
	// 目前end-effector的位置
	const rl::math::Transform::ConstTranslationPart& position = kinematics->forwardPosition().translation();
	const float xi = position.x();
	const float yi = position.y();
	const float zi = position.z();

	// 內插的每個step
	float step[3];
	step[0] = (x - xi) / interpolate_times;
	step[1] = (y - yi) / interpolate_times;
	step[2] = (z - zi) / interpolate_times;

	// 解IK的initial value，設定為目前encoder的值，單位：degree
	float joint[USAGE_CHANNELS-1] = {0};
	joint[0] = ENCounter[0]*Motor_Factor[0]/REV_RAD*360;
	joint[1] = ENCounter[1]*Motor_Factor[1]/REV_RAD*360;
	joint[2] = ENCounter[3]*Motor_Factor[3]/REV_RAD*360;
	joint[3] = ENCounter[4]*Motor_Factor[4]/REV_RAD*360;
	joint[4] = ENCounter[5]*Motor_Factor[5]/REV_RAD*360;
	joint[5] = ENCounter[6]*Motor_Factor[6]/REV_RAD*360;
	
	for( int i = 0; i < interpolate_times; i++)
	{
		float xp = xi + step[0]*(i+1);
		float yp = yi + step[1]*(i+1);
		float zp = zi + step[2]*(i+1);
		
		float transform12D[12] = { 0, 0, -1, 0, 1, 0, 1, 0, 0, xp, yp, zp};
		
		// 反向運動學，無解時跳出
		if ( !InverseKinematics12( kinematics, transform12D, joint))
		{
			COMMAND = QUIT;
			return 0;
		}
		
		//IK返回結果單位為rad，轉成degree
		for( int j = 0; j < USAGE_CHANNELS - 1; j++)
			joint[j] *= rl::math::RAD2DEG;
	}
		
	// tacking各關節的count目標值，由角度轉成count值
	int jointTarget[7];
	jointTarget[0] = joint[0] * MT0_GEAR_RATIO * MT0_ENCODER_4X / 360.0;
	jointTarget[1] = joint[1] * MT1_GEAR_RATIO * MT1_ENCODER_4X / 360.0;
	jointTarget[2] = 0; 
	jointTarget[3] = joint[2] * MT3_GEAR_RATIO * MT3_ENCODER_4X / 360.0;
	jointTarget[4] = joint[3] * MT4_GEAR_RATIO * MT4_ENCODER_4X / 360.0;
	jointTarget[5] = joint[4] * MT5_GEAR_RATIO * MT5_ENCODER_4X / 360.0;
	jointTarget[6] = joint[5] * MT6_GEAR_RATIO * MT6_ENCODER_4X / 360.0;

	
	for( int i = 0; i < USAGE_CHANNELS; i++)
		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);

	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);
	return 1;
}



bool tracking(  boost::shared_ptr< rl::kin::Kinematics> kinematics, float x, float y, float z, float th, float time)
{
	tracking_count = 1;
	TRACKING_TIME = time;

	//內插次數設定
	const float interpolate_times = 10;
	
	// update kinemctics 角度資訊，單位：rad
	rl::math::Vector q(kinematics->getDof());
	q(0) = ENCounter[0] * Motor_Factor[0];
	q(1) = ENCounter[1] * Motor_Factor[1];
	q(2) = ENCounter[3] * Motor_Factor[3];
	q(3) = ENCounter[4] * Motor_Factor[4];
	q(4) = ENCounter[5] * Motor_Factor[5];
	q(5) = ENCounter[6] * Motor_Factor[6];

	//正向運動學計算
	kinematics->setPosition(q);
	kinematics->updateFrames();	
	
	// 目前end-effector的位置
	const rl::math::Transform::ConstTranslationPart& position = kinematics->forwardPosition().translation();
	const float xi = position.x();
	const float yi = position.y();
	const float zi = position.z();

	// 內插的每個step
	float step[3];
	step[0] = (x - xi) / interpolate_times;
	step[1] = (y - yi) / interpolate_times;
	step[2] = (z - zi) / interpolate_times;

	// 解IK的initial value，設定為目前encoder的值，單位：degree
	float joint[USAGE_CHANNELS-1] = {0};
	joint[0] = ENCounter[0]*Motor_Factor[0]/REV_RAD*360;
	joint[1] = ENCounter[1]*Motor_Factor[1]/REV_RAD*360;
	joint[2] = ENCounter[3]*Motor_Factor[3]/REV_RAD*360;
	joint[3] = ENCounter[4]*Motor_Factor[4]/REV_RAD*360;
	joint[4] = ENCounter[5]*Motor_Factor[5]/REV_RAD*360;
	joint[5] = ENCounter[6]*Motor_Factor[6]/REV_RAD*360;
	
	float r1 = cos(3.14159*th/180);
	float r2 = sin(3.14159*th/180);

	for( int i = 0; i < interpolate_times; i++)
	{
		float xp = xi + step[0]*(i+1);
		float yp = yi + step[1]*(i+1);
		float zp = zi + step[2]*(i+1);
		
		float transform12D[12] = {-r1, r2, 0, r2, r1, 0, 0, 0, -1, xp, yp, zp};
		
		// 反向運動學，無解時跳出
		if ( !InverseKinematics12( kinematics, transform12D, joint))
		{
			COMMAND = QUIT;
			return 0;
		}
		
		//IK返回結果單位為rad，轉成degree
		for( int j = 0; j < USAGE_CHANNELS - 1; j++)
			joint[j] *= rl::math::RAD2DEG;
	}
		
	// tacking各關節的count目標值，由角度轉成count值
	int jointTarget[7];
	jointTarget[0] = joint[0] * MT0_GEAR_RATIO * MT0_ENCODER_4X / 360.0;
	jointTarget[1] = joint[1] * MT1_GEAR_RATIO * MT1_ENCODER_4X / 360.0;
	jointTarget[2] = 0; 
	jointTarget[3] = joint[2] * MT3_GEAR_RATIO * MT3_ENCODER_4X / 360.0;
	jointTarget[4] = joint[3] * MT4_GEAR_RATIO * MT4_ENCODER_4X / 360.0;
	jointTarget[5] = joint[4] * MT5_GEAR_RATIO * MT5_ENCODER_4X / 360.0;
	jointTarget[6] = joint[5] * MT6_GEAR_RATIO * MT6_ENCODER_4X / 360.0;

	
	for( int i = 0; i < USAGE_CHANNELS; i++)
		pathSetting( interpolator[i], TRACKING_TIME, ENCounter[i], jointTarget[i]);

	COMMAND = TRACKING;
	Sleep(TRACKING_TIME*1000);
	return 1;
}

void BoxPosDefinition(Box box[2][4])
{
	// box[i][j] :  i:1->球  0->方塊  (形狀)
	//              j:0->紅  1->綠  2->藍  3->黃
	ifstream fin("HolePosData.txt");
	int size;
	fin>>size;	
	
	// 紅綠藍黃球
	for( int i = 0; i < 4; i++)
	{
		fin>>box[1][i].x;
		fin>>box[1][i].y;
		box[1][i].z = 0.08;
	}

	// 紅綠藍黃方塊
	for( int i = 0; i < 4; i++)
	{
		fin>>box[0][i].x;
		fin>>box[0][i].y;
		box[0][i].z = 0.13;
	}
	fin.close();
}