#include"Countdown.h"
#include<iostream>
#include<string>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <ctime>
#include <windows.h>
#include <MMSystem.h>
//OpenCV Header
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\imgproc.hpp>
#include<opencv2\video\background_segm.hpp>
#pragma comment(lib, "winmm.lib")
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

//using namespace cv;
using namespace std;
// Kinect for Windows SDK Header

#define THRESHOLD_X 0.3
#define THRESHOLD_Y 0.5

class System{
    private:
		double lockX[JointType_Count] = { 0, 0, 0, 0, -0.7,
										 -0.96, -1, -0.83, 0.62, 1,
										 0.97, 0.82, -0.33, -0.31, -0.46,
										 -0.54, 0.32, 0.32, 0.31, 0.31,
										 -0.03, -0.77, -0.74, 0.8, 0.69 };
		double lockY[JointType_Count] = { -0.13, -0.5, -0.83, -1, -0.69,
										 -0.46, -0.29, -0.25, -0.7, -0.49,
										 -0.31, -0.28, -0.13, 0.29, 0.82,
										 0.97, -0.13, 0.28, 0.85, 1,
										 -0.75, -0.16, -0.31, -0.18, -0.3 }; // 比一個插腰的姿勢
		clock_t start, check, try_unlock_start, try_unlock_end, unlocked_start, unlocked_end, rec_unlocked_end;
		bool isStart, isResampleStart , isTryStart, isUnlockedStart, new_lock_is_set, from_unlock_to_lock;
		int mode;
		IKinectSensor   * mySensor;
		int myBodyCount;
		IBodyFrameSource    * myBodySource ;//一個IBodyFrameSource(身體數據源)
		IBodyFrameReader    * myBodyReader;//一個IBodyFrameReader(身體閱讀器)
		IColorFrameSource    * myColorSource ;//一個IColorFrameSource(色彩數據源)
		IColorFrameReader    * myColorReader;//一個IColorFrameReader(色彩閱讀器)
		int width, height ;
		IFrameDescription   * myColorDescription;
		IBodyFrame  * myBodyFrame ;
		IColorFrame  * myColorFrame;
		ICoordinateMapper* myCoordinateMapper;
        bool IS_LOCKED;
        bool IS_STAY_SYSTEM;
       std::string command;
       enum System_tip{
         LOCKED,
         UNLOCKED,
         SET
       };//define system message
       enum command_tip{
         SetInitialCode,
         VaildCode,
         DeleteCode
       };
     // Time System_time;
	  std::mutex SystemExetx;
	  std::mutex CommandExetx;
	  std::mutex MainExetx;
	  std::condition_variable SystemExecv;
	  std::condition_variable CommandExecv;
	  std::condition_variable MainExecv;
	  bool command_num;
    public:
      System();
      void System_Exe();
      void Enter_Command();
//	  int mode = 0; 
	  const  string  get_name(int n);  
	  void getColorSpacePoint(int , ColorSpacePoint);//
	  void resize(ColorSpacePoint *, double, double, double, double);//
	  bool isMeetPose(double lockX[JointType_Count], double lockY[JointType_Count], ColorSpacePoint *);//
};