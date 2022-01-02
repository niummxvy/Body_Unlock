//Standard Library
#include <iostream>
#include <stdio.h>
#include <string>
#include <ctime>
#include <windows.h>
//OpenCV Header
#include <opencv2\opencv.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\imgproc.hpp>
#include<opencv2\video\background_segm.hpp>
//using namespace cv;
using namespace std;
// Kinect for Windows SDK Header
#include <Kinect.h>
#include "Body_Unlock.h"

const  string  get_name(int n);  //判斷關節的名字

void getColorSpacePoint(int skeleton, ColorSpacePoint Point);

int main()
{
// ----------------------------------------------Preprocessing----------------------------------------------------------
	/*
		(1) 取得 Sensor
		(2)選擇你所要讀取的資料源(data source) / Select the data source to read from
		(3)處理從資料源讀取到的資料 / Handle the data you read from the source
	*/
	//取得預設傳感器並開啟
	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	//一個關於Kinect V2觀念  :  不同種數據會搭配兩個項目(源、閱讀器) IBody , IDepth , IColor .....etc
	//一個xxxFrameSource(源)
	//一個xxxFrameReader(閱讀器)

	//處理三步驟
	/*
		從「傳感器」獲得「源」
		從「源」打開「閱讀器」
		從「閱讀器」獲得「視訊幀(frame)」
	*/

	int myBodyCount = 1;
	IBodyFrameSource    * myBodySource = nullptr;//一個IBodyFrameSource(身體數據源)
	IBodyFrameReader    * myBodyReader = nullptr;//一個IBodyFrameReader(身體閱讀器)

	//從「傳感器」獲得「身體數據源」..................................................................................step1_1
	mySensor->get_BodyFrameSource(&myBodySource);
	
	//從「身體數據源」打開「身體閱讀器」..............................................................................step1_2
	myBodySource->OpenReader(&myBodyReader);
	
//	myBodySource->get_BodyCount(&myBodyCount);// 取得預設微軟限定的遍歷6人的bodyCount


	IColorFrameSource    * myColorSource = nullptr;//一個IColorFrameSource(色彩數據源)
	IColorFrameReader    * myColorReader = nullptr;//一個IColorFrameReader(色彩閱讀器)

	//從「傳感器」獲得「色彩數據源」..................................................................................step3_1
	mySensor->get_ColorFrameSource(&myColorSource);
	//從「深度數據源」打開「色彩閱讀器」..............................................................................step3_2
	myColorSource->OpenReader(&myColorReader);


	int width = 0, height = 0;
	IFrameDescription   * myColorDescription = nullptr;
	myColorSource->get_FrameDescription(&myColorDescription);
	myColorDescription->get_Height(&height);
	myColorDescription->get_Width(&width);   //以上為準備好色彩數據和骨骼數據的Reader


	IBodyFrame  * myBodyFrame = nullptr;

	IColorFrame  * myColorFrame = nullptr;
	// Prepare OpenCV data
	cv::Mat	mImg(height, width, CV_8UC4);
	UINT uBufferSize = height * width * 4 * sizeof(BYTE);
	cv::namedWindow("Color Map");

	// Coordinate Mapper
	ICoordinateMapper* myCoordinateMapper = nullptr;
	while (mySensor->get_CoordinateMapper(&myCoordinateMapper) != 0);
	

//	double MaxX = 0, minX = 0, MaxY = 0, minY = 0;
	Body_Unlock user;
	clock_t start, check, resample_start, resample_end;
	bool isStart = false, isResampleStart = false;
//	vector<ColorSpacePoint*> resample;
// ----------------------------------------------Capturing(1 frame per Sleep(x)(ms))----------------------------------------------------------
	while (1)
	{
		if (user.getMode() == 1 && !isStart) { // Calculate unlocked idle time(start).
			start = clock();
			isStart = true;
		}

//		cout << "start time: " << start << endl;


// ---------------------------------------------------Acquire latest frame then imshow--------------------------------------------------------
		//從「身體閱讀器」獲得「身體視訊幀(frame)」//..............................................step1_3
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != 0); // Break when body reader read body frame/skeleton.
		//從「色彩閱讀器」獲得「色彩視訊幀(frame)」//..............................................step1_3
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != 0); // Break when color reader read color frame/skeleton.
		if(myColorFrame->CopyConvertedFrameDataToArray(uBufferSize, mImg.data, ColorImageFormat_Bgra) == 0) 
			cv::imshow("Color Map", mImg);
		else cerr << "Data copy error" << endl; 
// --------------------------------------------Introduce bodyArr to store 25 joints per person------------------------------------------------
		int myBodyCount = 0;

		myBodySource->get_BodyCount(&myBodyCount);//取得攝影機捕捉到的人體總數


		IBody ** bodyArr = new IBody*[myBodyCount];
		//bodyArr 就是之後讓 K4W SDK v2 紀錄骨架資料的陣列

		for (int i = 0; i < myBodyCount; i++)   //bodyArr的初始化
			bodyArr[i] = nullptr;

		myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr); // 可能還有前人記錄未刪，所以要先Refrash

		for (int i = 0; i < myBodyCount; i++)   //遍歷6個人
		{
			BOOLEAN     result = true;
			if (bodyArr[i]->get_IsTracked(&result) == 0 && result)
			{
				std::cout << "Body " << i << " tracked!" << endl;

				int count = 0;
				Joint  jointArr[JointType_Count]; // Store 3D coordinates (CameraSpacePoint)
				ColorSpacePoint ColorArr[JointType_Count]; // Store 2D coordinates (ColorSpacePoint)

				//「JointType_Count」，來代表總共的關節數目(25)
				bodyArr[i]->GetJoints(JointType_Count, jointArr);    //獲取此人的關節數據

				for (int j = 0; j < JointType_Count; j++){
					myCoordinateMapper->MapCameraPointToColorSpace(jointArr[j].Position, &ColorArr[j]);
					getColorSpacePoint(j, ColorArr[j]);
				}
				cout << endl;

/*				if (user.getMode() == 0) { // 指定動作維持2秒可解鎖
					if (user.judge(ColorArr)) {
						user.setMode(1);
						Sleep(3000); // 改完先間隔三秒
					}
				} */
/*				else if (user.getMode() == 1) { // 指定動作維持2秒可重設密碼
					if (user.judge(ColorArr) && !isResampleStart) {
						resample_start = clock();
						isResampleStart = true;
					}
					
					resample_end = clock();
					if (resample_end - resample_start > 2000) {

					}
				} */


			}





			
		}

		
		//delete bodyArr;
		myColorFrame->Release();
		myBodyFrame->Release(); //  查詢有沒有可以自動清空IBodyFrame的函式or限制memory用量
		for (int i = 0; i < myBodyCount; i++)
			bodyArr[i] = nullptr;
		delete[]  *bodyArr ;

		if (cv::waitKey(30) == 27)
			break;
		Sleep(40);    //每秒鐘更新一次, 放慢速度用
		
		check = clock();
//		cout << "check time: " << check << endl;
		if (check - start > 10000) {
			user.setMode(0); // unlock 10s
			isStart = false;
		}

		Sleep(1000);
		system("cls");

	}


	myBodyReader->Release();
	myColorReader->Release();
	myBodySource->Release();
	myColorSource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

const   string  get_name(int n)
{
	switch (n)
	{
	case 2:
		return "脖子";
		break;
	case 20:
		return  "肩膀中央";
		break;
	case 4:
		return  "左肩膀";
		break;
	case 8:
		return  "右肩膀";
		break;
	case 1:
		return  "脊椎中央";
		break;

	default:return "NULL";
	}
}

void getColorSpacePoint(int skeleton, ColorSpacePoint Point)
{
	if (skeleton == JointType_Head) cout << "Head:      " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HandLeft) cout << "LeftHand:  " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HandRight) cout << "RightHand: " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_SpineBase) cout << "SpineBase: " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_FootLeft) cout << "LeftFoot:  " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_FootRight) cout << "RightFoot: " << Point.X << ", " << Point.Y << endl;
}

