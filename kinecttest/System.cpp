#include"System.h"
System::System(){
    IS_LOCKED = true;
    IS_STAY_SYSTEM = true;
    command = "";
//	clock_t start, check, try_unlock_start, try_unlock_end, unlocked_start, unlocked_end;
	rec_unlocked_end = 0;
	isStart = false; isResampleStart = false; isTryStart = false; isUnlockedStart = false, new_lock_is_set = false, from_unlock_to_lock = false;
	mode = 0;
    mySensor = nullptr;
	myBodyCount = 1;
    myBodySource = nullptr;//一個IBodyFrameSource(身體數據源)
    myBodyReader = nullptr;//一個IBodyFrameReader(身體閱讀器)
    myColorSource = nullptr;//一個IColorFrameSource(色彩數據源)
    myColorReader = nullptr;//一個IColorFrameReader(色彩閱讀器)
	width = 0; height = 0;
    myColorDescription = nullptr;
    myBodyFrame = nullptr;
    myColorFrame = nullptr;
    myCoordinateMapper = nullptr;
	command_num = true;
}
void System::Enter_Command(){
	while (std::cin >> command) {
		if (command == "1") {
			command_num = false;
			break;
		}
		else continue;
	}
	if (command_num)SystemExecv.notify_one();
}
void System::System_Exe() {
	std::unique_lock<std::mutex> lck(SystemExetx);
	std::thread Commandth(&System::Enter_Command, this);
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();
	mySensor->get_BodyFrameSource(&myBodySource);
	myBodySource->OpenReader(&myBodyReader);
	mySensor->get_ColorFrameSource(&myColorSource);
	myColorSource->OpenReader(&myColorReader);
	myColorSource->get_FrameDescription(&myColorDescription);
	myColorDescription->get_Height(&height);
	myColorDescription->get_Width(&width);
	cv::Mat	mImg(height, width, CV_8UC4);
	UINT uBufferSize = height * width * 4 * sizeof(BYTE);
	//	cv::namedWindow("Color Map");



	while (mySensor->get_CoordinateMapper(&myCoordinateMapper) != 0);
	while (1) {
		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != 0) {
			cout << "Waiting for Latest Body Frame.\n"; 
			system("cls");
		}
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != 0) {
			cout << "Waiting for Latest Color Frame.\n"; 
			system("cls");
		}
		//			if (myColorFrame->CopyConvertedFrameDataToArray(uBufferSize, mImg.data, ColorImageFormat_Bgra) == 0)
		//				cv::imshow("Color Map", mImg); // open window
		//			else cerr << "Data copy error" << endl;
		int myBodyCount = 0;
		myBodySource->get_BodyCount(&myBodyCount);
		IBody ** bodyArr = new IBody*[myBodyCount];
		for (int i = 0; i < myBodyCount; i++)
			bodyArr[i] = nullptr;
		myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr);


		for (int i = 0; i < myBodyCount; i++)
		{
			double leftbound = 2000;
			double rightbound = -1000;
			double upbound = 2000;
			double downbound = -1000;
			BOOLEAN     result = true;

			if (bodyArr[i]->get_IsTracked(&result) == 0 && result)
			{
				std::cout << "Body " << i << " tracked!" << endl;

				int count = 0;
				Joint  jointArr[JointType_Count]; // joint coordinates (3D)
				ColorSpacePoint ColorArr[JointType_Count];  // joint coordinates (2D)
				bodyArr[i]->GetJoints(JointType_Count, jointArr);


				for (int j = 0; j < JointType_Count; j++) {
					myCoordinateMapper->MapCameraPointToColorSpace(jointArr[j].Position, &ColorArr[j]); // map camera point(3D) to color space(2D)
					if (ColorArr[j].X > -INFINITY && ColorArr[j].X < leftbound) leftbound = ColorArr[j].X; // update bound
					if (ColorArr[j].X > -INFINITY && ColorArr[j].X > rightbound) rightbound = ColorArr[j].X; // update bound
					if (ColorArr[j].Y > -INFINITY && ColorArr[j].Y < upbound) upbound = ColorArr[j].Y; // update bound
					if (ColorArr[j].Y > -INFINITY && ColorArr[j].Y > downbound) downbound = ColorArr[j].Y; // update bound
//					cout << '(' << ColorArr[j].X << ',' << ColorArr[j].Y << ") / ";
//					if (j != 0 && j % 5 == 0) cout << endl;
				}
				cout << endl << endl;
				for (int j = 0; j < JointType_Count; j++) {
//					resize(&ColorArr[j], leftbound, rightbound, upbound, downbound);
					ColorArr[j].X = 2 * (ColorArr[j].X - (rightbound + leftbound) / 2) / (rightbound - leftbound);
					ColorArr[j].Y = 2 * (ColorArr[j].Y - (downbound + upbound) / 2) / (downbound - upbound);
					// resize coordinates according to 4 bounds
				}
//				cout << "leftbound: " << leftbound << "\nrightbound: " << rightbound << "\nupbound: " << upbound << "\ndownbound: " << downbound << endl;
/*				for (int j = 0; j < JointType_Count; ++j) {
					cout << lockX[j] << " / " << ColorArr[j].X << ", ";
					if (j != 0 && j % 5 == 0) cout << endl;
				}
				cout << endl << endl;
				for (int j = 0; j < JointType_Count; ++j) {
					cout << lockY[j] << " / " << ColorArr[j].Y << ", ";
					if (j != 0 && j % 5 == 0) cout << endl;
				}
				cout << endl << endl;
*/
				if (isMeetPose(lockX, lockY, ColorArr)) { // if the pose can unlock

					if (!isTryStart) { // make a timestamp try_unlock_start, which can record a person start to try to unlock
						try_unlock_start = clock();
						isTryStart = true;
					}
					else {
						if (from_unlock_to_lock) { // avoid being able to unlock right after locked(can unlock after 0s during lock)
							try_unlock_start = rec_unlocked_end; // avoid accumulating time from last unlock
							rec_unlocked_end = clock() - rec_unlocked_end; // difference between now and last unlocked_end
							from_unlock_to_lock = false;
						}
						else try_unlock_end = clock() - rec_unlocked_end; // make a timestamp try_unlock_end (or resume to last unlocked_end time)
//							cout << "try_unlock_start: " << try_unlock_start << ", try_unlock_end: " << try_unlock_end << endl;
						if (try_unlock_end - try_unlock_start > 3000) { // if a person keep the same pose 4s, he/she can unlock/reset a lock
							isTryStart = false; // this can reset try_unlock_start next loop
							if (mode == 0) { // lock -> unlock
								mode = 1;
								cout << "Now is unlocked!!\n";
								PlaySound("unlock.wav", NULL, SND_FILENAME | SND_ASYNC); // play alart sound
								Sleep(3000); // display "Now is unlocked!!" for 3s but clock_t is still going
								if (!isUnlockedStart) { // make a timestamp unlocked_start, which can record the start time of unlocked
									unlocked_start = clock();
									isUnlockedStart = true;
									new_lock_is_set = false;
								}
							}
							else if (mode == 1 && !new_lock_is_set) { // reset lock
								system("cls");
								cout << "Ready to set new lock." << endl;
								Sleep(1000); // but clock_t is still going
								for (int j = 3; j > 0; --j) {
									cout << "Ready to get a screenshot after " << j << "s  (Confirm your posture now!)\n";
									Sleep(1000); // but clock_t is still going
								}

								leftbound = 2000;
								rightbound = -1000;
								upbound = 2000;
								downbound = -1000;
								// get new coordinates this frame
								myColorFrame->Release();
								myBodyFrame->Release();
								while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != 0) cout << "Waiting for Latest Body Frame.\n";
								while (myColorReader->AcquireLatestFrame(&myColorFrame) != 0) cout << "Waiting for Latest Color Frame.\n";
								myBodyFrame->GetAndRefreshBodyData(myBodyCount, bodyArr);
								bodyArr[i]->GetJoints(JointType_Count, jointArr);
								for (int j = 0; j < JointType_Count; j++) {
									myCoordinateMapper->MapCameraPointToColorSpace(jointArr[j].Position, &ColorArr[j]); // map camera point(3D) to color space(2D)
									if (ColorArr[j].X > -INFINITY && ColorArr[j].X < leftbound) leftbound = ColorArr[j].X; // update bound
									if (ColorArr[j].X > -INFINITY && ColorArr[j].X > rightbound) rightbound = ColorArr[j].X; // update bound
									if (ColorArr[j].Y > -INFINITY && ColorArr[j].Y < upbound) upbound = ColorArr[j].Y; // update bound
									if (ColorArr[j].Y > -INFINITY && ColorArr[j].Y > downbound) downbound = ColorArr[j].Y; // update bound
//									cout << '(' << ColorArr[j].X << ',' << ColorArr[j].Y << ") / ";
//									if (j != 0 && j % 5 == 0) cout << endl;
								}
//								cout << "\n\n";
								
								for (int j = 0; j < JointType_Count; j++) {
//									resize(&ColorArr[j], leftbound, rightbound, upbound, downbound);
									ColorArr[j].X = 2 * (ColorArr[j].X - (rightbound + leftbound) / 2) / (rightbound - leftbound);
									ColorArr[j].Y = 2 * (ColorArr[j].Y - (downbound + upbound) / 2) / (downbound - upbound);
									// resize coordinates according to 4 bounds
									lockX[j] = ColorArr[j].X; // set new lock: x
									lockY[j] = ColorArr[j].Y; // set new lock: y
//									cout << '(' << lockX[j] << ',' << lockY[j] << ") / ";
//									if (j != 0 && j % 5 == 0) cout << endl;
								}
//								cout << "\n\n";
//								cout << "leftbound: " << leftbound << "\nrightbound: " << rightbound << "\nupbound: " << upbound << "\ndownbound: " << downbound << endl;
								cout << "New lock is set!!\n";
								new_lock_is_set = true;
								PlaySound("set_success.wav", NULL, SND_FILENAME | SND_ASYNC); // play alart sound
								Sleep(3000); // display "New lock is set!!" for 3s but clock_t is still going
							}
						}
					}
				}
				else { // reset try_unlock_start
					isTryStart = false;
				}

				if (mode == 0) {
					cout << "State: locked\n";
				}
				else if (mode == 1) {
					cout << "State: unlocked\n";
					unlocked_end = clock(); // make a timestamp unlocked_end, which can record how long it passed by after unlocked
//						cout << "unlocked_start: " << unlocked_start << ", " << "unlocked_end: " << unlocked_end << endl;

				}
				if (isUnlockedStart) {
					cout << "Unlocked: " << (unlocked_end - unlocked_start) / 1000 << "s\n"; // every second counts
					if (unlocked_end - unlocked_start > 60000) { // unlocked can idle 20s, then resume to locked
						cout << "Now is locked!!\n";
						//Enter_Command();
						//if (command == "Exit")IS_STAY_SYSTEM = false;
						PlaySound("lock.wav", NULL, SND_FILENAME | SND_ASYNC); // play alart sound
						from_unlock_to_lock = true;
						rec_unlocked_end = unlocked_end;
						Sleep(3000); // display "Now is locked!!" for 3s but clock_t is still going
						mode = 0;
						isUnlockedStart = false; // reset unlocked_start
					}
				}
			}
		}

		myColorFrame->Release();
		myBodyFrame->Release();
		for (int i = 0; i < myBodyCount; i++)
			bodyArr[i] = nullptr;
		delete[] * bodyArr;

		if (cv::waitKey(30) == 27) // pressing "esc" will break
			break;
		system("cls");

	}
	//Commandth.join();

	myBodyReader->Release();
	myColorReader->Release();
	myBodySource->Release();
	myColorSource->Release();
	mySensor->Close();
	mySensor->Release();
}
	

void System::getColorSpacePoint(int skeleton, ColorSpacePoint Point)
{

	if (skeleton == JointType_SpineBase) cout << "SpineBase:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_SpineMid) cout << "SpineMid:      " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_Neck) cout << "Neck:          " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_Head) cout << "Head:          " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_ShoulderLeft) cout << "ShoulderLeft:  " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_ElbowLeft) cout << "ElbowLeft:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_WristLeft) cout << "WristLeft:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HandLeft) cout << "LeftHand:      " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_ShoulderRight) cout << "ShoulderRight:  " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_ElbowRight) cout << "ElbowRight:    " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_WristRight) cout << "WristRight:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HandRight) cout << "HandRight:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HipLeft) cout << "HipLeft:       " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_KneeLeft) cout << "KneeLeft:      " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_AnkleLeft) cout << "AnkleLeft:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_FootLeft) cout << "FootLeft:      " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HipRight) cout << "HipRight:      " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_KneeRight) cout << "KneeRight:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_AnkleRight) cout << "AnkleRight:    " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_FootRight) cout << "FootRight:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_SpineShoulder) cout << "SpineShoulder: " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HandTipLeft) cout << "HandTipLeft:   " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_ThumbLeft) cout << "ThumbLeft:     " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_HandTipRight) cout << "HandTipRight:  " << Point.X << ", " << Point.Y << endl;
	if (skeleton == JointType_ThumbRight) cout << "ThumbRight:    " << Point.X << ", " << Point.Y << endl;

}
/*
void System::resize(ColorSpacePoint *Point, double leftbound, double rightbound, double upbound, double downbound) {
	Point->X = 2 * (Point->X - (rightbound + leftbound) / 2) / (rightbound - leftbound);
	Point->Y = 2 * (Point->Y - (downbound + upbound) / 2) / (downbound - upbound);
}
*/
bool System::isMeetPose(double lockX[JointType_Count], double lockY[JointType_Count], ColorSpacePoint *newPos)
{
	for (int i = 0; i < JointType_Count; ++i) {
		if (abs(newPos[i].X - lockX[i]) > THRESHOLD_X || abs(newPos[i].Y - lockY[i]) > THRESHOLD_Y) {
			if (i == JointType_SpineBase) cout << "SpineBase: Error!!" << endl;
			if (i == JointType_SpineMid) cout << "SpineMid: Error!!" << endl;
			if (i == JointType_Neck) cout << "Neck: Error!!" << endl;
			if (i == JointType_Head) cout << "Head: Error!!" << endl;
			if (i == JointType_ShoulderLeft) cout << "ShoulderLeft: Error!!" << endl;
			if (i == JointType_ElbowLeft) cout << "ElbowLeft: Error!!" << endl;
			if (i == JointType_WristLeft) cout << "WristLeft: Error!!" << endl;
			if (i == JointType_HandLeft) cout << "LeftHand: Error!!" << endl;
			if (i == JointType_ShoulderRight) cout << "ShoulderRight: Error!!" << endl;
			if (i == JointType_ElbowRight) cout << "ElbowRight: Error!!" << endl;
			if (i == JointType_WristRight) cout << "WristRight: Error!!" << endl;
			if (i == JointType_HandRight) cout << "HandRight: Error!!" << endl;
			if (i == JointType_HipLeft) cout << "HipLeft: Error!!" << endl;
			if (i == JointType_KneeLeft) cout << "KneeLeft: Error!!" << endl;
			if (i == JointType_AnkleLeft) cout << "AnkleLeft: Error!!" << endl;
			if (i == JointType_FootLeft) cout << "FootLeft: Error!!" << endl;
			if (i == JointType_HipRight) cout << "HipRight: Error!!" << endl;
			if (i == JointType_KneeRight) cout << "KneeRight: Error!!" << endl;
			if (i == JointType_AnkleRight) cout << "AnkleRight: Error!!" << endl;
			if (i == JointType_FootRight) cout << "FootRight: Error!!" << endl;
			if (i == JointType_SpineShoulder) cout << "SpineShoulder: Error!!" << endl;
			if (i == JointType_HandTipLeft) cout << "HandTipLeft: Error!!" << endl;
			if (i == JointType_ThumbLeft) cout << "ThumbLeft: Error!!" << endl;
			if (i == JointType_HandTipRight) cout << "HandTipRight: Error!!" << endl;
			if (i == JointType_ThumbRight) cout << "ThumbRight: Error!!" << endl;
			cout << "Target: (" << lockX[i] << ',' << lockY[i] << ")\nNow: (" << newPos[i].X << ',' << newPos[i].Y << ")\n";
			return false;
		}
	}

	return true;
}

