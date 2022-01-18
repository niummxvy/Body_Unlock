#include <iostream>
#include <cmath>
#include <ctime>
#include "Body_Unlock.h"
#define THRESHOLD 50

using namespace std;

Body_Unlock::Body_Unlock()
{
	for (int i = 0; i < JointType_Count*6; ++i) { pwd[i] = 0; }
	mode = 1;
/*	leftbound = 0;
	rightbound = 0;
	upbound = 0;
	downbound = 0; */
}

Body_Unlock::~Body_Unlock(){}

int Body_Unlock::getMode()
{
	return mode;
}
/*
void Body_Unlock::setLeftBound(double pos)
{
	if (pos < leftbound) leftbound = pos;
}

void Body_Unlock::setRightBound(double pos)
{
	if (pos > rightbound) rightbound = pos;
}

void Body_Unlock::setUpBound(double pos)
{
	if (pos < upbound) upbound = pos;
}

void Body_Unlock::setDownBound(double pos)
{
	if (pos > downbound) downbound = pos;
}

void Body_Unlock::setCenter(ColorSpacePoint *Point)
{
	Point->X = (Point->X - (rightbound + leftbound) / 2) / (rightbound - leftbound);
	Point->Y = (Point->Y - (downbound + upbound) / 2) / (downbound - upbound);
}

double Body_Unlock::getLeftBound()
{
	return leftbound;
}

double Body_Unlock::getRightBound()
{
	return rightbound;
}

double Body_Unlock::getUpBound()
{
	return upbound;
}

double Body_Unlock::getDownBound()
{
	return downbound;
}

void Body_Unlock::setMode(int newmode)
{
	mode = newmode;
	if (mode == 0) cout << "State: Locked" << endl;
	else if (mode == 1) cout << "State: Unlocked" << endl;
}

void Body_Unlock::resize(ColorSpacePoint * Point)
{
	Point->X = (Point->X - (rightbound + leftbound) / 2) / (rightbound - leftbound);
	Point->Y = (Point->Y - (downbound + upbound) / 2) / (downbound - upbound);
}
*/
bool Body_Unlock::judge(ColorSpacePoint *newPos)
{
	clock_t start, end;
	start = clock();
	while (1) {
		for (int i = 0; i < JointType_Count; ++i) {
//			if (pwd[i] > 50) return false;
			
			if (sqrt(pow(newPos[i * 6 + 0].X - newPos[JointType_Head].X, 2)
				+ pow(newPos[i * 6 + 0].Y - newPos[JointType_Head].Y, 2)) - pwd[i * 6 + 0] > THRESHOLD || // 與頭的距離(新-舊) > THRESHOLD就false，以下以此類推，只是換關節而已
				sqrt(pow(newPos[i * 6 + 1].X - newPos[JointType_HandLeft].X, 2)
					+ pow(newPos[i * 6 + 1].Y - newPos[JointType_HandLeft].Y, 2)) - pwd[i * 6 + 1] > THRESHOLD ||
				sqrt(pow(newPos[i * 6 + 2].X - newPos[JointType_HandRight].X, 2)
					+ pow(newPos[i * 6 + 2].Y - newPos[JointType_HandRight].Y, 2)) - pwd[i * 6 + 2] > THRESHOLD ||
				sqrt(pow(newPos[i * 6 + 3].X - newPos[JointType_SpineBase].X, 2)
					+ pow(newPos[i * 6 + 3].Y - newPos[JointType_SpineBase].Y, 2)) - pwd[i * 6 + 3] > THRESHOLD ||
				sqrt(pow(newPos[i * 6 + 4].X - newPos[JointType_FootLeft].X, 2)
					+ pow(newPos[i * 6 + 4].Y - newPos[JointType_FootLeft].Y, 2)) - pwd[i * 6 + 4] > THRESHOLD ||
				sqrt(pow(newPos[i * 6 + 5].X - newPos[JointType_FootRight].X, 2)
					+ pow(newPos[i * 6 + 5].Y - newPos[JointType_FootRight].Y, 2)) - pwd[i * 6 + 5] > THRESHOLD) return false;
			}
		end = clock();
		if (end - start > 2000) break; // Keep same pose for 2s.
	}
	mode = 1;
	cout << "Successfully unlocked!!" << endl;
	return true;
}

void Body_Unlock::locked()
{
	mode = 0;
	cout << "State: locked" << endl;
}

void Body_Unlock::resetLock(ColorSpacePoint *newPos)
{
	double resample[101][150] = {};
	clock_t start, end;
	start = clock();
	while (1) {





		end = clock();
		if (end - start > 2000) break;
	}


	if (mode == 1) {
		for (int i = 0; i < JointType_Count; ++i) {
			pwd[i * 6 + 0] = sqrt(pow(newPos[i].X - newPos[JointType_Head].X, 2)
								+ pow(newPos[i].Y - newPos[JointType_Head].Y, 2));
			pwd[i * 6 + 1] = sqrt(pow(newPos[i].X - newPos[JointType_HandLeft].X, 2)
								+ pow(newPos[i].Y - newPos[JointType_HandLeft].Y, 2));
			pwd[i * 6 + 2] = sqrt(pow(newPos[i].X - newPos[JointType_HandRight].X, 2)
								+ pow(newPos[i].Y - newPos[JointType_HandRight].Y, 2));
			pwd[i * 6 + 3] = sqrt(pow(newPos[i].X - newPos[JointType_SpineBase].X, 2)
								+ pow(newPos[i].Y - newPos[JointType_SpineBase].Y, 2));
			pwd[i * 6 + 4] = sqrt(pow(newPos[i].X - newPos[JointType_FootLeft].X, 2)
								+ pow(newPos[i].Y - newPos[JointType_FootLeft].Y, 2));
			pwd[i * 6 + 5] = sqrt(pow(newPos[i].X - newPos[JointType_FootRight].X, 2)
								+ pow(newPos[i].Y - newPos[JointType_FootRight].Y, 2));
		}
		cout << "New password is set." << endl;
	}
		
}

void Body_Unlock::clear()
{
	for (int i = 0; i < JointType_Count*6; ++i) { pwd[i]; }
//	cout << "Password is empty!!"
}
