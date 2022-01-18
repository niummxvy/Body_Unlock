#pragma once
#include <Kinect.h>

class Body_Unlock {
private:
	double pwd[150];
/*
���Y���ܤU�蹫�ѳ�
3-> �Y(JointType_Head)
2 -> ��l(JointType_Neck)
20 -> �ӻH����(JointType_SpineShoulder)
1 -> �I��(��դ���)(JointType_SpineMid)
0 -> ��թ�(���ѳ�����)(JointType_SpineBase)


���b�䨭��
�W��"��"����
4-> ���ӻH(JointType_ShoulderLeft)
5-> ���y(JointType_ElbowLeft)
6-> ����(JointType_WristLeft)
7-> ����(JointType_HandLeft)
21->����y��(JointType_HandTipLeft)
22-> ������(JointType_ThumbLeft)

�U��"�}"����
12->���v(JointType_HipLeft)
13->����(JointType_KneeLeft)
14->���}��(JointType_AnkleLeft)
15->���}(JointType_FootLeft)


�k�b�䨭��
�W��"��"����
8-> �k�ӻH(JointType_ShoulderRight)
9-> �k�y(JointType_ElbowRight)
10-> �k��(JointType_WristRight)
11-> �k��(JointType_HandRight)
23->�k��y��(JointType_HandTipRight)
24->�k����(JointType_ThumbRight)

�U��"�}"����
16->�k�v(JointType_HipRight)
17->�k��(JointType_KneeRight)
18->�k�}��(JointType_AnkleRight)
19->�k�}(JointType_FootRight)
*/
/*
0~5(i == JointType_SpineBase), 6~11(i == JointType_SpineMid), 12~17(i == JointType_Neck), ..., 144~149(i == JointType_ThumbRight)
pwd[i%6] == 0: dist(i, JointType_Head)
		 == 1: dist(i, JointType_HandLeft)
		 == 2: dist(i, JointType_HandRight)
		 == 3: dist(i, JointType_SpineBase)
		 == 4: dist(i, JointType_FootLeft)
		 == 5: dist(i, JointType_FootRight)
*/

	int mode;
/*	mode = 0: locked
		 = 1: unlocked(can set new lock)
*/
/*	double leftbound;
	double rightbound;
	double upbound;
	double downbound; */
public:
	Body_Unlock();
	~Body_Unlock();
	int getMode();
/*	void setLeftBound(double);
	void setRightBound(double);
	void setUpBound(double);
	void setDownBound(double);
	void setCenter(ColorSpacePoint *);
	double getLeftBound();
	double getRightBound();
	double getUpBound();
	double getDownBound(); */
//	void setMode(int);
	bool judge(ColorSpacePoint *);
	void locked();
	void resetLock(ColorSpacePoint *);
	void clear();

};