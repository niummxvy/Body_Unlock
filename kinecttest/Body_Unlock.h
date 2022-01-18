#pragma once
#include <Kinect.h>

class Body_Unlock {
private:
	double pwd[150];
/*
由頭頂至下方鼠蹊部
3-> 頭(JointType_Head)
2 -> 脖子(JointType_Neck)
20 -> 肩膀中央(JointType_SpineShoulder)
1 -> 背骨(脊椎中央)(JointType_SpineMid)
0 -> 脊椎底(鼠蹊部中央)(JointType_SpineBase)


左半邊身體
上方"手"部分
4-> 左肩膀(JointType_ShoulderLeft)
5-> 左肘(JointType_ElbowLeft)
6-> 左腕(JointType_WristLeft)
7-> 左手(JointType_HandLeft)
21->左手尖端(JointType_HandTipLeft)
22-> 左手拇指(JointType_ThumbLeft)

下方"腳"部分
12->左臀(JointType_HipLeft)
13->左膝(JointType_KneeLeft)
14->左腳踝(JointType_AnkleLeft)
15->左腳(JointType_FootLeft)


右半邊身體
上方"手"部分
8-> 右肩膀(JointType_ShoulderRight)
9-> 右肘(JointType_ElbowRight)
10-> 右腕(JointType_WristRight)
11-> 右手(JointType_HandRight)
23->右手尖端(JointType_HandTipRight)
24->右手拇指(JointType_ThumbRight)

下方"腳"部分
16->右臀(JointType_HipRight)
17->右膝(JointType_KneeRight)
18->右腳踝(JointType_AnkleRight)
19->右腳(JointType_FootRight)
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