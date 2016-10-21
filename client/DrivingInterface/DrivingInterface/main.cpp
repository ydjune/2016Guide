
#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#include <math.h>
#include <Windows.h>
#include <iostream>
using namespace std;

#define SHARED_MOMORY_NAME1 "TORCS_SHARED1"
#define SHARED_MOMORY_NAME2 "TORCS_SHARED2"

#define CURVE_TYPE_RIGHT		1
#define CURVE_TYPE_LEFT			2
#define CURVE_TYPE_STRAIGHT		3

#define GEAR_FORWARD			0   // 전진 (D)
#define GEAR_BACKWARD			-1	// 후진 (R)




#define INPUT_AICAR_SIZE			20
#define INPUT_FORWARD_TRACK_SIZE	20

struct shared_use_st
{
	// System Value
	BOOL connected;
	int written;

	// Driving Parameters
	double toMiddle;//자신의 중심과 트랙중심까지의 위치
	double angle;
	double speed;

	// Track Parameters
	double toStart;//출발점까지의 거리
	double dist_track;
	double track_width;//트랙의 폭
	double track_dist_straight;//차량 전방의 연속된 직진 트랙의 길이
	int    track_curve_type;//차량 전방의 직진이 끝나는 지점의 커브 종류
	double track_forward_angles[INPUT_FORWARD_TRACK_SIZE];
	double track_forward_dists[INPUT_FORWARD_TRACK_SIZE];
	double track_current_angle;

	// Other Cars Parameters
	double dist_cars[INPUT_AICAR_SIZE];

	// Racing Info. Parameters
	double damage;
	double damage_max;
	int    total_car_num;
	int    my_rank;
	int    opponent_rank;

	// Output Values
	double steerCmd;
	double accelCmd;
	double brakeCmd;
	int    backwardCmd;
};

double STEER_V = 0.02;
double ACCEL_V = 0.1;
double BREAKE_V= 0.1;
int index = 0;
int controlDriving(shared_use_st *shared){
	if (shared == NULL) return -1;

	//Input : shared_user_st
	//TO-DO : 알고리즘을 작성하세요.
	//Output : 4개의 Output Cmd 값을 도출하세요.
	
	
	
	double targetPosition;

	int frontCar = -1;
	for (int i = 18; i >= 0; i -= 2) {
		if (0 < shared->dist_cars[i] && shared->dist_cars[i] < 100) {
			frontCar = i;
			break;
		}
	}
	
	//커브를 돌 때 최단거리
	//앞차가 가로막고 있을 때 피해서 지나가기

	double vMax;// = 32;
	double vBrake;
	if (shared->track_dist_straight < 160) {
		//int angle = shared->angle<0 ? -1 * shared->angle : shared->angle;
		//?????????????????????????? 이 angle 이 어떻게 되는 걸까?

		double diffAngle = shared->track_forward_angles[19] - shared->track_current_angle;

		//vMax = 200 - abs(diffAngle)*37;
		vMax = 200;
		vBrake = abs(diffAngle)*0.037;
		//printf("%f\r\n", shared->track_forward_angles[0]);
	}
	else {
		vMax = 200;
		vBrake = 0;
	}
	
	double c = 3.772;//이 변수도 차간 거리에 영향을 주는 듯 싶다. 원래 2.772->1.772바꾸엇더니 앞차와의 거리가 더 멀어진 상태에서 주행을 함(d값은 원래값으로 돌려논상태)
	


	
	double d = -0.693;//이 값을 줄일수록 앞 차와의 거리가 준 상태에서 따라간다.
	//cout << exp(-1 * c / vMax * shared->dist_cars[0] - d) << endl;
	//cout << "vMax값: "<<vMax << endl;
	double avoidToMiddle = 0;
	double halfTrack_width = shared->track_width/2;
	if (shared->dist_cars[index]!=0 && shared->dist_cars[index]<26 && shared->toMiddle - shared->dist_cars[index + 1] >= 0)//내차의트랙중심거리와 목표차량트랙중심거리차가 음수이면 차가 부딪칠 수 있는 라인에 있다는 뜻이므로 가까이 있다면 다른차를 목표로 따라가야 한다.
		index+=2;
	
	
	
	
	double steer_C = 3.0;//C값이 낮을수록 커브에 대한 민감도가 줄어든다.
	//커브가 급할때는 c값을 높이 올려주고 커브가 약할때는 C값을 낮게~
	//앞에 차량이 있는 곳을 피한 도로의 중심점을 찾기 위한 계산

	/*cout << abs(shared->track_forward_angles[0]) - abs(shared->angle) << endl;
	if (abs(shared->angle) < abs(shared->track_forward_angles[0]))
	{
		
	    if (abs(shared->track_forward_angles[0]) - abs(shared->angle) >2)
			steer_C = 5.0;
		else if (abs(shared->track_forward_angles[0]) - abs(shared->angle) >= 1)
			steer_C = 3.0;
		else if (abs(shared->track_forward_angles[0]) - abs(shared->angle) > 0.5)
			steer_C = 1.2;
		else
			steer_C = 0.5;
	}*/



	if (shared->dist_cars[1]>0)//도로의 우측
		avoidToMiddle = (halfTrack_width - shared->dist_cars[1]) / 2;
	else//도로의 좌측
		avoidToMiddle = (-1 * halfTrack_width - shared->dist_cars[1]) / 2;

	if (shared->dist_cars[0]<18 && (abs(shared->toMiddle)+1 >= abs(shared->dist_cars[1]) && abs(shared->toMiddle)-1 <= abs(shared->dist_cars[1])))//충돌각임...
		shared->steerCmd = steer_C * (shared->angle - avoidToMiddle / shared->track_width);
	else
		shared->steerCmd = steer_C * (shared->angle - shared->toMiddle / shared->track_width);//충돌각이 아니라면 중심을 유지하며 달리세요~
	
	
	
	/*
	앞 뒤 전체 차량을 스캔해서 내 차와 근접한 거리에 있다고 하면 
	다른 차로 계속 갈아타야 앞으로 갈 수 있을 듯 하다.
	일단 적절한 거리를 유지하게 하는 인수를 찾아서 좀 더 가까운 거리에도 
	따라가게 만들어야 함.
	갑자기 장애물이 나타나면 스티어값을 변경해주어야 함 ->그런고로 앞차 뒷차와의 거리와 
	도로중심사이의 차이를 계속 계산해서 특이점이 발견됐을 때 적절한 조치를 취해주어야 한다.*/

	
	double vOpt = vMax*(1 - exp(-1 * c / vMax * shared->dist_cars[index] - d));
	//cout << vOpt << endl;
	shared->accelCmd = (shared->speed < vOpt) ? 0.5 : 0;
	shared->brakeCmd = vBrake;
	shared->backwardCmd = GEAR_FORWARD;


	return 0;
}

void endShare(struct shared_use_st *&shared, HANDLE &hMap){
	// shared memory initialize
	if (shared != NULL)	{
		UnmapViewOfFile(shared);
		shared = NULL;
	}
	if (hMap != NULL) {
		CloseHandle(hMap);
		hMap = NULL;
	}
}

int main(int argc, char **argv){
	////////////////////// set up memory sharing
	struct shared_use_st *shared = NULL;

	// try to connect to shared memory 1
	HANDLE hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, false, SHARED_MOMORY_NAME1);
	if (hMap == NULL){
		fprintf(stderr, "Shared Memory Map open failed.\n");
		exit(EXIT_FAILURE);
	}

	shared = (struct shared_use_st*) MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(struct shared_use_st));
	if (shared == NULL){
		fprintf(stderr, "Shared Memory Map open failed.\n");
		exit(EXIT_FAILURE);
	}

	// shared memory 1 is already occupied.
	if (shared->connected == true) {
		endShare(shared, hMap);
		hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, false, SHARED_MOMORY_NAME2);
		if (hMap == NULL){
			fprintf(stderr, "Shared Memory Map open failed.\n");
			exit(EXIT_FAILURE);
		}
		shared = (struct shared_use_st*) MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(struct shared_use_st));
		if (shared == NULL){
			fprintf(stderr, "Shared Memory Map open failed.\n");
			exit(EXIT_FAILURE);
		}
	}
	printf("\n********** Memory sharing started, attached at %X **********\n", shared);

	////////////////////// DON'T TOUCH IT - Default Setting
	shared->connected = true;
	shared->written = 0;
	////////////////////// END Default Setting

	////////////////////// Initialize
	shared->steerCmd = 0.0;
	shared->accelCmd = 0.0;
	shared->brakeCmd = 0.0;
	shared->backwardCmd = GEAR_FORWARD;
	////////////////////// END Initialize

	while (shared->connected){
		if (shared->written == 1) { // the new image data is ready to be read
			controlDriving(shared);
			shared->written = 0;
		}

		if (_kbhit()){
			char key = _getch();
			if (key == 'q' || key == 'Q'){
				break;
			}
		}
	}

	endShare(shared, hMap);

	return 0;
}
