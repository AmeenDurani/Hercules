const int ROBOT_WIDTH_CM = 18;
const int ROBOT_LENGTH_CM = 30;
const int PICKUP_DIST = 23;

void configureAllSensors();
void objectDetection(int room_width, int room_length, int & num_pickup, float & end_distance_travelled, float & end_length_travelled, bool &valid);
void armPickUp(int mot_pow1, int mot_pow2);
float DriveDist(bool middle, bool & condition, int mot_pow, float distance); //returns distance travelled
void perimeter(float & room_width, float & room_length, int & num_pickup);
void Turn(bool orientation, int angle);
void go(int power);
void ExitProtocol(float length_travelled, float distance_travelled);

void space(float angle, float distance_cm, int &counter)
{

	bool temp = true;

	if (counter == 0)
	{
		int other_angle;
		Turn(0,angle);
		wait1Msec(100);
		other_angle = 90-angle;
		DriveDist(0, temp, 25, (distance_cm)/cos((other_angle)*PI/180.0));
		wait1Msec(100);
		Turn(1,angle);
		wait1Msec(100);
		go(-25);
		while(!SensorValue[S4])
		{}
		go(0);
		wait1Msec(100);
		Turn(1,90);
		wait1Msec(100);

		DriveDist(0, temp,-25,-13);
		wait1Msec(100);
		Turn(1,21.038);
		wait1Msec(100);
		DriveDist(0, temp,25,13.9284);
		wait1Msec(100);
		Turn(0,21.0384 + 1);
		counter++;
	}

	else
	{
		int other_angle;
		Turn(0,angle);
		wait1Msec(100);
		other_angle = 90-angle;
		DriveDist(0,temp, 25, (distance_cm-15.0)/cos((other_angle)*PI/180.0));
		wait1Msec(100);
		Turn(1,angle);
		wait1Msec(100);
		go(-25);
		while(!SensorValue[S4])
		{}
		go(0);
		wait1Msec(100);
		Turn(1,90);
		wait1Msec(100);

		DriveDist(0,temp,-25,-13);
		wait1Msec(100);
		Turn(1,21.038);
		wait1Msec(100);
		DriveDist(0,temp,25,10);
		wait1Msec(100);
		Turn(0,21.0384 );
		counter++;
	}

}

task main()
{
	configureAllSensors();
	wait1Msec(100);

	time1[T1] = 0;


	bool valid = true;
	int num_pickup = 0;
	float room_width = 0, room_length = 0;

	float end_distance_travelled = 0, end_length_travelled = 0;

	perimeter(room_width, room_length, num_pickup);
	objectDetection(room_width, room_length, num_pickup, end_distance_travelled, end_length_travelled, valid);


	if (valid == true){
		ExitProtocol(end_length_travelled, end_distance_travelled);
	}
	displayString(3, "%d", time1[T1]);
	wait1Msec(50000);

}

void objectDetection(int room_width, int room_length, int & num_pickup, float & end_distance_travelled, float & end_length_travelled, bool &valid){
	float distance_travelled = 0;
	float length_travelled = 0;
	bool condition = true;

	while((distance_travelled < room_width - ROBOT_LENGTH_CM - ROBOT_WIDTH_CM) && condition == true){

		if (SensorValue[S1] < (room_width - ROBOT_LENGTH_CM - ROBOT_WIDTH_CM - distance_travelled) && condition == true && num_pickup < 3){
			distance_travelled += abs(DriveDist(1, condition, -25, -(SensorValue[S1] - PICKUP_DIST)));

			if (SensorValue[S4] == 0){
				armPickUp(-30, 20);
				num_pickup += 1;
			}
			else{
				condition=false;
			}
		}
		else if (condition == true && num_pickup < 3){
			distance_travelled = DriveDist(1, condition, 25, distance_travelled);
			wait1Msec(1000);
			Turn(1, 90);
			wait1Msec(500);
			if((SensorValue[S1] > room_length - ROBOT_LENGTH_CM - 2*ROBOT_WIDTH_CM - length_travelled) && condition == true){

				length_travelled += DriveDist(1, condition, -25, -40);
				wait1Msec(1000);
				Turn(0, 90);
				distance_travelled = 0;
			}
			else if (condition == true){
				DriveDist(1, condition, 25, -length_travelled);
				condition = false;
				valid = false;

			}
		}
	}
	end_distance_travelled = distance_travelled;
	end_length_travelled = length_travelled;
}
void armPickUp(int mot_pow1, int mot_pow2)
{
	nMotorEncoder[motorC] = 0;
	int ENCLIM = -850;
	motor[motorC]=mot_pow1;
	while (abs(nMotorEncoder[motorC])<abs(ENCLIM))
	{}
	motor[motorC]=0;

	wait1Msec(100);

	nMotorEncoder[motorB] = 0;
	int ENCLIM2 = 80;
	motor[motorB]=mot_pow2;
	while (nMotorEncoder[motorB]<ENCLIM2)
	{}
	motor[motorB]=0;

	motor[motorC]=-mot_pow1;
	while (abs(nMotorEncoder[motorC])>0)
	{}
	motor[motorC]=0;

	motor[motorB]=-mot_pow2;
	while (abs(nMotorEncoder[motorB])>0)
	{}
	motor[motorB]=0;
}
void perimeter(float & room_width, float & room_length, int & num_pickup){

	int enc_width = 0, enc_length = 0;
	int counter = 0;
	bool temp = true;

	for (int count = 1; count <= 2; count++){
		nMotorEncoder[motorA] = 0;
		go(-25);
		while(SensorValue[S4] == 0)
		{
			if (SensorValue[S1] <=7 && SensorValue[S4] == 0 && num_pickup < 3)
			{
				DriveDist(0,temp,25, 11);
				armPickUp(-30, 20);
				num_pickup+=1;
				go(-25);
			}
		}
		go(0);


		if (count == 1){
			enc_width = abs(nMotorEncoder[motorA]);
			room_width = abs(nMotorEncoder[motorA]*2*PI*2.75/360.0);
		}
		else {
			enc_length = abs(nMotorEncoder[motorA]);
			room_length = abs(nMotorEncoder[motorA]*2*PI*2.75/360.0);
		}

		DriveDist(0, temp, 25, 3);
		space(45, 20, counter);
	}
	nMotorEncoder[motorA] = 0;
	go(-25);

	while (abs(nMotorEncoder[motorA]) <enc_width)
	{
		if (SensorValue[S1] <=7 && !SensorValue[S4] && num_pickup < 3)
		{


			DriveDist(0,temp,25, 12);
			armPickUp(-30, 20);
			num_pickup+=1;
			go(-25);
		}
	}
	go(0);

	Turn(1,90);

	nMotorEncoder[motorA] = 0;
	go(-25);
	while (abs(nMotorEncoder[motorA]) <enc_length)
	{
		if (SensorValue[S1] <=7 && !SensorValue[S4] && num_pickup < 3)
		{
			DriveDist(0,temp,25, 15);
			armPickUp(-30, 20);
			num_pickup+=1;
			go(-25);
		}
	}
	go(0);
	DriveDist(0,temp,25, ROBOT_WIDTH_CM);
	Turn(1, 90);
}


float DriveDist(bool middle, bool & condition,int mot_pow, float distance){

	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorD] = 0;

	const int ENC_LIM = distance*360/(2*PI*2.75);

	motor[motorA] = motor[motorD] = mot_pow;

	if (mot_pow < 0 && distance < 0){
		while(nMotorEncoder[motorA] > ENC_LIM && condition == true)
		{
			if (middle == 1 && SensorValue[S4] == 1){
				condition = false;
			}
		}
	}
	else{
		while((nMotorEncoder[motorA] + nMotorEncoder[motorD])/2 < ENC_LIM)
		{}
	}

	motor[motorA] = motor[motorD] = 0;
	return (((nMotorEncoder[motorA] + nMotorEncoder[motorD])/2)*2*PI*2.75/360);
}

void Turn(bool orientation, int angle){

	if (orientation == 1){ //CW
		motor[motorA] = -15;
		motor[motorD] = 15;
		int prev_angle = SensorValue[S2];

		while(SensorValue[S2] < prev_angle + angle - 5)
		{}
	}
	else{ //CCW
		motor[motorA] = 15;
		motor[motorD] = -15;
		int prev_angle = SensorValue[S2];

		while(SensorValue[S2] > prev_angle - angle + 6)
		{}
	}
	motor[motorA] = motor[motorD] = 0;
}

void ExitProtocol(float length_travelled, float distance_travelled){

	bool temp = false;

	DriveDist(0, temp, 25, distance_travelled);
	Turn(1, 90);
	DriveDist(0, temp, 25, -length_travelled);
}

void configureAllSensors(){

	SensorType[S4] = sensorEV3_Touch;
	SensorType[S1] = sensorEV3_Ultrasonic;
	SensorType[S2] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S2] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[S2] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
}
void go(int power)
{
	motor[motorA]=motor[motorD]=power;
}
