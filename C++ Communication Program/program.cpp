////
//FINAL VERSION <-- Communication Program
//TEAM (MECH 6621 - Mtl Drift)
//		||Members
//		||Chandan Satija
//		||Colby Neald
//		||Tasnim Haque
//		||Farshad Zaboli


#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <conio.h>

#include <windows.h>

#include "serial_com.h"

#include "timer.h"

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

int main()
{

    //Declare variable to be mapped with keys from keyboard
	short int angle_pulse_width, wb_ref; //RPM is RPM of back wheel (Not the Motor)
	short int max_angle_pulse_width, min_angle_pulse_width, max_rpm, max_rpm_reverse; //Boundaty Limits
	char message[64];
	short int *int_msg; // Store in Hybrid array that would be shared with arduino

	int_msg = (short int*)(message+1);

	// Select Mode Feature - We can have different Modes by sending first byte of message as a character e.g = d mean dry road condition,w means wet road condition
	message[0] = 'w';

	//Define increment step for Keyboard input
	short int rpm_step = 50; //rad/s - Max Limit is 810 rad
	short int angle_pulse_width_step = 50; // Aprrox step for increase/decrease of angle by 0.9 degrees 

	//Initial Input 
	angle_pulse_width = 1500; //Zero Steering pulse width

	wb_ref = 0; //Zero RPM for back wheel

	// Out of bounds check values (The are the limiting values i.e. pressing keys won't go beyond these values - Each key has defined limiting values)
	min_angle_pulse_width = 1000;
	max_angle_pulse_width = 2000;

	max_rpm = 810;
	max_rpm_reverse = -810; 
	
	//Send message to arduino

	HANDLE h1; //object for serial communication lib
	const int NMAX = 64; // Arduino buffer size 
	char buffer[NMAX];
	char rec[NMAX]; // Message Recived from Arduino
	int speed;
	short int *int_rec; //Arduino int is 2 bytes
	float *float_rec;

	speed = 0; // Communication Rate - 9600 baud/s for max stability in communication

	// Enter COM Number over here 
	open_serial("COM6", h1, speed);

	int len; // Byte of Message that is sent to arduino

	int echo; // Byte of Message that is recieved from arduino

	ofstream fout("my_sim.csv");

	float time, readingA1, readingA3, readingA5; // reading are Arduino Analog pin values

	if (!fout) {
		cout << "\nerror opening output file";
	}

	// label the columns (the % is so matlab plotting m-files ignore this line)
	fout << "%time(millis),wb,wf,Arduino r_motor,Arduino pw_steering,wb_ref,angle_pulse_width\n";


	while (1)
	{
		if (KEY('X')) break;

		if (KEY(VK_UP)) {
			if (wb_ref <= max_rpm - rpm_step) wb_ref += rpm_step;
			int_msg[0] = wb_ref;
			cout << "\nMoving Forward " << "\t" << int_msg[0];
		}
		else if (KEY(VK_DOWN)) {
			if (wb_ref >= max_rpm_reverse + rpm_step) wb_ref -= rpm_step;
			int_msg[0] = wb_ref;
			cout << "\nMoving Backward " << "\t" << int_msg[0];
		}
		else int_msg[0] = wb_ref; //Retain prev rpm of back wheel

		if (KEY(VK_LEFT)) {
			if (angle_pulse_width <= max_angle_pulse_width - angle_pulse_width_step) angle_pulse_width += angle_pulse_width_step;
			int_msg[1] = angle_pulse_width;
			cout << "\nSteering Left " << "\t" << int_msg[1];
		}
		else if (KEY(VK_RIGHT)) {
			if (angle_pulse_width >= min_angle_pulse_width + angle_pulse_width_step) angle_pulse_width -= angle_pulse_width_step;
			int_msg[1] = angle_pulse_width;
			cout << "\nSteering Right" << "\t" << int_msg[1];
		}
		else int_msg[1] = angle_pulse_width; // Retain prev angle value
		
		
		//Fetch information and put into buffer message that will sent to arduino
		buffer[0] = message[0]; // Character for Mode
		buffer[1] = message[1]; // u1 = wb_ref
		buffer[2] = message[2];
		buffer[3] = message[3]; // u2 = Angle
		buffer[4] = message[4];

		//Control Message Refresh Rate 
		Sleep(10);

		// Send Data to Arduino
		len = 5;
		serial_send(buffer, len, h1);

		cout << "Mode \t";
		cout << message[0];
		cout << "\n";
		cout << "Motor rpm(rad/s) is \t";
		cout << int_msg[0];
		cout << "\n";
		cout << "Steering is \t";
		cout << int_msg[1];
		cout << "\n";

		cout << "Sending message to arduino" << "\n";

		// Recieve Data from Arduino

		echo = 17;//Recieve 17 bytes
		serial_recv(rec,echo, h1); 

		float_rec = (float*)(rec + 1);

		time = float_rec[0];
		readingA1 = float_rec[1];
		readingA3 = float_rec[2];

		cout << "Time(millis) \t";
		cout << float_rec[0];
		cout << "\n";

		cout << "Channel wb \t";
		cout << float_rec[1];
		cout << "\n";

		cout << "Channel wf \t";
		cout << float_rec[2];
		cout << "\n";

		int_rec = (short int*)(rec + 1 + 12);

		// For Debugging purpose only - To check Arduino is reading correct message or not 
		cout << "Message Read by Arduino r_motor \t";
		cout << int_rec[0];
		cout << "\n";

		cout << "Message Read by Arduino pw_steering \t";
		cout << int_rec[1];
		cout << "\n";

		//Recieve message from arduino (Serial Monitor)
		cout << "Message recieved from arduino" "\n";

		fout << time << "," << readingA1 << "," << readingA3 << "," << int_rec[0] << "," << int_rec[1] << "," << wb_ref << "," << angle_pulse_width << "\n";

	}

	// close output file
	fout.close();

	close_serial(h1);

	return 0;

}