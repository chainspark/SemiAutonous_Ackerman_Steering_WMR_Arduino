
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <windows.h>

#include "serial_com.h"

using namespace std;

int open_serial(char *port_name, HANDLE &h, int speed)
// note the default for speed is 1 (115200 bps / baud)
// 0 - 9600 bps -- slow but very stable
// 1 - 115200 bps -- fast and stable
// 2 - 250000 bps -- some Arduinos can only go this fast
// 3 - 1000000 (1 Mbps)
// 4 - 2000000 (2 Mbps) -- max speed for some Arduino boards
// 5 - 5000000 (5 Mbps)
// 6 - 12000000 (12 Mbps) -- max Teensy 3.2 speed
// 7 - 480000000 (480 Mbps) -- max Teensy 3.5 / 3.6 speed (might require hardware changes)

// if you need high speeds then keep on increasing speed 
// for your application until you get garbage -- then 
// reduce for a reasonable factor of safety (i.e. 2-4x).
// note: the serial port driver should be set to the same data (baud) rate
// in the device manager com port settings.
// the uploader should also use the same rate.
{
	DCB param = {0};
	COMMTIMEOUTS CommTimeouts;
	char str[30] = "\\\\.\\";

	// typical Arduino speed settings
	DWORD BR[] = {CBR_9600, CBR_115200, 250000, 1000000, 2000000, 
		5000000, 12000000, 480000000 };

	cout << "\nInitializing serial port and resetting Arduino.\n\nPlease wait ...";

	// need the following form for COM > 9, "\\\\.\\COM10"
	// this form also works for COM < 10
	strcat(str,port_name);

    h = CreateFile(str, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
						FILE_ATTRIBUTE_NORMAL, NULL);

    if( h == INVALID_HANDLE_VALUE ) {
       cout << "\nerror opening serial port";
	   return 1; // error
	}

	// get current serial port parameters
	if( !GetCommState(h,&param) ) {
		cout << "\nerror: could not get serial port parameters";
		return 1;
	}

	param.StopBits    = ONESTOPBIT;
	param.Parity      = NOPARITY;
	param.ByteSize    = 8;
	
	if( speed < 0 || speed > 8 ) {
		cout << "\nerror: speed out of range [0 to 8]";
		return 1;
	}

	param.BaudRate = BR[speed];
	
	param.fDtrControl = DTR_CONTROL_ENABLE; // reset arduino with serial connection

	if( !SetCommState(h,&param) ) {
		cout << "\nerror: serial port parameters could not be set";
		return 1; // error
	} 
	
	// set serial port time-out parameters for a total timeout of 10s
	CommTimeouts.ReadIntervalTimeout = 0; // not used
	CommTimeouts.ReadTotalTimeoutMultiplier = 0; // total time-out per byte
	CommTimeouts.ReadTotalTimeoutConstant = 10000; // add constant to get total time-out
	CommTimeouts.WriteTotalTimeoutMultiplier = 0; // not used
	CommTimeouts.WriteTotalTimeoutConstant = 0; // not used

	if( !SetCommTimeouts(h,&CommTimeouts) ) {
		cout << "\nerror: serial port time-out parameters could not be set";
		return 1; // error
	}

	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	Sleep(100); // note: the extra delay helping / needed here
	
	// one more time to make sure Arduino got the "memo"
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	Sleep(3000); // wait for Arduino to fully reset

	cout << "\n\nSerial port and Arduino are ready.\n";

	return 0; // OK
}


int close_serial(HANDLE &h)
{
	if( h == INVALID_HANDLE_VALUE ) {
		cout << "\nerror closing serial port";
		return 1; // error
	}

	CloseHandle(h);
	h = INVALID_HANDLE_VALUE;

	return 0; // OK
}


int serial_recv(char *buffer, int n, HANDLE h)
// n - number of bytes desired/requested (0 < n <= nbuffer)
// note: this function will halt (ie block) the program as it waits for all n bytes.
// if you don't want to halt (block) then call n = serial_available() first
// and only call serial_recv (with n bytes) when n > 0.
// the time-out / blocking behaviour is invoked in my functions because
// I want to make sure the entire message is received before continuing the program
// and non-blocking can be easily achieved using the serial_available() function.
{
    DWORD nrecv;

	if( !ReadFile(h, (LPVOID)buffer, (DWORD)n, &nrecv, NULL) ) {
		cout << "\nerror: serial port read error";
		return 1; // error
	}
	
	if( nrecv != n ) {
		cout << "\nerror: serial port read time-out";
		return 1; // error
	}

	return 0; // OK
}


int serial_send(char *buffer, int n, HANDLE h)
{
    DWORD nsent; // number of bytes to be sent

    if( !WriteFile(h, (LPVOID)buffer, (DWORD)n, &nsent, NULL) ) {
		cout << "\nserial port write error";
		return 1; // error
	}

	return 0; // OK
}


int serial_available(HANDLE h)
// return the number of bytes available for reading from the serial port buffer
{
	COMSTAT status;
	DWORD errors;
	int nq; // number of bytes in serial port queue
	 
    // get serial port status
    ClearCommError(h, &errors, &status);

	nq = status.cbInQue;
    
	return nq;
}

