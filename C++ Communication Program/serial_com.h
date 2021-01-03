
int open_serial(char *port_name, HANDLE &h, int speed = 1);
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

int close_serial(HANDLE &h);

// function will block until n bytes are received
int serial_recv(char *buffer, int n, HANDLE h);

int serial_send(char *buffer, int n, HANDLE h);

// use this for non-blocking reveive
int serial_available(HANDLE h);

