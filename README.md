# SemiAutonous Ackerman Steering Arduino

The overall breakdown for microcontrollers software implementation for this project is to read keyboard input from a user, relay these inputs to a microcontroller using serial communication, get ADC sensor reading from the microcontroller, and calcualte control input and send PWM signals intended to change and control the hardware. 

The microprocessor used in project was Arduin Uno board. Arduino standard library functions were used only for serial communication between C++ program and sketch loaded on arduino. Interrupt Service Routine and Register Level programming was used for I/O Port configuration, Pulse Width Modulation and ADC coversion. 
