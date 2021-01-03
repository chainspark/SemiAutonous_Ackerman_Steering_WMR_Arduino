////
//FINAL VERSION <-- Arduino Controller Program Version 7.3
//TEAM (MECH 6621 - Mtl Drift)
//		||Members
//		||Chandan Satija
//		||Colby Neald
//		||Tasnim Haque
//		||Farshad Zaboli


#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/atomic.h>

#define BIT(a) (1 << (a))

//PWM
volatile unsigned int pin_motor;
volatile unsigned int pin_steering;
volatile unsigned int pin_ex;

volatile unsigned int pw_motor;
volatile int pw_steering;
volatile int r_motor;

unsigned int t_offset=25535;			//count offset for T = 20ms 

//Timer Counter 
volatile unsigned int time_counter=0;

//ADC
int channelA1 = 1;
int channelA3 = 3;
int channelA5 = 5;
volatile int complete; //will be a boolean character which determines if an ADC process is complete or not. 
volatile int a_in; //represents the analog input reading. 
int n = 4;

//Communication - These would be called in PID function and sent back to PC
volatile float t_clock;
volatile float wb,wf,y3;

//ADC Functions
void ADC_setup(int channel);
float ADC_read(int channel, int n);  
void ADC_configuration(int channel);
void ADC_complete(); 

//PWM Functions
void setup_pwm();
void setup_pwm_interrupt();
void setup_pwm_s1(unsigned int w1);
void setup_pwm_s2(unsigned int w2);
void setup_pwm_external();

//PID
void speed_PID_controller();
void setup_timer0();
void traction_PID_controller();
void brake_PID_controller();
void launch_PID_controller(); 
bool speed=0,traction=0,brake=0, launch = 0; 
volatile float tp = 0.0; 
volatile float ei = 0.0; 
volatile float ep = 0.0;

void setup()
{
	pin_motor = 7; 		// Pin 7 on board
	pin_steering = 0; 	// Pin 8 on board
	pin_ex = 2; 		// External interrupt pin for ulatrasonic sensor
	
	Serial.begin(9600);
	
	setup_pwm();
	
	setup_timer0();
	
	//Intialize the values to neutral
	pw_motor=1500;
	pw_steering=1500;
	
	setup_pwm_s1((unsigned int)pw_motor*2 + t_offset);		//setup pulse width of servo 1 (DC Motor)
	setup_pwm_s2((unsigned int)pw_steering*2 + t_offset);	//setup pulse width of servo 2 (Steering)
	
	//Communication variables
	const int NMAX = 64;
    static char buffer[NMAX];
    int i = 0, len,echo;
  
    len = 5; 			// Bytes of message recieved from PC 
	echo= 17; 			// Bytes of message sent back to PC
	
	//For serial recieve
    int *int_msg; 		// Pointer to read integers from message recived 
	
	//For serial send
	float *float_msg; 	//Pointer to send float data types to PC 
	int *int_rtn; 		//Pointer to send int data types to PC 

    ADC_configuration(channelA1);
    ADC_configuration(channelA3);
    ADC_configuration(channelA5);
  
	while(1)
	{
	
	// wait until new data is available
    // -> force blocking
   while( Serial.available() == 0 ) i++;

   // Read the bytes rseceived from the read function
   Serial.readBytes(buffer,len);

   int_msg = (int *)(buffer+1);
  
   //Atomic Access to avoid any updation error
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		r_motor=int_msg[0];  		//RPM
		pw_steering=int_msg[1];  	//Steering angle
   // restore status register including interrupt register
    }
   
	//Steering Servo Pulse 	
	setup_pwm_s2((unsigned int)pw_steering*2 + t_offset);	
	
  // Check front wheel rpm to decide on choice of controller
  float back_wheel,front_wheel,slip;
    float tol_slip = 1.0e-10;
    back_wheel = read_ADC(channelA1,20);
    front_wheel = read_ADC(channelA3,20);
   
  slip=(back_wheel - front_wheel) / (fabs(front_wheel) + tol_slip);
  
  if(r_motor > 0.00 && front_wheel < 400 ) //rad/s
    {
        //Clear Previous errors value before switching the controllers
        tp = 0.0; // previous time
        ei = 0.0; // integral state
        ep = 0.0;
        traction=0;
        launch = 1;
        speed =0;
        brake=0;
    }
  
    else if (slip > 1.00 && slip < -1.00)
    {
        //Clear Previous errors value before switching the controllers
        tp = 0.0; // previous time
        ei = 0.0; // integral state
        ep = 0.0;
        traction=1;
        launch=0;
        speed =0;
        brake=0;
    }
   
   else if (r_motor < 0 )
    {
        //Clear Previous errors value before switching the controllers
        tp = 0.0; // previous time
        ei = 0.0; // integral state
        ep = 0.0;
        traction=0;
        launch = 0;
        speed =0;
        brake=1;
    }
   
       else if (r_motor < 0 && front_wheel < -100 )
    {
        //Clear Previous errors value before switching the controllers
        tp = 0.0; // previous time
        ei = 0.0; // integral state
        ep = 0.0;
        traction=0;
        launch=0;
        speed =1;
        brake=0;
    }
    else
    {
        //Clear Previous errors value before switching the controllers
        tp = 0.0; // previous time
        ei = 0.0; // integral state
        ep = 0.0;
        traction=0;
        launch=0;
        speed =1;
        brake=0;
    }
	
	
	// Message sent back to PC 
	float_msg = (float *)(buffer+1);
	float_msg[0]=t_clock;
	float_msg[1]=wb;
	float_msg[2]=wf;
	//float_msg[3]=int_msg[0];
	
	int_rtn = (int *)(buffer+1+12);
	int_rtn[0]=r_motor;
	int_rtn[1]=pw_steering;
	
			
    // these lines send information back to the PC
    Serial.write(buffer,echo);
	
	
	}
	
  
}

void loop()
{
	
	// Do Nothing 
  	
}



ISR(TIMER1_COMPA_vect)
{
	//timer1 compare match A interrupt
	
	PORTD &= ~BIT(pin_motor);		//set pin 7 to LOW (end of pw1)
}

ISR(TIMER1_COMPB_vect)
{
	//timer1 compare match B interrupt
	
	PORTB &= ~BIT(pin_steering);	//set pin 8 to LOW (end of pw2)
}


ISR(TIMER1_OVF_vect)
{
	//timer1 overflow interrupt
	
	PORTB |=BIT(pin_steering);		//set pin 7 to HIGH (start of pw1)
	PORTD |=BIT(pin_motor);			//set pin 8 to HIGH (start of pw2)
	TCNT1 = 25535;					//set Timer1 count for desired T = 20ms
	time_counter++;					//timer counter
}

void setup_pwm(){
	//setup for pwm
	
	//1--configure DIO pins to OUTPUT
	DDRD |= BIT(pin_motor);			//servo 1 output to pin 7
	DDRB |= BIT(pin_steering);		//servo 2 output to pin 8
	
	//2--setup for timer1 interrupts for pwm
	setup_pwm_interrupt();			//setup of timer1 interrupt for pwm
}

void setup_pwm_interrupt(){
	//setup of timer1 interrupts for pwm
	
	cli();							//disable interrupts
	
	TCCR1A = 0;						//clear control registers
	TCCR1B = 0;
	
	TCCR1B |= BIT(CS11);			//set prescaler to 8, for best resolution
	
	TIFR1 |= BIT(OCF1A) | BIT(OCF1B) | BIT(TOV1);		//clear previous overflow
	TIMSK1 |= BIT(OCIE1A) | BIT(OCIE1B) | BIT(TOIE1);	//enable timer interrupt
	
	sei();
}

void setup_pwm_s1(unsigned int w1){
	OCR1A = w1;						//set pulse width 1 for pint 7
}

void setup_pwm_s2(unsigned int w2){
	OCR1B = w2;						//set pulse width 2 for pin 8
}

void setup_pwm_external(){
	//setup for s/w enabled external interrupt
	
	cli();							//disable interrupts
	
	DDRD |= BIT(pin_ex);			//output for SW generated interrupt
	EICRA |= BIT(ISC00) | BIT(ISC01);//set external interrupt control register A on rising edge INT0
	EIMSK = BIT(INT0);				//enable external interrupt
	
	sei();							//re-enable interrupts
}

void ADC_setup(int channel)
{
  cli(); //temporarily dissables interrupts. 
  
  //clear multiplexer selection registry.
  ADMUX = 0;

  //ADATE and ADEN must be clear while the channel is being switched.
  //normally these lines would come later, but a channel switch is about to occur so ADATE and ADEN need to be cleared.  
  ADCSRA = 0; 
  ADCSRB = 0;

  //select channel for the set-up.
  switch (channel) 
  {
    case 1: ADMUX |= BIT(MUX0); break; //channel becomes Pin A1.
    case 3: ADMUX |= BIT(MUX0) | BIT(MUX1); break; //channel becomes Pin A3. 
    case 5: ADMUX |= BIT(MUX0) | BIT(MUX2); break; //channel becomes Pin A5. 
    //default: Serial.print("\nError in channel selection.|n"); break; 
  }
  //delayMicroseconds(20); //delay for stability in switch.
  //switch function takes at most 8 us to complete. A delay of 20 us is used to include a safety factor.  

  //set the reference value for the ADC conversion. 
  ADMUX |= BIT(REFS0); //reference voltage is set to the Arduino voltage of 5 Vcc. 
  //the reference voltage is the higher voltage connected to the ground. 
  //In this case, that would be the voltage comming fromt the Arduino Uno board at 5 V. 

  //set the ADC registry. 

  //set the ADC pre-scaler. 
   ADCSRA = 0; //clears any defaul values stored in the ADC control and status registry A.
   ADCSRA |= BIT(ADPS1) | BIT(ADPS2); //pre-scaler of 64  

  //enable ADC processes.
  ADCSRA |= BIT(ADEN); //enables ADC processes; 

  //set the ADC to free-running mode.
 // ADCSRA |= BIT(ADATE); //enables auto trigger. 
 // ADCSRB = 0; //sets the ADC control and status registry B to free-running mode.
  //the ADC process will now continuously sample and update the ADC registry without a trigger event occuring. 

  //set interrupt.  
  ADCSRA |= BIT(ADIE); //enables ADC interrupts. ADC interrupts will occur when an ADC process has reached completion. 
  //if polling is used, and an average ADC voltage is used instead of a running average ADC voltage, then interrupts mays not be necissary. 
  
  //start an ADC process.
  ADCSRA |= BIT(ADSC); //begins an ADC process. Needed here if interrupts are used.  

  sei(); //re-enables interrupts. 
}

void ADC_complete() //function used to wait until an AD conversion is complete. 
{
  complete = 0;
  while (!complete) //while an ADC process is not complete.
  {
    ADCSRA |= BIT(ADSC); //continuously start a new conversion process while waiting for the last one. 
  }
   return; //return once complete. 
}

void ADC_configuration(int channel) //function used to configure the Arduino analog Pin during an ADC process. 
{
  int config = 0; //counts the number of configuration readings performed.
  int con_vol; //temporarily stores the ADC value after conversion. 
  ADC_setup(channel);
  //loop used to perform an intial 13 ADC processes. 
  while(config < 13)
  {
    ADC_complete(); //waits for an ADC process to be complete. 
    con_vol = ADC; //takes an ADC reading.   
    config++;
  } 
}
//entire configuration process takes roughly 1,500 us. This is a worthwhile tradeoff for having more accurate data.

float read_ADC(int channel, int n) //averaging ADC process. 
{
  //declare local variables
  int i = 1; //counter variable.
  long int sum = 0; 
  float a_vol; //average voltage.
  float conversion = 5.0/1023; //changes a reading from an integer (scale 0 - 1023) to a floating voltage value (0 V - 5 V). 
 
  ADC_setup(channel); //calls the ADC setup function. Configures the ADC settings for a given analog pin channel. 
  //ADCSRA |= BIT(ADSC); //begins an ADC process.
  
  //Configure the ADC.
  //ADC_configuration(channel); //performs 13 ADC processes to help configure the pin on the Arduino board.
  //13 cycles is recommended for configuring an ADC process on an Arduino Uno board becuase the first 13 readings are usually inaccurate.  
  
  //loop used to store the integer sum value of all ADC readings in a sample set. 
  while (i<n+1) 
  {
    ADC_complete(); //checks/waits for an ADC completion of one voltage reading. 
    sum = sum + a_in; //adds the ADC reading to the sum. 
    i = i+1;
  }  
  a_vol = (sum*conversion)/n; //the average voltage reading is calculated in volts (scale 0 V - 5 V). 
  
  return a_vol;
}

ISR(ADC_vect) //ADC interrupt function. 
{
  a_in = ADC; //stores the ADC reading in an analog input variable. 
  complete = 1; //indicates that a conversion process is complete. 
}


void setup_timer0()
{	
	//setup of timer0 interrupts for PID
	cli();
	TCCR0A = 0;							//clear control registers
	TCCR0B = 0;

	//Set Prescalar
	TCCR0B |= BIT(CS02); 					//set prescaler to 64,
	
	TIFR0 |=  BIT(OCF0B);	//clear previous overflow
	TIMSK0 |= BIT(OCIE0B);
	
	// Setup sampling time for PID
	// Timer 0 is 8 bit 
	// Least Count is 8us for prescalar 64
	// PID Functions takes around 1.2ms to complete
	// Therefore 2ms is taken as sampling time 
    OCR0B = 250;

	sei();
	
}

ISR(TIMER0_COMPB_vect)
{
	sei();
	if(traction==1)
	{
		traction_PID_controller();
	}
	else if(speed ==1)
	{
    speed_PID_controller();
	}
	else if (brake == 1)
	{
	brake_PID_controller();
	}
   else if (launch==1)
   {
   launch_PID_controller();
   }
	TCNT0 = 0;
}

void speed_PID_controller()
{
	// PID function local varaiable Variables 
	float dt; //local variable
	float reference, y; // local 
	float kp, ki, kd; //local variable
	float e, ed, z, ei_max; //local variable
	
	float u_V_DC; //local variable
		
	float readingA1=-1,readingA3=-1,readingA5=-1; //Local varaibles
	
	//Local variables
	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 810.0; // maximum back wheel speed (rad/s)	
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;

	unsigned int w1;//local variable	
	
	// PID controller gains
	kp = 40.00;
	ki = 10.00;
	kd = 3.00;
	
	//Read Time
	t_clock = TCNT1*0.0005 + time_counter*20;
	
	//Read ADC Values
	readingA1 = read_ADC(channelA1,n);
	readingA3 = read_ADC(channelA3,n);
	readingA5 = read_ADC(channelA5,n);
	
	// note that readingA1 (back wheel), readingA3 (right front wheel), and 
	// readingA5 (left front wheel) are the outputs for the car system
	
	// Convert Reading to actual things wb,wf,y3
	// use the following scale for the output reading1
	// w = 0 	-> readingA1 = 2.5 V
	// w = wmax -> readingA3 = 5.0 V
	// reading1 = 2.5 + w * wmax_inv * 2.5 ->
	
	// back wheel angular velocity (rad/s)
	wb = (readingA1 - 2.5) * 0.4 * wmax;

	// front wheel angular velocity (rad/s)
	wf = (readingA3 - 2.5) * 0.4 * wmax;
	
	y3 = readingA5;
	//y3 = (readingA5 - 2.5) * 0.4 * wmax; // Assuming two wheel car model
	
	// Get Reference values
	reference = (float)r_motor;// Reference RPM from int_msg[0]
	
	// dt to should be as small as possible , around 1ms or less
	// ADC Conversion should as fast or lower the average number of readings taken for averaging
	dt =t_clock-tp;
	tp = t_clock;
	
	//Speed Controller 
	y=wb;
	
	//Calculate e,ei,ed
	e = reference - y;
	ed = (e - ep) / dt;
	ep = e;

	//Anti-Windup
		if (ki > 0.0) {
			ei_max = 0.14*12.00 / ki; // want ki*ei < 0.14*V_batt
		}
		else {
			ei_max = 0.0;
		}
		if ((ei > ei_max) && (e > 0)) {
			// positive out of bounds, positive integration
			z = 0; // stop integration
		}
		else if ((ei < -ei_max) && (e < 0)) {
			// negative out of bounds, negative integration		
			z = 0; // stop integration
		}
		else { // either in bounds or integration reduces integral
			z = e; // normal integration
		}
		ei += z*dt;
	
	
	// PID controller
	u_V_DC = kp*e + ki*ei + kd*ed;

	if (u_V_DC >  V_bat) u_V_DC = V_bat;
	if (u_V_DC < -V_bat) u_V_DC = -V_bat;
	
	//write actuator
	
	//Convert Voltage to PWM
	// convert motor voltage to pulse width
	// u1 = (w1 - PW_0) * PW_R * V_bat ->
	w1 = u_V_DC * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
	
	// saturate input if out of range
	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;
	
	pw_motor = w1;
	
	//Set PWM for DC Motor 
	setup_pwm_s1((unsigned int)pw_motor*2 + t_offset);
	//END of controller function
	
}

void launch_PID_controller()
{
  // PID function local varaiable Variables 
  float dt; //local variable
  float reference, y; // local 
  float kp, ki, kd; //local variable
  float e, ed, z, ei_max; //local variable
  
  float u_V_DC; //local variable
  
  float readingA1=-1,readingA3=-1,readingA5=-1; //Local varaibles
  
  //Local variables
  const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
  const float wmax = 810.0; // maximum back wheel speed (rad/s) 
  const float V_bat = 12.0; // lipo battery voltage
  const float V_bat_inv = 1/V_bat;

  unsigned int w1;//local variable  
  float tol = 1.0e-10;
  // PID controller gains
  kp = 30.0;
  ki = 0.12;
  kd = 8.00;
  
  //Read Time
  t_clock = TCNT1*0.0005 + time_counter*20;
  
  //Read ADC Values
  readingA1 = read_ADC(channelA1,n);
  readingA3 = read_ADC(channelA3,n);
  readingA5 = read_ADC(channelA5,n);
  
  // note that readingA1 (back wheel), readingA3 (right front wheel), and 
  // readingA5 (left front wheel) are the outputs for the car system
  
  // Convert Reading to actual things wb,wf,y3
  // use the following scale for the output reading1
  // w = 0  -> readingA1 = 2.5 V
  // w = wmax -> readingA3 = 5.0 V
  // reading1 = 2.5 + w * wmax_inv * 2.5 ->
  
  // back wheel angular velocity (rad/s)
  wb = (readingA1 - 2.5) * 0.4 * wmax;

  // front wheel angular velocity (rad/s)
  wf = (readingA3 - 2.5) * 0.4 * wmax;
  
  y3 = readingA5;
  //y3 = (reading2 - 2.5) * 0.4 * wmax; // Assuming two wheel car model
  
  // Get Reference values
  reference = 7;
  
  // dt to should be as small as possible , around 1ms or less
  // ADC Conversion should as fast or lower the readings
  dt =t_clock-tp;
  tp = t_clock;
    
  //Traction Controller
  //y=slip
  y =  (wb- wf) / (fabs(wf) + tol);
  
  //Calculate e,ei,ed
  e = reference - y;
  ed = (e - ep) / dt;
  ep = e;
  
  //Anti-Windup
    if (ki > 0.0) {
      ei_max = 0.14*12.00 / ki; // want ki*ei < 0.14*V_batt
    }
    else {
      ei_max = 0.0;
    }
    if ((ei > ei_max) && (e > 0)) {
      // positive out of bounds, positive integration
      z = 0; // stop integration
    }
    else if ((ei < -ei_max) && (e < 0)) {
      // negative out of bounds, negative integration   
      z = 0; // stop integration
    }
    else { // either in bounds or integration reduces integral
      z = e; // normal integration
    }
    ei += z*dt;
  
  
  // PID controller
  u_V_DC = kp*e + ki*ei + kd*ed;

  if (u_V_DC >  V_bat) u_V_DC = V_bat;
  if (u_V_DC < -V_bat) u_V_DC = -V_bat;
  
  //write actuator
  
  //Convert Voltage to PWM
    // convert motor voltage to pulse width
  // u1 = (w1 - PW_0) * PW_R * V_bat ->
  w1 = u_V_DC * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
  
  // saturate input if out of range
  if(w1 > PW_MAX) w1 = PW_MAX;
  if(w1 < PW_MIN) w1 = PW_MIN;
  
  pw_motor = w1;
  
  //Set PWM for DC Motor 
  setup_pwm_s1((unsigned int)pw_motor*2 + t_offset);
  //END of controller function
  
}


void traction_PID_controller()
{
	// PID function local varaiable Variables 
	float dt; //local variable
	float reference, y; // local 
	float kp, ki, kd; //local variable
	float e, ed, z, ei_max; //local variable
	
	float u_V_DC; //local variable
	
	float readingA1=-1,readingA3=-1,readingA5=-1; //Local varaibles
	
	//Local variables
	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 810.0; // maximum back wheel speed (rad/s)	
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;

	unsigned int w1;//local variable	
	float tol = 1.0e-10;
	// PID controller gains
	kp = 30.0;
	ki = 0.12;
	kd = 8.00;
	
	//Read Time
	t_clock = TCNT1*0.0005 + time_counter*20;
	
	//Read ADC Values
	readingA1 = read_ADC(channelA1,n);
	readingA3 = read_ADC(channelA3,n);
	readingA5 = read_ADC(channelA5,n);
	
	// note that readingA1 (back wheel), readingA3 (right front wheel), and 
	// readingA5 (left front wheel) are the outputs for the car system
	
	// Convert Reading to actual things wb,wf,y3
	// use the following scale for the output reading1
	// w = 0 	-> readingA1 = 2.5 V
	// w = wmax -> readingA3 = 5.0 V
	// reading1 = 2.5 + w * wmax_inv * 2.5 ->
	
	// back wheel angular velocity (rad/s)
	wb = (readingA1 - 2.5) * 0.4 * wmax;

	// front wheel angular velocity (rad/s)
	wf = (readingA3 - 2.5) * 0.4 * wmax;
	
	y3 = readingA5;
	//y3 = (reading2 - 2.5) * 0.4 * wmax; // Assuming two wheel car model
	
	// Get Reference values
	reference = 7;
	
	// dt to should be as small as possible , around 1ms or less
	// ADC Conversion should as fast or lower the readings
	dt =t_clock-tp;
	tp = t_clock;
		
	//Traction Controller
	//y=slip
	y =  (wb- wf) / (fabs(wf) + tol);
	
	//Calculate e,ei,ed
	e = reference - y;
	ed = (e - ep) / dt;
	ep = e;
	
	//Anti-Windup
		if (ki > 0.0) {
			ei_max = 0.14*12.00 / ki; // want ki*ei < 0.14*V_batt
		}
		else {
			ei_max = 0.0;
		}
		if ((ei > ei_max) && (e > 0)) {
			// positive out of bounds, positive integration
			z = 0; // stop integration
		}
		else if ((ei < -ei_max) && (e < 0)) {
			// negative out of bounds, negative integration		
			z = 0; // stop integration
		}
		else { // either in bounds or integration reduces integral
			z = e; // normal integration
		}
		ei += z*dt;
	
	
	// PID controller
	u_V_DC = kp*e + ki*ei + kd*ed;

	if (u_V_DC >  V_bat) u_V_DC = V_bat;
	if (u_V_DC < -V_bat) u_V_DC = -V_bat;
	
	//write actuator
	
	//Convert Voltage to PWM
		// convert motor voltage to pulse width
	// u1 = (w1 - PW_0) * PW_R * V_bat ->
	w1 = u_V_DC * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
	
	// saturate input if out of range
	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;
	
	pw_motor = w1;
	
	//Set PWM for DC Motor 
	setup_pwm_s1((unsigned int)pw_motor*2 + t_offset);
	//END of controller function
	
}

void brake_PID_controller()
{
	// PID function local varaiable Variables 
	float dt; //local variable
	float reference, y; // local 
	float kp, ki, kd; //local variable
	float e, ed, z, ei_max; //local variable
	
	float u_V_DC; //local variable
	
	float readingA1=-1,readingA3=-1,readingA5=-1; //Local varaibles
	
	//Local variables
	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
	const float wmax = 810.0; // maximum back wheel speed (rad/s)	
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;

	unsigned int w1;//local variable	
	float tol = 1.0e-10;
	// PID controller gains
	kp = 8.00;
	ki = 0.01;
	kd = 30.0;
	
	//Read Time
	t_clock = TCNT1*0.0005 + time_counter*20;
	
	//Read ADC Values
	readingA1 = read_ADC(channelA1,n);
	readingA3 = read_ADC(channelA3,n);
	readingA5 = read_ADC(channelA5,n);
	
	// note that readingA1 (back wheel), readingA3 (right front wheel), and 
	// readingA5 (left front wheel) are the outputs for the car system
	
	// Convert Reading to actual things wb,wf,y3
	// use the following scale for the output reading1
	// w = 0 	-> readingA1 = 2.5 V
	// w = wmax -> readingA3 = 5.0 V
	// reading1 = 2.5 + w * wmax_inv * 2.5 ->
	
	// back wheel angular velocity (rad/s)
	wb = (readingA1 - 2.5) * 0.4 * wmax;

	// front wheel angular velocity (rad/s)
	wf = (readingA3 - 2.5) * 0.4 * wmax;
	
	y3 = readingA5;
	//y3 = (reading2 - 2.5) * 0.4 * wmax; // Assuming two wheel car model
	
	// Get Reference values
	reference = -4.0;
	
	// dt to should be as small as possible , around 1ms or less
	// ADC Conversion should as fast or lower the readings
	dt =t_clock-tp;
	tp = t_clock;
	
	
	//Brake Controller
	//y=slip
	y =  (wb- wf) / (fabs(wf) + tol);
	
	//Calculate e,ei,ed
	e = reference - y;
	ed = (e - ep) / dt;
	ep = e;
	
	//Anti-Windup
		if (ki > 0.0) {
			ei_max = 0.14*12.00 / ki; // want ki*ei < 0.14*V_batt
		}
		else {
			ei_max = 0.0;
		}
		if ((ei > ei_max) && (e > 0)) {
			// positive out of bounds, positive integration
			z = 0; // stop integration
		}
		else if ((ei < -ei_max) && (e < 0)) {
			// negative out of bounds, negative integration		
			z = 0; // stop integration
		}
		else { // either in bounds or integration reduces integral
			z = e; // normal integration
		}
		ei += z*dt;
	
	
	// PID controller
	u_V_DC = kp*e + ki*ei + kd*ed;

	if (u_V_DC >  V_bat) u_V_DC = V_bat;
	if (u_V_DC < -V_bat) u_V_DC = -V_bat;
 if(wf<12.5){
  u_V_DC =0;
 }
	
	//write actuator
	
	//Convert Voltage to PWM
	// convert motor voltage to pulse width
	// u1 = (w1 - PW_0) * PW_R * V_bat ->
	w1 = u_V_DC * V_bat_inv * (PW_MAX - PW_0) + PW_0; 
	
	// saturate input if out of range
	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;
	
	pw_motor = w1;
	
	//Set PWM for DC Motor 
	setup_pwm_s1((unsigned int)pw_motor*2 + t_offset);
	//END of controller function
	
}
