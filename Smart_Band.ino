//------------------Dependencies------------------------//
#include "FastLED.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <RtcDS3231.h>
#include <U8x8lib.h>

//------------------Constants-----------------------------//
// Pins
#define INTERRUPT_PIN 2
//#define INTERRUPT_PIN 7 // 7 for Leonardo, using #5 int
#define LED_PIN 3
//#define LED_PIN 8 // 8 for leonardo
#define BLUETOOTH_STATE_PIN 4
#define BUTTON_PIN 5
#define VIBRATOR_PIN 6

// Interface state
#define CONNECTION_STATE 0
#define TIME_DATE 1
#define STEP_AMOUNT 2
#define SLEEP_QUALITY 3
#define BIO_STATE 4
#define MAX_DISPLAY_STATE 5

// Step count
#define MAX_LENGTH 50
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
// Others
#define NUM_LEDS 1
#define countof(a) (sizeof(a) / sizeof(a[0]))
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

//-----------------Initialize sensors-----------------------//
CRGB leds[NUM_LEDS]; // Initialize the LED array
U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8; // Initialize the OLED screen
RtcDS3231<TwoWire> Rtc(Wire);
MPU6050 mpu(0x69);

//-----------------Initialize vars & functions-------------------//
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion qu;      		  // Quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];
float ypr[3];  			    // Yaw, Pitch, Roll
volatile bool mpuInterrupt = false;
void dmpDataReady() {
	mpuInterrupt = true;
}
// RTC vars & function
RtcDateTime now; //
char* now_char;
char* printDateTime(const RtcDateTime& dt);
// Interface vars & function
bool button_down = false;
char display_state = -1;
char pre_display_state = -1;
void pre_state_check(U8X8_SSD1306_128X32_UNIVISION_HW_I2C);
// Bluetooth connection
bool bluetooth_connection = false;
void bluetooth_connection_checker();
void vibrator(bool);
// Step count
// Base on the result of this webpage http://www.analog.com/cn/analog-dialogue/articles/pedometer-design-3-axis-digital-acceler.html by Neil Zhao
// Initialize containers in order to store samples in three axes
float x_accel[MAX_LENGTH];
float y_accel[MAX_LENGTH];
float z_accel[MAX_LENGTH];
// Dynamic thresholds
// As they can update every 50 samples, they can provide much more accurate measure compare to static thresholds
// With their ability to evolve, they can quickly accommodate to their present environment
float* dynamic_thresholds = [0,0,0]; // 0 -> x, 1 -> y, 2 -> z
// Dynamic precisions
// To avoid high frequency noise due to unknown reasons
// Due to their ability to evolve every 50 samples just like the thresholds, these precisions can self-evolve to accommodate the environment
float* dynamic_precisions = [0,0,0]	// 0 -> x, 1 -> y, 2 -> z
short counter = 0;					// Init counter for storing samples
long lastsample_time = 0;
float threshold_calculator(float *);
float precision_calculator(float *);
float precision_checker(float *, float, float);
float * max_change(float *, float *, float *);
//-------------------------------------------Main Function------------------------------------------------------//
//-------------------------------------------Initialization-----------------------------------------------------//
void setup() {
	Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  //Start initialization
  Serial.println("System initializing...");
  //Serial.println("Initializing modules...");
  Wire.begin();
    Serial.println("System initializing...");
  Wire.setClock(400000);
    Serial.println("System initializing...");
  u8x8.begin();
   Serial.println("System initializing...");
  FastLED.addLeds<WS2811, LED_PIN, RGB>(leds, NUM_LEDS);
  Rtc.Begin();
  //mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // Empty buffer

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    /*	Serial.print(F("DMP Initialization failed (code "));
    	Serial.print(devStatus);
    	Serial.println(F(")"));	*/
    }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(VIBRATOR_PIN, OUTPUT);
  pinMode(BLUETOOTH_STATE_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //Serial.println("Initialization completed");
  //Finish initialization
}

//----------------------------------------Main Loop---------------------------------------------//
void loop() {
  //----------------------------------------Get data from sensors---------------------------------//
  // Get results from MPU6050
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize)
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
    	while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    	mpu.getFIFOBytes(fifoBuffer, packetSize);
    	fifoCount -= packetSize;
    	mpu.dmpGetQuaternion(&qu, fifoBuffer);
    	mpu.dmpGetGravity(&gravity, &qu);
    	mpu.dmpGetAccel(&aa,fifoBuffer);
    	mpu.dmpGetLinearAccel(&aaReal,&aa,&gravity);	// Get acceleration
    	mpu.dmpGetYawPitchRoll(ypr, &qu, &gravity);		// Get YawPitchRoll
    /*Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);*/

	}
	if(counter != 0){
		// Adding new samples
		x_accel[counter] = precision_checker(x_accel, aaReal.x, x_dynamic_precision);
		y_accel[counter] = precision_checker(y_accel, aaReal.y, y_dynamic_precision);
		z_accel[counter] = precision_checker(z_accel, aaReal.z, z_dynamic_precision);
		// Check if there is any step

	}

  // Check for bluetooth connection
  //Serial.println(bluetooth_connection);
  bluetooth_connection_checker();
  //----------------------------------------Decode serial data---------------------------------//

  //----------------------------------------Display--------------------------------------------//
  // Check if button was down
  if (digitalRead(BUTTON_PIN) == HIGH && !button_down) {
    display_state++; // Change state
    button_down = true;
}

  else if (digitalRead(BUTTON_PIN) == LOW && button_down) button_down = false; // Check if already released the button!
   Serial.println("System initializing...");
  //Serial.println(bluetooth_connection);
  // Switching states
  if (display_state == MAX_DISPLAY_STATE) display_state = CONNECTION_STATE; // Switch back
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  /*Serial.print("State: ");
    Serial.print(display_state);
    Serial.print("\t Pre: ");
    Serial.println(pre_display_state);*/
    switch (display_state) {
    	case CONNECTION_STATE:
    		pre_state_check(u8x8);
      		u8x8.home();// Reset cursor
      		u8x8.print("Connection: ");
      		u8x8.print(bluetooth_connection);
      	break;
      	case TIME_DATE:
      		pre_state_check(u8x8);
      		u8x8.home();// Reset cursor
      		now = Rtc.GetDateTime();
      		now_char = printDateTime(now);
      		u8x8.print("Time: \n");
      		u8x8.print(now_char);
      	break;
    	case STEP_AMOUNT: // steps
    		pre_state_check(u8x8);
      		u8x8.home();// Reset cursor
      		u8x8.print("Today's steps: ");
      	break;
    	case SLEEP_QUALITY: // quality & quantity
    		pre_state_check(u8x8);
      		u8x8.home();// Reset cursor
      		u8x8.print("Last 24hrs\nsleeptime: ");
      	break;
    	case BIO_STATE: // heart rate
    		pre_state_check(u8x8);
      		u8x8.home();// Reset cursor
      	break;
  	}

}

//----------------------------------Functions--------------------------------//
void pre_state_check(U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled) {
	if (pre_display_state != display_state) {
		pre_display_state = display_state;
		oled.clear();
    //Serial.print("Check");
  	}//Can not use oled.home();
}

char* printDateTime(const RtcDateTime& dt){
	char datestring[20];
	snprintf_P(datestring,
		countof(datestring),
		PSTR("%02u/%02u/%04u\n%02u:%02u:%02u"),
		dt.Month(),
		dt.Day(),
		dt.Year(),
		dt.Hour(),
		dt.Minute(),
		dt.Second() );
  	//Serial.println(datestring);
  	return datestring;
}

void bluetooth_connection_checker() {
	if (digitalRead(BLUETOOTH_STATE_PIN) == HIGH) {
		bluetooth_connection = true;
	}
	else {
		if (bluetooth_connection==true) {
			vibrator(true);
      	//Serial.println("Lost connection");
      	display_state = CONNECTION_STATE;
      	}
      	bluetooth_connection = false;
      	vibrator(false);
  	}
}

void vibrator(bool on) {
	if (on){
		digitalWrite(VIBRATOR_PIN,HIGH);
		delay(500);
	}
	else{
		digitalWrite(VIBRATOR_PIN, LOW);
	}
}

float threshold_calculator(float * data){
	float total = 0;
	for(int i = 0; i<MAX_LENGTH; i++ ){
		total+=data[i];				// Count the total
	}
	return total/float(MAX_LENGTH); // Avoid integer divison
}

float precision_calculator(float * data){
	float total_change = 0;
	for(int i = 1; i < MAX_LENGTH; i++){
		total_change+=abs(data[i]-data[i-1]);	// Acquire total change
	}
	return total_change/float(MAX_LENGTH);		// Avoid integer divison
}

float precision_checker(float * data, float new_data, float precision){ // Avoid noise
	// The new_data is valid only if the change in acceleration is greator than the precision
	// If the new_data is invalid, discard it and return the data[counter-1]
	if(abs(new_data-data[counter-1]) > precision) return new_data;
	else return data[counter-1];
}

float * max_change(float * x, float * y, float * z,){
	max(max(, ),);
}