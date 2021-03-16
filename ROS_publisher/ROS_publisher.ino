/****************************************************************
      ROS STATEMENTS
****************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h> // Type of message to be published

//create an object which represents the ROS node.
ros::NodeHandle nh;

//Create three instances for range messages.
sensor_msgs::Range range_sensor_1;
sensor_msgs::Range range_sensor_2;
sensor_msgs::Range range_sensor_3;
sensor_msgs::Range range_sensor_4;
sensor_msgs::Range range_sensor_5;
sensor_msgs::Range range_sensor_6;

//Create publisher objects for all sensors
ros::Publisher pub_range_sensor_1("/ultrasound_1", &range_sensor_1);
ros::Publisher pub_range_sensor_2("/ultrasound_2", &range_sensor_2);
ros::Publisher pub_range_sensor_3("/ultrasound_3", &range_sensor_3);
ros::Publisher pub_range_sensor_4("/ultrasound_4", &range_sensor_4);
ros::Publisher pub_range_sensor_5("/ultrasound_5", &range_sensor_5);
ros::Publisher pub_range_sensor_6("/ultrasound_6", &range_sensor_6);


/****************************************************************
      SENSORS STATEMENTS
****************************************************************/
#define ultrasoundNum 6 // Total number of sensors
#define triggerInterval 40 // How many milliseconds between the measurement of each sensor; keep > 5ms
#define debugInterval 200 // How many milliseconds between each publication in ROS ; keep > 200ms
#define soundSpeed 343.0 // Speed of sound in m/s

// Pins
const int triggerPin[ultrasoundNum] = {4, 5,  6,  7,  8,  9}; // OUTPUTS
const int echoPin[ultrasoundNum]   = {2, 3, 18, 19, 20, 21}; // INPUTS, Must be interrupts pins

volatile unsigned long travelTime[ultrasoundNum]; // Place to store traveltime of the pusle
volatile unsigned long startTime[ultrasoundNum]; // Place to store trigger times (interrupt)
float distance[ultrasoundNum]; // Calculated distances in cm
unsigned long lastDebugMillis;
unsigned long triggerTimer[ultrasoundNum];



void sensor_msg_init(sensor_msgs::Range &range_name, char *frame_id_name)
{
  range_name.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_name.header.frame_id = frame_id_name;
  range_name.field_of_view = 0.26; // ~ 14.89 degree
  range_name.min_range = 0.2;
  range_name.max_range = 6.0;
}



/****************************************************************
      SETUP
****************************************************************/
void setup()
{
  // Init ROS node
  nh.initNode();

  // Set the ROS publishers
  nh.advertise(pub_range_sensor_1);
  nh.advertise(pub_range_sensor_2);
  nh.advertise(pub_range_sensor_3);
  nh.advertise(pub_range_sensor_4);
  nh.advertise(pub_range_sensor_5);
  nh.advertise(pub_range_sensor_6);

  // ROS topic setup
  sensor_msg_init(range_sensor_1, "/ultrasound_1");
  sensor_msg_init(range_sensor_2, "/ultrasound_2");
  sensor_msg_init(range_sensor_3, "/ultrasound_3");
  sensor_msg_init(range_sensor_4, "/ultrasound_4");
  sensor_msg_init(range_sensor_5, "/ultrasound_5");
  sensor_msg_init(range_sensor_6, "/ultrasound_6");

  setupMultipleUltrasoundIntervals();

  // Trigger pins
  pinMode(triggerPin[0], OUTPUT);
  pinMode(triggerPin[1], OUTPUT);
  pinMode(triggerPin[2], OUTPUT);
  pinMode(triggerPin[3], OUTPUT);
  pinMode(triggerPin[4], OUTPUT);
  pinMode(triggerPin[5], OUTPUT);

  // Echo pins
  pinMode(echoPin[0], INPUT); // Set interrupt pin 02 (INT0) as INPUT (sensor 1)
  pinMode(echoPin[1], INPUT); // Set interrupt pin 03 (INT1) as INPUT (sensor 2)
  pinMode(echoPin[2], INPUT); // Set interrupt pin 18 (INT2) as INPUT (sensor 3)
  pinMode(echoPin[3], INPUT); // Set interrupt pin 19 (INT3) as INPUT (sensor 4)
  pinMode(echoPin[4], INPUT); // Set interrupt pin 20 (INT4) as INPUT (sensor 5)
  pinMode(echoPin[5], INPUT); // Set interrupt pin 21 (INT5) as INPUT (sensor 6)

  // Interrupts for the Echo pins
  attachInterrupt(digitalPinToInterrupt(echoPin[0]), call_INT0, CHANGE ); // ISR for INT0
  attachInterrupt(digitalPinToInterrupt(echoPin[1]), call_INT1, CHANGE ); // ISR for INT1
  attachInterrupt(digitalPinToInterrupt(echoPin[2]), call_INT2, CHANGE ); // ISR for INT2
  attachInterrupt(digitalPinToInterrupt(echoPin[3]), call_INT3, CHANGE ); // ISR for INT3
  attachInterrupt(digitalPinToInterrupt(echoPin[4]), call_INT4, CHANGE ); // ISR for INT4
  attachInterrupt(digitalPinToInterrupt(echoPin[5]), call_INT5, CHANGE ); // ISR for INT5

  lastDebugMillis = millis();
}



/****************************************************************
      LOOP
****************************************************************/
void loop()
{

  // trigger times are managed here
  for (uint8_t i = 0; i < ultrasoundNum; i++){
    if (millis() >= triggerTimer[i]){

      triggerTimer[i] +=  triggerInterval * ultrasoundNum;
      
      digitalWrite(triggerPin[i], HIGH);    // HIGH pulse for at least 10µs
      delayMicroseconds(10);
      digitalWrite(triggerPin[i], LOW);     // Set LOW again
    }
  }

  // Here is where the data is to be processed or published out
  if (millis() - lastDebugMillis >= debugInterval)
  {

    noInterrupts();

    doMeasurements(); // Retrieve measurements
    
    range_sensor_1.range = distance[0];
    range_sensor_2.range = distance[1];
    range_sensor_3.range = distance[2];
    range_sensor_4.range = distance[3];
    range_sensor_5.range = distance[4];
    range_sensor_6.range = distance[5];

    // Update time stamp
    range_sensor_1.header.stamp = nh.now();
    range_sensor_2.header.stamp = nh.now();
    range_sensor_3.header.stamp = nh.now();
    range_sensor_4.header.stamp = nh.now();
    range_sensor_5.header.stamp = nh.now();
    range_sensor_6.header.stamp = nh.now();

    // Publish in each ROS sensor topic
    pub_range_sensor_1.publish(&range_sensor_1);
    pub_range_sensor_2.publish(&range_sensor_2);
    pub_range_sensor_3.publish(&range_sensor_3);
    pub_range_sensor_4.publish(&range_sensor_4);
    pub_range_sensor_5.publish(&range_sensor_5);
    pub_range_sensor_6.publish(&range_sensor_6);
    interrupts();

    lastDebugMillis = millis();
  }
  nh.spinOnce();//Handle ROS events
}

/****************************************************************
      Retrieve measurement
****************************************************************/
void doMeasurements()
{
  // First read will be 0 (no distance  calculated yet)
  
  // Calculates the speed of all sensors
  for (int i = 0; i < ultrasoundNum; i++)
  {
    distance[i] = travelTime[i] / 2.0 * (float)soundSpeed / (1000000.0);   // in meters
  }
}



/****************************************************************
      ISR - Interrupt Service Routine
****************************************************************/
// The PIN register is used to read the digital value of the pin.

// INTerrupt number 0 (pin 2 on Mega)
void call_INT0()
{
  byte pinRead;
  pinRead = PINE & B00010000; // Read pin 2/PE4
  interruptHandler(pinRead, 0);
}


// INTerrupt number 1 (pin 3 on Mega)
void call_INT1()
{
  byte pinRead;
  pinRead = PINE & B00100000; // Read pin 3/PE5
  interruptHandler(pinRead, 1);
}


// INTerrupt number 2 (pin 18 on Mega)
void call_INT2()
{
  byte pinRead;
  pinRead = PIND & B00001000; // Read pin 18/PD3
  interruptHandler(pinRead, 2);
}


// INTerrupt number 3 (pin 19 on Mega)
void call_INT3()
{
  byte pinRead;
  pinRead = PIND & B00000100; // Read pin 19/PD2
  interruptHandler(pinRead, 3);
}


// INTerrupt number 4 (pin 20 on Mega)
void call_INT4()
{
  byte pinRead;
  pinRead = PIND & B00000010; // Read pin 20/PD1
  interruptHandler(pinRead, 4);
}


// INTerrupt number 5 (pin 21 on Mega)
void call_INT5()
{
  byte pinRead;
  pinRead = PIND & B00000001; // Read pin 21/PD0
  interruptHandler(pinRead, 5);
}



// Common function for interrupts
void interruptHandler(bool pinState, int nIRQ)
{
  unsigned long currentTime = micros();  // Get current time (in µs)
  if (pinState)
  {
    // If pin state has changed to HIGH -> remember start time (in µs)
    startTime[nIRQ] = currentTime;
  }
  else
  {
    // If pin state has changed to LOW -> calculate time passed (in µs)
    travelTime[nIRQ] = currentTime - startTime[nIRQ];
  }
}



/****************************************************************
      Auxiliaries functions
****************************************************************/
void setupMultipleUltrasoundIntervals() {
  triggerTimer[0] = millis() + 75; // First trigger start in ms.
  for (uint8_t i = 1; i < ultrasoundNum; i++)
  triggerTimer[i] = triggerTimer[i - 1] + triggerInterval;
}


// END
