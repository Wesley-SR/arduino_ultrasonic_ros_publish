/****************************************************************
      SENSORS STATEMENTS
****************************************************************/
#define ultrasoundNum 6 // Total number of sensors
#define triggerInterval 40 // How many milliseconds between the measurement of each sensor; keep > 5ms
#define debugInterval 200 // How many milliseconds between each publication in serial monitor; keep > 200ms
#define soundSpeed 343.0 // Speed of sound in m/s

// Pins
const int triggerPin[ultrasoundNum] = {4, 5,  6,  7,  8,  9}; // OUTPUTS
const int echoPin[ultrasoundNum]   = {2, 3, 18, 19, 20, 21}; // INPUTS, Must be interrupts pins

// Variables
volatile unsigned long travelTime[ultrasoundNum]; // Place to store traveltime of the pusle
volatile unsigned long startTime[ultrasoundNum]; // Place to store trigger times (interrupt)
float distance[ultrasoundNum]; // Calculated distances in cm
unsigned long lastDebugMillis;
unsigned long triggerTimer[ultrasoundNum];



/****************************************************************
      SETUP
****************************************************************/
void setup(){

  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial
  }
  Serial.println("--- Serial monitor started ---");

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

      triggerTimer[i] +=  triggerInterval * ultrasoundNum; // Set next trigger for this sensor

      digitalWrite(triggerPin[i], HIGH);    // HIGH pulse for at least 10µs
      delayMicroseconds(10);
      digitalWrite(triggerPin[i], LOW);     // Set LOW again
    }
  }

  // Here is where the data is to be processed or printed out
  if (millis() - lastDebugMillis >= debugInterval)
  {
    noInterrupts(); // Pause interrupts
    
    doMeasurements(); // Retrieve measurements
    
    Serial.println();
    Serial.println("S__1     S__2     S__3     S__4     S__5     S__6    ");
    for (int i = 0; i < ultrasoundNum; i++) //ultrasoundNum; i++)
    {
      Serial.print(distance[i]);
      Serial.print(" --- ");
    }
    
    interrupts(); // Enable interrupts

    Serial.println();
    lastDebugMillis = millis();
  }
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
