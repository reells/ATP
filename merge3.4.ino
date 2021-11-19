
// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the left
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1785
#define M_Y_MIN -4990
#define M_Z_MIN +358
#define M_X_MAX +1643
#define M_Y_MAX -1121
#define M_Z_MAX +7572

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

//ULTRASONIC
const byte RIGHTechoPin = 40;
const byte RIGHTtrigPin = 41;
const byte FRONTechoPin = 42;
const byte FRONTtrigPin = 43;
const byte LEFTechoPin = 44;
const byte LEFTtrigPin = 45;
long FRONTduration = 0;
long FRONTdistance = 0;
long RIGHTduration = 0;
long RIGHTdistance = 0;
long LEFTduration = 0;
long LEFTdistance = 0;
int distance = 0;

//IMU

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;
float rollAvg;
float pitchAvg;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {
    1, 0, 0
  }
  , {
    0, 1, 0
  }
  , {
    0, 0, 1
  }
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


float Temporary_Matrix[3][3] = {
  {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
  , {
    0, 0, 0
  }
};

//rcvOps
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;
bool startUp = true;
double xCoordRcv = 0;
double xCoord = 0;
double yCoordRcv = 0;
double yCoord = 0;
double zCoordRcv = 0;
double zCoord = 0;
bool camRcv = false;

//stepper
const byte stpPulOut = 9;
const byte stpDir = 8;
int stpPul = 0;
int pulsOut = 0;
bool calSet = false;
bool cal = false;
const byte grnBtn = 26;         // Green Button
bool grn_state = false;
bool stpEn = false;
bool calMsg = true;
bool calWait = true;
float yawOut;
float yawPreOut;
int turnAng;
const double pulToAng = 0.075;
double stpAng = stpPul * pulToAng;

//drive motor
const byte mtrEn2 = 7;
const byte mtrEn1 = 6;
const byte mtrSpd = 5;
bool mtrRunFwd = false;
int mtrAccel = 0;
double mtrFloor = 0;
double mtrMax = 0;
double mtrOut = 0;


//coord_builder
bool coordEntry = true;
bool coordBuilt = false;
bool coordRcv = false;
bool cycleCoords = false;
bool movCalcTgt = false;
bool IMUcalc = false;
double coordArrayX[8] = {};
double coordArrayY[8] = {};
int i = 0;
int j = 0;
int k = 0;
int l = 0;
double yCoordSto = 0;
double xCoordSto = 0;
double yDelta = 0;
double xDelta = 0;

//speedBack
double count = 0;
bool prox_state = false;

//PID
double K = 0;
double tI = 0;
double tD = 0;
double preErr = 0;
double reset = 0;
double output = 0;

//millis()
unsigned long startMillis = 0;
unsigned long currentMillis = 0;
unsigned long period = 1000;

//nav
const double rad2deg = (180 / M_PI); //radians to degrees
double hdgSet = 0;
double hdgInit = 0;
double distSet = 0;
double distTrav = 0;
double distLeft = 0;
float IMUhdg = 0;
float calOff = 0;

double hdgErr = 0;
double countPrev = 0;
double trigDist = (6 * M_PI) / 11;
double turnRad = 0;

volatile const byte proxTrig = 3;

//sendOps
int countNew = 0;


void setup() {
  //ULTRASONIC

  pinMode(RIGHTtrigPin, OUTPUT);
  pinMode(RIGHTechoPin, INPUT);
  pinMode(FRONTtrigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(FRONTechoPin, INPUT); // Sets the echoPin as an INPUT
  pinMode(LEFTtrigPin, OUTPUT);
  pinMode(LEFTechoPin, INPUT);


  //IMU
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(9600);

  pinMode (STATUS_LED, OUTPUT); // Status LED

  I2C_Init();

  digitalWrite(STATUS_LED, LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / 32;

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  //Serial.println("Offset:");
  for (int y = 0; y < 6; y++)
    Serial.println(AN_OFFSET[y]);

  delay(2000);
  digitalWrite(STATUS_LED, HIGH);

  timer = millis();
  delay(20);
  counter = 0;

  //Main
  pinMode(grnBtn, INPUT_PULLUP);
  pinMode(mtrSpd, OUTPUT);
  pinMode(mtrEn2, OUTPUT);  //low fwd, high rev
  pinMode(mtrEn1, OUTPUT);  //high fwd, low rev
  pinMode(stpPulOut, OUTPUT);
  pinMode(stpDir, OUTPUT);
  pinMode(proxTrig, INPUT_PULLUP);

  startMillis = millis();  //initial start time
  attachInterrupt(digitalPinToInterrupt(3), trigRcv, CHANGE);
  Serial.print('\n');
  Serial.println("Ready");
}

void loop() {
  //ULTRASONIC
  distance = getFRONTdistance();
  //IMU

  if ((millis() - timer) >= 20) // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer = millis();
    if (timer > timer_old)
    {
      G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;



    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter = 0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***

  }

  //Main
  recvWithStartEndMarkers();
  parseData();
  showParsedData();
  trigRcv();
  coordBuilder();
  coordExe();
  mtrOp();
  stepOp();
  nav();
  PID();
  E_Stop();
  sendOps();
}
