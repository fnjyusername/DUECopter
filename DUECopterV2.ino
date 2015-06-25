Enter file contents here

 /*
V2 -DRIFT CORRECTION 
Important to set the proper orientation
of the IMU in relation to the Quadcopter intended
orientation and identifying the sign (negative/posotive) 
convention of the imu as it turns about the axis.Write it
down, it will be helpfull in debugging strange motor response.

It uses two Microcontroller, Arduino DUE and Trinket 3v/12Mhz
The Trinket captures the RC Receiver and send to DUE via RX1, The DUE process
the Sensors, PID, and Motor Output. 

The Configurations are as follow:
3S Lipo Battery 3000-4000mah to Power Motor
3S Lipo Battery 1000mah to power the Microcontrollers
1100kv Motor
9" x 45pitch 
Uses 450 Frame
Total Weight 1.50kg
*/
//////////////////////////////
//  IMU Bank Orientation    //
//        -X  +X            //
//           |              //
//    -Y     |    -Y        //
//       ---------          //
//    +Y     |    +Y        //
//           |              //  
//        -X  +X            //
//       Pitch about Y      //
//       Rolls about X      //
//////////////////////////////
#include "Arduino.h"
#include "Math.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#define Divider_100HZ 4
#define Divider_50HZ 8
#define Divider_20HZ 20
#define Divider_10HZ 40
#define Divider_5HZ 80
#define Divider_2HZ 200
#define Divider_1HZ 400



float  OutputAtt_y, OutputGyr_y, OutputGyr_x, OutputAtt_x;
#define MOTOR1 0 //Pin  34  (Not used)
#define MOTOR2 1 //Pin  36  (Not used)
#define MOTOR3 2 //Pin  38  (Not used)
#define MOTOR4 3 //Pin  40  (Not used)
#define MOTOR5 4 //Pin  9  Actual Motor 1 RF1 FRONR RIGHT QUAD X
#define MOTOR6 5 //Pin  8  Actual Motor 2 LF2 FRONT LEFT  QUAD X
#define MOTOR7 6 //Pin  7  Actual Motor 3 LB3 BACK  LEFT  QUAD X
#define MOTOR8 7 //Pin  6  Actual Motor 4 RB4 BACK  RIGHT QUAD X
#define MINPULSE 1100
#define MAXPULSE 2000
#define PWM_PERIOD 2500 

volatile byte numberOfMotors = 4;
int motorCommand[8] = {0,0,0,0,0,0,0,0};  // LASTMOTOR not know here, so, default at 8 @todo : Kenny, find a better way
  
void initializeMotors(byte numbers = 4);
void writeMotors();


/**********RC Definition**************/
#define USE_PPM 0 //#define USE_PPM 1
volatile uint16_t PPMt[16]; // unvalidated input
volatile uint16_t PPM[16];
volatile uint8_t  PPMch = 255;
volatile uint32_t PPMlast=0;
volatile uint32_t Interr=1;
/*********IMU Definition**************/


#define LED_PIN 13 
MPU6050 mpu;
// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
float last_x_angle=0;  // These are the filtered angles
float last_y_angle=0;
float last_z_angle=0;  
float last_gyro_x_angle=0;  // Store the gyro angles to compare drift
float last_gyro_y_angle=0;
float last_gyro_z_angle=0;




//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

float    GYRO_FACTOR;// This global variable tells us how to scale gyroscope data
float    ACCEL_FACTOR;// This global varible tells how to scale acclerometer data

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Buffer for data output
char dataOut[256];

//ALL MY DEFINITIONS:
int Setpoint=0, n=0;  byte DelimRc, Delim; boolean ESC_ARMED=false, statusESC=false;

/*IMU STABILIZATION VARIABLES */
float Acc_x, Acc_y, Acc_z;
float Gyr_x, Gyr_y, Gyr_z;
float Rol_x, Pit_y, Yaw_z;
float  Mag_z; float Zmap; 

/*Define RATE and ATTITUDE Mode by stick movement*/
bool AI_Rate=false, EL_Rate=false;

/*PID VARIABLES*/
//Y-Axis
float Iay=0, Py_Error_rateOld=0, SetPitchAbout_Yold=0, Output_Y=0;
//X-Axis
float Iax=0, Px_Error_rateOld=0, SetRollsAbout_Xold=0, Output_X=0;
//Z-Axis
float SetRuddrAbout_Z=0, SetRuddrAbout_Zold=0, error_Z=0, lastErr_Z=0, Pz_Error_rate=0, Pz_Error_rateOld=0, Iaz=0, dErr_Z, Output_Z=0;
float Acc_Zprev=0, errSum_Z=0;
float Adrift=0, Gdrift=0;
float Acc_ZnowOld=0;
//Time Variable
volatile unsigned long  lastTime=0, now=0; 
volatile unsigned long  dt_loop=0; 
//OUTPUT Variable
volatile float AI_Pulse, EL_Pulse, TH_Pulse, RD_Pulse;
volatile float RF1, LF2, LB3, RB4;

boolean StickCenter=true;
//http://www.w//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://www.w//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://www.w//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://www.w//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID Gains
//float Kp_rateRoll = 1.02;//2.78 1.18 5.28
//float Ki_rateRoll = 0.82;//2.75
//float Kd_rateRoll = 0.065;//0.085 0.025 - 0.045
static float Output_YMax=250.0, Output_YMin=-250.0;
static float Output_XMax=250.0, Output_XMin=-250.0;

float mapxlim=250, mapylim=250, mapzlim=150;  float deadBand=0.50, Itakeoff=1200, PIDtakeoff=1170;
float tauZ=0.125, tau=0.125; float kpxx=0.40; float kpyy=0.40; float kpzz=4.0;float kg=1.00;
float Iax_Limit=120.0, Iay_Limit=120.0, Iaz_Limit=50;


float  kpy=1.60,        kiy=0.010,          kdy=kpy*0.025;    
float  kpx=1.60,        kix=0.010,          kdx=kpx*0.025;  
float  kpz=2.00,        kiz=0.000,          kdz=kpz*0.005;
//http://www.w////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://www.w////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://www.w////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//http://www.w////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



float Th=0, Ave=0;
void setup() 
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif  
  
pinMode(LED_PIN, OUTPUT);pinMode(22, OUTPUT); pinMode(23, OUTPUT);
//Serial Initialization
Serial.begin(115200); Serial1.begin(115200); delay(500);//give time for serial to initialized 
while (Serial.available() && Serial.read()); delay(500);// empty buffer 
while (Serial1.available() && Serial1.read()); delay(500);// empty buffer 

initializeMotors(numberOfMotors);
//calibrateESC();
//commandAllMotors(MINPULSE);

/*********************START OF MPU 6050 CONFIG********************************/ 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/*
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 
 MPU6050_GYRO_FS_250         0x00  (Default see .cpp file on MPU6050::initialize() )
 MPU6050_GYRO_FS_500         0x01
 MPU6050_GYRO_FS_1000        0x02
 MPU6050_GYRO_FS_2000        0x03
 
 MPU6050_ACCEL_FS_2          0x00  (Default see .cpp file on MPU6050::initialize() )
 MPU6050_ACCEL_FS_4          0x01
 MPU6050_ACCEL_FS_8          0x02
 MPU6050_ACCEL_FS_16         0x03
*/
    //A. GYRO SETTINGS
        // Set the full scale range of the GYROSCOPE (FS_SEL is 0 TO 3 see table above)
        //Match FS_SEL and LSB from above
        uint8_t FS_SEL = 0, LSB = 131;      
        mpu.setFullScaleGyroRange(FS_SEL);delay(100);
                
        //Read and check actual settings
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange(); delay(50); 
        Serial.print("GyroRange = ");
        Serial.println(READ_FS_SEL);
        GYRO_FACTOR = LSB/(FS_SEL + 1);
               
     //B. ACCEL SETTINGS    
        // Set the full scale range of the ACCELEROMETER (FS_SEL is 0 TO 3 see table above)
        //Match FS_SEL and LSB from above     
        uint8_t AFS_SEL = 3;
        mpu.setFullScaleAccelRange(AFS_SEL);delay(100);
                
        //Read and check actual settings
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange(); delay(50); 
        Serial.print("AccelRange = ");
        Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
      //C. Read High pass filter
         READ_AFS_SEL = mpu.getDHPFMode(); delay(50);     
        Serial.print("DHPFMode = ");
        Serial.println(READ_AFS_SEL);     

      //C. Read Low pass filter     
         mpu.setDLPFMode(2);
         READ_AFS_SEL = mpu.getDLPFMode(); delay(50);     
        Serial.print("DLPFMode = ");
        Serial.println(READ_AFS_SEL);          

     //C. CALIBRATION
      calibrate_sensors();
/*******END OF MPU 6050 CONFIG********************************/  


// Configure LED Indicator
digitalWrite(LED_PIN, LOW);
digitalWrite(22, LOW);digitalWrite(23,LOW);
}


uint16_t pulseCounter=0; 
int start = 0;
void loop() 
{
       now=micros();
       dt_loop = (now- lastTime);
    
        if (dt_loop >= 2500)
           {//Start 400 Hz
            lastTime = micros();  pulseCounter++;    
              smoothRCinput();  //Extract RC
              loopIMU(dt_loop); //Extract IMU  
              Compute_Elevator(dt_loop);   // Calculate for Output_Y
              Compute_Aileron(dt_loop);    // Calculate for Output_X
              Compute_Rudder(Acc_z, dt_loop);   // Calculate for Output_Z  
              loopMotor(); // Calculate Value for Motor PWM input to writeMotors();
              writeMotors(); Acc_ZnowOld=Acc_z;
              //PWM Motor Output  
              /*
              Serial.print(Gyr_x);  
              Serial.print('\t');
              Serial.print(Gyr_z); 
              Serial.println();*/ 
              /*if(Gyr_x>=1.0||Gyr_x<=-1.0) {Serial.print(Gyr_y); Serial.print("\t"); Serial.println(Gyr_x);}*/
             //Serial.println(dt_loop);
             //digitalWrite(LED_PIN,LOW);          
         }//End 400 Hz loop   
//Serial.print(AI_Pulse); Serial.print("\t");Serial.print(EL_Pulse); Serial.print("\t");Serial.print(TH_Pulse); Serial.print("\t"); Serial.println(RD_Pulse);      
 
}//End Main Loop


/****************************************************************************/
/************************(RC PPM RECEIVER FONBCTIONS)*****************************/
/****************************************************************************/
void smoothRCinput()
{
  if (Serial1.available()>0);
  {
    Delim = Serial1.read();

    if (Delim==58){AI_Pulse=ailRCEMA(Serial1.parseInt());}
    if (Delim==59){EL_Pulse=eleRCEMA(Serial1.parseInt());}             
    if (Delim==60){TH_Pulse=thrRCEMA(Serial1.parseInt());} 
    if (Delim==61){RD_Pulse=rudRCEMA(Serial1.parseInt());}       
  }//If
}//End

/****************************************************************************/
/****************************MPU FUNCTIONS***********************************/
/****************************************************************************/
float emaX_ail=0, emaX_ele=0, emaX_thr=0, emaX_rud=0;
float emaX=0, emaY=0, emaZ=0, emaXg=0, emaYg=0, emaZg=0;
float Gyr_xp=0, Gyr_yp=0, Gyr_zp=0;float get_last_time=0;
const float alphax = 0.20; //0 to 1
void loopIMU(unsigned long  dt_imu)
{ float dt=dt_imu*0.000001; //micro to seconds
  
////////////////////////////////////START IMU DATA WXTRACTION////////////////////////////////////////
const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        unsigned long t_now = millis();
        
        //STORE IMU DATA TO FLOAT VARIABLE
        double axx = ax ;//Serial.print(ax); Serial.print("\t");
        double ayy = ay ;//Serial.print(ay); Serial.print("\t");
        double azz = az ;//Serial.println(az);              
        //Normalise the measurements
        double R = sqrt(axx*axx + ayy*ayy + azz*azz); //Serial.println(R/16384);// R^2 = Rx^2 + Ry^2 + Rz^2  Pythegorian  

        float  Ax = axx/R ;//Serial.print(axx); Serial.print("\t");
        float  Ay = ayy/R ;//Serial.print(ayy); Serial.print("\t");
        float  Az = azz/R ;//Serial.println(azz);
          
        // Remove offsets and scale gyro data  
        float gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
        float gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
        float gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
        float accel_x = Ax; // - base_x_accel;
        float accel_y = Ay; // - base_y_accel;
        float accel_z = Az; // - base_z_accel;
        

              
        float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
        float accel_angle_z = 0;

        // Compute the (filtered) gyro angles
        
        float gyro_angle_x = gyro_x*dt + last_x_angle;
        float gyro_angle_y = gyro_y*dt + last_y_angle;
        float gyro_angle_z = gyro_z*dt + last_z_angle;
        
        
                           
         // Apply the complementary filter to figure out the change in angle - choice of alpha is
        // estimated now.  Alpha depends on the sampling rate...
        const float alpha = 0.98;
        float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x; last_x_angle = angle_x; 
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y; last_y_angle = angle_y;
        float angle_z = gyro_angle_z; last_z_angle = angle_z;  //Accelerometer doesn't give z-angle
        angle_z = alpha*(angle_z + gyro_z*dt)+(1.0 - alpha)*gyro_angle_z;
              if (angle_z>0)angle_z=angle_z-0.00005; 
              if (angle_z<0)angle_z=angle_z+0.00003; 
        // Compute the drifting gyro angles
        float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle; last_gyro_x_angle = unfiltered_gyro_angle_x;
        float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle; last_gyro_y_angle = unfiltered_gyro_angle_y;
        float unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle; last_gyro_z_angle = unfiltered_gyro_angle_z;     
       
        

///////////////////////////////////////////END IMU DATA WXTRACTION///////////////////////////////////////////////

////////////////////////////////////START IMU DATA USAGE AND MANUPULATION////////////////////////////////////////

                    Acc_x= angle_x+0.00; 
                    Acc_y=-angle_y-0.40;                   
                    Acc_z=-angle_z;      

                    
                    if       (Acc_x>45)  {Acc_x=45;}
                    else if  (Acc_x<-45) {Acc_x=-45;}
                    else     {Acc_x=Acc_x;}
                    
                    if       (Acc_y>45)  {Acc_y=45;}
                    else if  (Acc_y<-45) {Acc_y=-45;}
                    else     {Acc_y=Acc_y;}  
                    
                    if (Acc_x>=-0.20 && Acc_x<=0.20) {digitalWrite(23, HIGH);} else {digitalWrite(23, LOW);}
                    if (Acc_y>=-0.20 && Acc_y<=0.20) {digitalWrite(22, HIGH);} else {digitalWrite(22, LOW);}
                    
                    Gyr_x=gyro_x;Gyr_x=gxEMA(Gyr_x);  //Rolls Gyro + Right    -Left about X (See Axis top page)

                    Gyr_y=-gyro_y;Gyr_y=gyEMA(Gyr_y); //Pitch Gyro + Forward   -Backward about Y (See Axis top page)                          

                    Gyr_z=gyro_z;Gyr_z=gzEMA(Gyr_z); //Pitch Gyro -CW +CCW about Z (See Axis top page)                     
                  
                    /*DISPLAY
                      Serial.print(Gyr_x);  
                      Serial.print('\t');                   
                      Serial.print(Gyr_y);
                      Serial.print('\t');
                      Serial.print(Gyr_z);                      
                      Serial.print('\t');
                      Serial.print(Acc_y); 
                      Serial.print('\t');
                      Serial.print(Acc_z);                       
                      Serial.println();*/                   


////////////////////////////////////END IMU DATA USAGE AND MANUPULATION////////////////////////////////////////       
}


void loopMotor()
{
  //1 deg = 0.0174532925 rad
  float Ax=Acc_x*0.0174532925; 
  float Ay=Acc_y*0.0174532925; 
  
  float ThX=(TH_Pulse/cos(Ax))-TH_Pulse;
  float ThY=(TH_Pulse/cos(Ay))-TH_Pulse;
  Th=sqrt(sq(ThX)+sq(ThY));
  if (Th>15) {Th=15.0;}
  float zTh=abs(Gyr_z);
  if (zTh>10) {zTh=10.0;}


   float Throttle=0.0+0.0+TH_Pulse;   
   if (Throttle>1700) Throttle=1700;
    
    RF1 = Throttle + (-Output_Y*0.7071)  + (+Output_X*0.7071) + (-Output_Z) -  0.0;//Pin 6  FRONT RIGHT Motor 1 (to Probe1) - Green
    LF2 = Throttle + (-Output_Y*0.7071)  + (-Output_X*0.7071) + (+Output_Z) +  0.0;//Pin 7   FRONT LEFT  Motor 2 (to Probe2) - White
    
    //BACK MOTOR 
    RB4 = Throttle + (+Output_Y*0.7071)  + (+Output_X*0.7071) + (+Output_Z) -  0.0; //Pin 8  BACK RIGHT Motor 4(to Probe3) - Black
    LB3 = Throttle + (+Output_Y*0.7071)  + (-Output_X*0.7071) + (-Output_Z) +  0.0; //Pin 9  BACK LEFT Motor  3(to Probe4) - Orange
    
    Ave = ( RF1 + LF2 + RB4 + LB3 ) * 0.25;
    //Serial.print(Acc_x);Serial.print("\t");Serial.print(Gyr_x);Serial.print("\t");Serial.println(TH_Pulse);
}



//PID Calculation Elevator  
void Compute_Elevator(unsigned long dty)
{ dty =dty*0.001;     
float YSetpoint_rate, SetPitchAbout_Y, Att_error_Y, Py_Error_rate, Att_dErr_Y, OutAtti_y;
float erRangeY=deadBand;

  //ERROR BY STICK COMMAND  
  if (EL_Pulse>1400|| EL_Pulse<1380)
  {EL_Rate=true;
    SetPitchAbout_Y = -map(EL_Pulse,1390,1770,0,mapylim); // rolls output range 0 - 8 degrees  "AI_Pulse"  
  }else {SetPitchAbout_Y=0; EL_Rate=false;}

   if   (Acc_y<=erRangeY && Acc_y>=-erRangeY) {Acc_y=0;}
   else if   (Acc_y>erRangeY)  {Acc_y=Acc_y-erRangeY;}   
   else if   (Acc_y<-erRangeY) {Acc_y=Acc_y+erRangeY;}    

  //P.I.D Activation
   if (TH_Pulse>PIDtakeoff)
   {//P.I. Start only at Throttle >1170  
 /****************************RATE****************************************/               
          Att_error_Y = Acc_y - SetPitchAbout_Y;         
          YSetpoint_rate = (tau*YSetpoint_rate/(tau+dty)) + ((SetPitchAbout_Y - SetPitchAbout_Yold)/(tau+dty));
          SetPitchAbout_Yold = SetPitchAbout_Y;
         
         //RATE P + LAG COMPENSATOR:          
          Py_Error_rate = YSetpoint_rate + Att_error_Y * kpyy + kg*Gyr_y; 
          
          
          //DERIVATIVE OF RATE P
          Att_dErr_Y = (tau*Att_dErr_Y)/(tau+dty) + (Py_Error_rate - Py_Error_rateOld) / (tau+dty);
          Py_Error_rateOld = Py_Error_rate;
          
          //INTEGRAL OF RATE P
              if(TH_Pulse>Itakeoff)
              {Iay = Iay + kiy*Py_Error_rate*dty;} else{Iay=0;} 
              if  (Iay>=Iay_Limit || Iay<=-Iay_Limit) {Iay -= kiy*Py_Error_rate*dty;}//Stop integration at limits                        
        //REVISED PID      
              OutAtti_y = kpy*Py_Error_rate + Iay + kdy*Att_dErr_Y;
         
         //Final Combined Output          
          OutputAtt_y=OutAtti_y;


/****************************OUTPUT-Y****************************************/      
      if ((OutputAtt_y <= Output_YMax) && (OutputAtt_y >Output_YMin)){Output_Y=OutputAtt_y;}
        else if  (OutputAtt_y>Output_YMax)  {Output_Y=Output_YMax;}//Stop integration at limits
        else if  (OutputAtt_y<Output_YMin)  {Output_Y=Output_YMin;}//Stop integration at limits      
        
        
/*******************************DEBUG*****************************************/
        //Serial.print(Att_error_Y);Serial.print("\t");Serial.print(kpy*Py_Error_rate);Serial.print("\t");Serial.print(Iay);Serial.print("\t");Serial.print(kdy*Att_dErr_Y);Serial.print("\t");Serial.print(Output_Y);Serial.println();
        //Serial.flush();
  }
   else
  {  
   OutAtti_y=0;Output_Y=0;   
   Py_Error_rate=0;
   Py_Error_rateOld=0;
   Iay=0;
   Att_dErr_Y=0;
   dty=0;
   YSetpoint_rate=0;
   Att_error_Y=0;
  }//End of if TH_Pulse   
}


//PID Calculation Elevator
float erRangeX=deadBand;
void Compute_Aileron(unsigned long dtx)
{dtx =dtx*0.001; 
float XSetpoint_rate, SetRollsAbout_X, Att_error_X, Px_Error_rate, Att_dErr_X, OutAtti_x;
float erRangeX=deadBand;
 
  if (AI_Pulse>1400 || AI_Pulse<1380)
  {AI_Rate=true;
    SetRollsAbout_X = -map(AI_Pulse, 1390,1770,0,mapxlim); // pitch output range 0 - 8 degrees  "EL_Pulse"
  }else {SetRollsAbout_X=0;AI_Rate=false;}

   if   (Acc_x<=erRangeX && Acc_x>=-erRangeX) {Acc_x=0;}
   else if   (Acc_x>erRangeX)  {Acc_x=Acc_x-erRangeX;}   
   else if   (Acc_x<-erRangeX) {Acc_x=Acc_x+erRangeX;}  
   
   
  //P.I.D Activation
    if (TH_Pulse>PIDtakeoff)
    {
/****************************RATE****************************************/      
          Att_error_X = Acc_x - SetRollsAbout_X;         
          XSetpoint_rate = (tau*XSetpoint_rate/(tau+dtx)) + ((SetRollsAbout_X - SetRollsAbout_Xold)/(tau+dtx));
          SetRollsAbout_Xold = SetRollsAbout_X;
         
         //RATE P + LAG COMPENSATOR:          
         Px_Error_rate = XSetpoint_rate + Att_error_X * kpxx + kg*Gyr_x; 
         
          //DERIVATIVE OF RATE P
          Att_dErr_X = (tau*Att_dErr_X)/(tau+dtx) + (Px_Error_rate - Px_Error_rateOld) / (tau+dtx);
          Px_Error_rateOld = Px_Error_rate;
          
          //INTEGRAL OF RATE P 
              if (TH_Pulse>Itakeoff)
                {Iax = Iax + kix*Px_Error_rate*dtx;}else{Iax=0;}   
                if  (Iax>=Iax_Limit || Iax<=-Iax_Limit) {Iax = Iax - kix*Px_Error_rate*dtx;}//Stop integration at limits    
        
        //REVISED PID       
         float OutAtti_x = kpx*Px_Error_rate + Iax + kdx*Att_dErr_X;
         
         //Final Combined Output          
          OutputAtt_x=OutAtti_x;
          
/****************************OUTPUT-X ****************************************/          
      if ((OutputAtt_x <= Output_XMax) && (OutputAtt_x >=Output_XMin)){Output_X=OutputAtt_x;} 
          else if  (OutputAtt_x>Output_XMax)  {Output_X= Output_XMax;}//Stop integration at limits
          else if  (OutputAtt_x<Output_XMin)  {Output_X= Output_XMin;}//Stop integration at limits

/*******************************DEBUG******************************************/    
      //Serial.print(Acc_x);Serial.print("\t");Serial.print(kpx*Px_Error_rate);Serial.print("\t");Serial.print(Iax);Serial.print("\t");Serial.print(kdx*Att_dErr_X); Serial.print("\t");Serial.print(Output_X);Serial.println();

   }
     else
     {
     OutAtti_x=0;Output_X=0;   
     Px_Error_rate=0;
     Px_Error_rateOld=0;
     Iax=0;
     Att_dErr_X=0;
     dtx=0;
     XSetpoint_rate=0;
     Att_error_X=0;
     }//End of if TH_Pulse

}


/* 
GYRO Z CONVENTION: 
-Gyr_z is Positive Turn Left 
+Gyr_z is Negative Turn Right

      YAW MOTOR SPEED CONTROL
    TURN LEFT       TURN RIGHT
    +  L   -         -  R   +
     \    /           \    /
      \  /             \  /
       \/               \/
       /\               /\
      /  \             /  \
     /    \           /    \
    -      +         +      -
*/

//PID Calculation Rudder
void Compute_Rudder(float Acc_Znow, unsigned long dtz)
{static float Output_ZMax=200, Output_ZMin=-200; bool Center;
        float error_Z;float erRangeZ=0.10;
        float dtzi=dtz*0.0001; dtz=dtz*0.001; 
        
    if (TH_Pulse>PIDtakeoff)
    {//1.1 ERROR IDENTIFICATION
          //IDENTIFY SETPOINT AT CENTER STICK
         if (RD_Pulse>=1385 && RD_Pulse<=1410 )
          {//1.1.1 SETPOINT AT CENTER STICK
               if (n<1)
               {//One time Capture
                SetRuddrAbout_Z=Acc_Znow;
                Acc_Zprev = Acc_Znow; error_Z=0;
                Adrift=0; n=+1;
               }//catch the 1st value once Stick at center range    
           
           /////////////////////////DRIFT COMPENSATOR///////////////////////
            Gdrift= abs(Gyr_z-Acc_Zprev); 
            Adrift= abs(Acc_Znow-Acc_Zprev); 

                                
              if (Adrift<=0.002)
              {                  
                SetRuddrAbout_Z=Acc_Znow; Acc_Zprev=Acc_Znow;n=+1; //Serial.print("J ");
                error_Z=0;errSum_Z=0;            
              }
              else {SetRuddrAbout_Z=Acc_Zprev; n=+1;//Serial.print("K ");
              }  
  
          
           Zmap=0;
           error_Z = Acc_z-SetRuddrAbout_Z;
          } 
          //1.1.2 SETPOINT AT MOVING STICK          
          else 
          { n=0;Center=0;
           Zmap =map(RD_Pulse,1390, 1770, 0, mapzlim);         
           SetRuddrAbout_Z=Acc_z-Zmap;                   
           error_Z = Acc_Znow-SetRuddrAbout_Z;//Serial.print("B \t");Serial.println(RD_Pulse);             
          }
                    
      //1.2 ERROR IDENTIFICATION                                       
             /*Compute all the working error variables*/ 
           float ZSetpoint_rate = (0.065*ZSetpoint_rate/(0.065+dtz)) + ((SetRuddrAbout_Z - SetRuddrAbout_Zold)/(0.065+dtz));
           SetRuddrAbout_Zold = SetRuddrAbout_Z;
           
           //RATE P + LAG COMPENSATOR: 
           Pz_Error_rate = ZSetpoint_rate + error_Z * kpz - Gyr_z; 
           //DERIVATIVE OF RATE P
           dErr_Z = (tauZ*dErr_Z)/(tauZ+dtz) + (Pz_Error_rate - Pz_Error_rateOld) / (tauZ+dtz);
           Pz_Error_rateOld = Pz_Error_rate;
           //INTEGRAL OF RATE P
                       
           Iaz = Iaz + kiz*Pz_Error_rate*dtzi;   
           if  (Iaz>=Iaz_Limit || Iaz<=-Iaz_Limit) {Iaz -= kiz*Pz_Error_rate*dtz;}//Stop integration at limits  
           
           float OutAtti_z = kpzz*Pz_Error_rate + Iaz + kdz*dErr_Z; 
                 
          if       (OutAtti_z>Output_ZMax)  { Output_Z=Output_ZMax; }
          else if  (OutAtti_z<Output_ZMin)  { Output_Z=Output_ZMin; }
          else     {Output_Z=OutAtti_z;}
          //Serial.print(Acc_Znow);Serial.print("\t");Serial.print(error_Z);Serial.print("\t");Serial.print(Acc_z);Serial.print("\t"); Serial.println(SetRuddrAbout_Z);
          //Serial.print(Acc_z);Serial.print("\t"); Serial.print(ZSetpoint_rate);Serial.print("\t"); Serial.print(error_Z);Serial.print("\t"); Serial.print( - Gyr_z);Serial.print("\t"); Serial.print(Pz_Error_rate);Serial.print("\t");Serial.println(OutAtti_z);
          //Serial.print(error_Z);Serial.print('\t');Serial.print(Acc_Znow);Serial.print('\t');Serial.print(Acc_Zprev);Serial.print('\t');Serial.print(Output_Z);Serial.println();          

          //Record last value
          lastErr_Z = error_Z;  
     }
     else
     {
       Acc_Zprev=Acc_z;
       Acc_Znow=Acc_z; 
       SetRuddrAbout_Z=Acc_z; 
       error_Z=0;
       Output_Z = 0 ;  
       Iaz=0; Pz_Error_rate=0;
       lastTime = now;// Lasttime need to be updated to avoid big gap
       lastErr_Z=0;   
       errSum_Z=0;           //Revert to 0 
       dErr_Z=0;             //Revert to 0 
     }//End of if TH_Pulse>1170
  
}//End Compute Rudder



 
float readCompass(float readNow, float SetRuddr)
 {float eRR;   
  if (abs(readNow-SetRuddr)<=180)  
       {eRR = readNow - SetRuddr;}
   else 
        if (SetRuddr<readNow)
        {eRR = readNow - SetRuddr - 360; }
        else 
        {eRR = readNow - SetRuddr + 360; }  
   return eRR; 
  }// end readCompass


////////////////////////////////////////////////////////////////////////
////////////////////Motor Initialization ///////////////////////////////
/////Configures Pins, Channel, and PWM Period///////////////////////////
////////////////////////////////////////////////////////////////////////
void initializeMotors(byte numbers) {

    setPWMpin(9);  //PWM L4 or  //Pin  9
    setPWMpin(8);  //PWM L5 or  //Pin  8
    setPWMpin(7);  //PWM L6 or  //Pin  7
    setPWMpin(6);  //PWM L7 or  //Pin  6
    
/*
    PWMC_DisableChannel(PWM, 0); //Pin 34
    PWMC_DisableChannel(PWM, 1); //Pin 36
    PWMC_DisableChannel(PWM, 2); //Pin 38  
    PWMC_DisableChannel(PWM, 3); //Pin 40
    PWMC_DisableChannel(PWM, 4); //Pin  9
    PWMC_DisableChannel(PWM, 5); //Pin  8
    PWMC_DisableChannel(PWM, 6); //Pin  7
    PWMC_DisableChannel(PWM, 7); //Pin  6
*/

  pmc_enable_periph_clk(ID_PWM);

  // set PWM clock A to 1MHz
  PWMC_ConfigureClocks(1000000,0,VARIANT_MCK);
  
    configOneMotor(4, PWM_PERIOD); //PWM L4 or  //Pin  9
    configOneMotor(5, PWM_PERIOD); //PWM L5 or  //Pin  8
    configOneMotor(6, PWM_PERIOD); //PWM L6 or  //Pin  7
    configOneMotor(7, PWM_PERIOD); //PWM L7 or  //Pin  6
  
  //Start all motor with minimum pulse  
  commandAllMotors(MINPULSE);
}


////////////////////////////////////////////////////////////////////////
////////////////////Motor Realtime Write Command////////////////////////
///////motorCommand[MOTORx] - is Realtime PWM value for each motor//////
///////////////// called in void loop()/////////////////////////////////
////////////////////////////////////////////////////////////////////////

//Sample fix PWM input for Motor
void writeMotors()
{//Start generate  fix PWM pulse @ pin  34, 36, 38, and 40
  motorCommand[MOTOR5]=LB3+0; //pin  9, Back  Left
  motorCommand[MOTOR6]=RB4+0; //pin  8, Back  Right
  motorCommand[MOTOR7]=LF2; //pin  7, Front Left
  motorCommand[MOTOR8]=RF1; //pin  6, Front Right


  //Limit the motor pulse within 1000-2000 us for 2500us period
  //the "ch" refer to MOTORx (i.e. MOTOR1 to MOTOR8)
  for (int ch = 4;ch < 8; ch++) 
  {
    if (motorCommand[ch] <= 1000) { motorCommand[ch] = 1000;}
    if (motorCommand[ch] >= 2000) { motorCommand[ch] = 2000;}
  }


  
  //Set each individual motor pulse by feeding "motorCommand[MOTORx]"
  //Formula: PWMC_SetDutyCycle(PWM, Channel, Pulse)

      PWMC_SetDutyCycle(PWM, 4, motorCommand[MOTOR5]); //pin  9,
      PWMC_SetDutyCycle(PWM, 5, motorCommand[MOTOR6]); //pin  8,
      PWMC_SetDutyCycle(PWM, 6, motorCommand[MOTOR7]); //pin  7,
      PWMC_SetDutyCycle(PWM, 7, motorCommand[MOTOR8]); //pin  6,    
}

////////////////////////////////////////////////////////////
////////////Startup Minimum Pulse for All motor/////////////
///////////////////called at void Setup()///////////////////
////////////////////////////////////////////////////////////
void commandAllMotors(int command) 
{
      PWMC_SetDutyCycle(PWM, 4, command); //PWM L4 or  //Pin  9
      PWMC_SetDutyCycle(PWM, 5, command); //PWM L5 or  //Pin  8
      PWMC_SetDutyCycle(PWM, 6, command); //PWM L6 or  //Pin  7
      PWMC_SetDutyCycle(PWM, 7, command); //PWM L7 or  //Pin  6
}


/////////////////////////////////////////////////////////////////////////////////
/////////////////////FUNCTION INSIDE DUE LIBRARY/////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void setPWMpin(uint32_t pin) 
{
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B, //hack Arduino does not allow high PWM by default
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

static void configOneMotor(uint8_t ch, uint32_t period) {
  PWMC_ConfigureChannel(PWM, ch, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_SetPeriod(PWM, ch, period);
  PWMC_SetDutyCycle(PWM, ch, 1000);
  PWMC_EnableChannel(PWM, ch);
}




float xEMA(float new_value) {
  emaX += alphax*(new_value - emaX);
  return(emaX);
}

float yEMA(float new_value) {
  emaY += alphax*(new_value - emaY);
  return(emaY);
}

float zEMA(float new_value) {
  emaZ += alphax*(new_value - emaZ);
  return(emaZ);
}

float gxEMA(float new_value) {
  emaXg += alphax*(new_value - emaXg);
  return(emaXg);
}

float gyEMA(float new_value) {
  emaYg += alphax*(new_value - emaYg);
  return(emaYg);
}

float gzEMA(float new_value) {
  emaZg += alphax*(new_value - emaZg);
  return(emaZg);
}

float ailRCEMA(float new_value) {
  emaX_ail += alphax*(new_value - emaX_ail);
  return(emaX_ail);
}

float eleRCEMA(float new_value) {
  emaX_ele += alphax*(new_value - emaX_ele);
  return(emaX_ele);
}

float thrRCEMA(float new_value) {
  emaX_thr += alphax*(new_value - emaX_thr);
  return(emaX_thr);
}

float rudRCEMA(float new_value) {
  emaX_rud += alphax*(new_value - emaX_rud);
  return(emaX_rud);
}


// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() {
  int       num_readings = 1000; //Default 10

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
  
  Serial.print("xg ");Serial.println(base_x_gyro);
  Serial.print("yg ");Serial.println(base_y_gyro);
  Serial.print("zg ");Serial.println(base_z_gyro);
  Serial.print("xa ");Serial.println(base_x_accel);
  Serial.print("ya ");Serial.println(base_y_accel);
  Serial.print("za ");Serial.println(base_z_accel);

  delay(100);
}


/*****************************************************ALL SCRATCHES HERE*************************************************************
 
//DEFINE
float Setpoint_Y=0;                  //Setpoint define by stick (Setpoint_Y=0 on Stick Center)
float Setpoint_Yprev=0;               //Previous setpoint (Setpoint_Yprev=0 on Stick Center)

float dt_Setpoint_Y=0;             //Changing pitch setpoint over time (setpoint/dt)
double G_Dt=0;

  //CONTROL STICK INPUT (Setpoint_Y=0 on Center Stick)
  Setpoint_Y = SetRollsAbout_X; 
  
  //DEAD BAND
  applyDeadband(Setpoint_Y, 1.5);//1.2
  
  //CALCULATE AUTO P GAIN  "dt_Setpoint_Y"
  dt_Setpoint_Y = (tau*dt_Setpoint_Y/(tau+G_Dt)) + ((Setpoint_Y-Setpoint_Yprev)/(tau+G_Dt));
  Setpoint_Yprev = Setpoint_Y;   dt_Setpoint_Y = constrain(dt_Setpoint_Y, -80, 80);

  
  //CALCULATE ANGLE ERROR
  float error_pitch = Setpoint_Y - ahrs_p;//ahrs_p*RAD_TO_DEG
  
  //CALCULATE ERROR RATE (P)
  float error_rate_pitch = dt_Setpoint_Y + Kp*error_pitch - GyroYf*RAD_TO_DEG;

  //CALCULATE AUTO (I)  (base on error_rate_pitch)
  pitch_I_rate += error_rate_pitch*Ki*G_Dt;

  //CALCULATE AUTO (D)  (base on error_rate_pitch)
  pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));
  
  //RECORD LAST ERROR RATE
  error_rate_pitchold = error_rate_pitch;
 
 
  u_pitch = Kp*error_rate_pitch + pitch_I_rate + Kd*pitch_D_rate;
  u_pitch = constrain(u_pitch, -200, 200);//+-300 120
  
  
  
//REVISED P:  
  Att_dErr_X = tau*Att_dErr_X/(tau+dtx) + (Att_error_X - Att_lastErr_X) / (tau+dtx);
  float Px = Att_dErr_X + kpx * Att_error_X - Gyr_x; 
  
//REVISED I:
  float Ix += kix*Px*dtx;
  
//REVISED D:
  float Dx =   (tar*Dx/(tau+dtx)) + ((Px-error_rate_pitchold)/(tau+dtx));
  
  
PID=kp*Px + Ix + kd*Dx;
  
**********************************************************************************************************************************/
//NOT USED
void calibrateESC()
{
while(TH_Pulse<1900) 
{digitalWrite(22, HIGH); digitalWrite(23, HIGH);delay(100);digitalWrite(22, LOW);digitalWrite(23, LOW); delay(100);}
 digitalWrite(23, LOW);digitalWrite(22, LOW);delay(1000);
  
 while(TH_Pulse>1100)
      {
       writeMotors();    
      }
}














