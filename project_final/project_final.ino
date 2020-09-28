#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <firFilter.h>
#define PI  3.14159265359
#define Declination  0.00226777777777
void(* resetFunc) (void) = 0;

int resetPin = 12;
char data;

Adafruit_BMP280 bmp; // I2C Interface
MPU9250_asukiaaa mySensor;
firFilter Filter;


float ax,ay,az,gx,gy,gz;
float mDirection;
float mX, mY, mZ;

float angle = 0;
float Heading = 0;


const float CONST_16G = 2048;
const float CONST_2000 = 16.4;
const float CONST_G = 9.81;
const float RADIANS_TO_DEGREES = 180 / 3.14159;
const float ALPHA = 0.96;
const float KMPH = 3.6;

unsigned long last_read_time;
float gyro_angle_x_l, gyro_angle_y_l,gyro_angle_z_l;
float angle_x_l, angle_y_l, angle_z_l;
float ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
boolean flag;


void setup() {



Serial.begin(115200);

  Wire.begin();
  Filter.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  calibrate_sensors();
  set_last_time(millis());
  mySensor.magXOffset = -12;
  mySensor.magYOffset = -44;
//------------------------------------

  if (!bmp.begin()) {
    Serial.println("Error In Sensors : YES ");
    while (1);
  }

 bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//-----------------------------------------


}

void loop() 
{
  main_loop();
}


void main_loop() 
{
  

if(Serial.available()>0)
{
  data = Serial.read();

  
  if(data == 'c')
  {
    calibrate_sensors();
    Serial.println("calibration started!   WAIT!");
    for(int i=9;i>=0;i--)
    {
      Serial.println(i);
      delay(300);
    }
    Serial.println("Calibrated");
    delay(700);
  }
  
  if(data == 'r')
  {
    Serial.println("Restarting device");
    for(int i=2;i>=0;i--)
    {
      Serial.println(i);
      delay(700);
    }
    Serial.println("Starting");
    delay(700);
    //digitalWrite(resetPin,LOW);
    resetFunc();
  }
  
 }



 
     mySensor.accelUpdate();
     mySensor.gyroUpdate();
     mySensor.magUpdate();

  //-------------------------------------

    ax = mySensor.accelX();
    ay = mySensor.accelY();
    az = mySensor.accelZ();
    
  //------------------------
    gx = mySensor.gyroX();
    gy = mySensor.gyroY();
    gz = mySensor.gyroZ();


  //------------------------

     mX = mySensor.magX();
     mY = mySensor.magY();
     mZ = mySensor.magZ();
     mX = mX * 0.1;   // uT to gauss
     mY = mY * 0.1;
     mZ = mZ * 0.1;

    // mX = Filter.run(mX);
    // mY = Filter.run(mY); 
    //   mX = Filter.run(mX);
      Heading = atan2(mY,mX)+ Declination;
      if (Heading>2*PI) /* Due to declination check for >360 degree */
      {
        Heading = Heading - 2*PI;
      }
      if (Heading<0)    /* Check for sign */
      {
        Heading = Heading + 2*PI;
      }
      angle = Heading* 180 / PI   ;/* Convert into angle and return */

      angle = Filter.run(angle);




    
    
    unsigned long t_now = millis();
    float dt = get_delta_time(t_now);
    float ax_p_v = (ax - ax_offset) / CONST_16G;
    float ay_p_v = (ay - ay_offset) / CONST_16G;
  //float az_p_v = (az / CONST_16G);


  float ax_p = ax;
  float ay_p = ay;
  float az_p = az;
  
  
  float accel_angle_y = atan(-1 * ax_p / sqrt(pow(ay_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(ay_p / sqrt(pow(ax_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  
  float gx_p = (gx - gx_offset) / CONST_2000;
  float gy_p = (gy - gy_offset) / CONST_2000;
  float gz_p = (gz - gz_offset) / CONST_2000;

  float gyro_angle_x = gx_p * dt + angle_x_l;
  float gyro_angle_y = gy_p * dt + angle_y_l;
  float gyro_angle_z = gz_p * dt + angle_z_l;
  
  float unf_gyro_angle_x = gx_p * dt + gyro_angle_x_l;
  float unf_gyro_angle_y = gy_p * dt + gyro_angle_y_l;
  float unf_gyro_angle_z = gz_p * dt + gyro_angle_z_l;


  float angle_x = ALPHA * gyro_angle_x + (1.0 - ALPHA) * accel_angle_x;
  float angle_y = ALPHA * gyro_angle_y + (1.0 - ALPHA) * accel_angle_y;
  float angle_z = gyro_angle_z;

  float vel_x = (ax_p_v * dt * CONST_G);
  float vel_y = (ay_p_v * dt * CONST_G);
  float vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2)) * KMPH;


  vel =  vel *1000; 
     
  //Serial.print("   vel: ");
  
  
  //Serial.print("Vel:");
  Serial.print(vel,2);
  Serial.print(" ");
//  Serial.print("X:");
  Serial.print(angle_x,2);
  Serial.print(" ");
 // Serial.print("y:");
  Serial.print(angle_y,2);
  Serial.print(" ");
 // Serial.print("Z:");
  Serial.print(angle_z,2);
  Serial.print(" ");
 //Serial.print("Heading:");
  Serial.print(angle);
  Serial.print(" ");

  //  Serial.print("Temperature:");
    Serial.print(bmp.readTemperature());
  // Serial.print(" *C  ");
Serial.print(" ");
   //Serial.print("Pressure:");
    Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
  // Serial.print(" hPa  ");
Serial.print(" ");
 //  Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1039.15)); //The "1013.5" is the pressure(hPa) at sea level in day in your region
   // Serial.print(" m");                    //If you don't know it, modify it until you get your current altitude
Serial.print(" ");
    Serial.println();

  
 
  set_last_time(t_now);
  set_last_gyro_angle_x(unf_gyro_angle_x);
  set_last_gyro_angle_y(unf_gyro_angle_y);
  set_last_gyro_angle_z(unf_gyro_angle_z);
  set_last_angle_x(angle_x);
  set_last_angle_y(angle_y);
  set_last_angle_z(gyro_angle_z);
  delay(30);
}

void calibrate_sensors() 
{
  unsigned long int                   num_readings = 10;
  unsigned long int                   i = 0;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;

  //Serial.println("Starting Calibration");
 

 // for (int i = 0; i < num_readings; i++) 
  while(i<=num_readings)
  {
    
    mySensor.accelUpdate();
    mySensor.gyroUpdate();

   
   ax = mySensor.accelX();
  ay= mySensor.accelY();
  az = mySensor.accelZ();
  //------------------------
  gx = mySensor.gyroX();
  gy = mySensor.gyroY();
  gz = mySensor.gyroZ();
    
    x_accel += ax;
    y_accel += ay;
    z_accel += az;
    x_gyro += gx;
    y_gyro += gy;
    z_gyro += gz;

delay(100);
    i++;
  }


  
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  // Store the raw calibration values globally
  ax_offset = x_accel;
  ay_offset = y_accel;
  az_offset = z_accel;
  gx_offset = x_gyro;
  gy_offset = y_gyro;
  gz_offset = z_gyro;

 
}


inline unsigned long get_last_time() {
  return last_read_time;
}

inline void set_last_time(unsigned long _time) {
  last_read_time = _time;
}

inline float get_delta_time(unsigned long t_now) {
  return (t_now - get_last_time()) / 1000.0;
}



inline void set_last_gyro_angle_x(float _gyro_angle_x) {
  gyro_angle_x_l = _gyro_angle_x;
}




inline void set_last_gyro_angle_y(float _gyro_angle_y) {
  gyro_angle_y_l = _gyro_angle_y;
}

inline void set_last_gyro_angle_z(float _gyro_angle_z) {
  gyro_angle_z_l = _gyro_angle_z;
}



inline void set_last_angle_x(float _ang_x) {
  angle_x_l = _ang_x;
}


inline void set_last_angle_y(float _ang_y) {

  angle_y_l = _ang_y;
}


inline void set_last_angle_z(float _ang_z) {
 
  angle_z_l = _ang_z;
}
