/// C++ source code

// last modified at 0423

#include <RPLidar.h> // RPLidar函式庫
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <math.h>
RPLidar lidar;

#define RPLIDAR_MOTOR 3
#define ALARM 28
#define Light5v 2
#define PI 3.1415926535

#define TFT_CS 53
#define TFT_RST 8
#define TFT_DC 9

// timer
unsigned long long stopwatch = 0;

// serial monitor
const unsigned long printinterval_rp = 10000; //micro seconds
const unsigned long printinterval_tf = 50000; //micro seconds
unsigned long lastprinttime_rp = 0;
unsigned long lastprinttime_tf = 0;

// lcd screen
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
static unsigned long lastClearTime = 0;
const unsigned long lcdprintinterval = 150000; // micro seconds
const unsigned long lcdlast = 500000; // micro seconds 0 5000000

// TF02 Pro
int dist, strength, check;
int uart[9];
const int HEADER = 0x59;
unsigned long prevTime = 0;
int prevDist = -1;
const unsigned long interval = 50000; // micro seconds

// RPLidar 
static int checkLidar = 0;
const int checkpoint = 4; //5
const unsigned long timeWindow = 100000.0; // interval == 0.06s
const unsigned long receive_interval = 50000.0;
unsigned long last_receive_time = 0;
static unsigned long startTime = micros(); 
const unsigned int rplidarspeed = 255; // 255 200
unsigned long times = 0;
// RPLiDAR-xy cor 
const float magnification = 10; // 2// 10 -> detecting range(radius) == 5m ; 17 -> 3m; detecting radius : magnification*50 cm
// data
struct point {
  int x, y;
};
point data1[50];
point data2[50];



// alarm
static unsigned long alarmstart = 0;
const unsigned long alarmlast = 200000; // micro seconds

// led light
static unsigned long lightstart = 0; // micro seconds
const unsigned long lightlast = 500000;  // micro seconds

///////////////////////////////////////////////////////////
void setup() 
{
  // 初始化 Serial 通訊
  Serial.begin(115200);   // Serial Monitor
  Serial1.begin(115200);  // RPLidar
  Serial2.begin(115200);  // TF02 Pro

  // 初始化引腳
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  pinMode(ALARM, OUTPUT);
  pinMode(Light5v, OUTPUT);

  // 初始化 Lidar alarm
  lidar.begin(Serial1);
  digitalWrite(ALARM, LOW);
  analogWrite(RPLIDAR_MOTOR, rplidarspeed); 

  //initialize lcdscreen ST77XX
  tft.init(240, 240);        // 初始化解析度
  tft.setSPISpeed(16000000);  // SPI高速傳輸
  tft.setRotation(0); // rotate screen
  tft.fillScreen(ST77XX_BLACK); 
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // 白字＋黑底
  tft.setTextSize(1);
  tft.setCursor(13, 0);
  tft.print("system running...");
  tft.drawLine(46, 85, 54, 85, ST77XX_WHITE);
  tft.drawLine(46, 155, 54, 155, ST77XX_WHITE);
  tft.drawLine(50, 85, 50, 155, ST77XX_WHITE);
  tft.drawLine(48, 120, 52, 120, ST77XX_WHITE);
  tft.setCursor(34, 82);
  tft.print("0m");
  tft.setCursor(35, 117);
  tft.print("5m");
  tft.setCursor(28, 150);
  tft.print("10m");
  tft.fillTriangle(64, 65, 59, 80, 69, 80, tft.color565(0, 0, 200));
  tft.drawRect(13, 9, 102, 55, tft.color565(64, 64, 64));

}

///////////////////////////////////////// loop starts here /////////////////////////////////////////
void loop() 
{ 
    unsigned long currTime = micros(); // micro second
    if (IS_OK(lidar.waitPoint())) 
    {
      analogWrite(RPLIDAR_MOTOR, rplidarspeed); ////
      // get informations and calculate; frequent : fastest
      
      float distance = lidar.getCurrentPoint().distance;float angle = lidar.getCurrentPoint().angle;
      float degree = angle * PI / 180;
      float xcor = (distance / magnification) * cos(degree); // distance unit : mm to cm, detecting radius : magnification*50 cm
      float ycor = (distance / magnification) * sin(degree); // distance unit : mm to cm
      times++; // calculate how many times rplidar has scanned

      //print on lcd screen and serial monitor
      if (angle > 0 && angle < 180 && distance >= 10)
      {
        if (distance / 10.0 <= 50 && distance / 10.0 >= 2) 
        {
          checkLidar++;
        }

        if(micros() - lastprinttime_rp >= printinterval_rp)
        {
          Serial.print("Lidar Angle: "); Serial.print(angle);
          Serial.print(" | Distance: "); Serial.print(distance / 10); Serial.println(" cm");

          if (xcor > -500 && xcor < 500 && ycor > 10 && ycor < 500)
          {
            // tft.drawPixel(64 - xcor/10.0, -ycor/10.0 + 63, ST77XX_WHITE); // xcor/10.0 + 64
            tft.fillRect(64 - xcor/10.0, -ycor/10.0 + 62, 2, 2, ST77XX_WHITE); // xcor/10.0 + 64
            // tft.drawCircle(xcor/10.0 + 64, -ycor/10.0 + 63, 1, ST77XX_WHITE);
          }
          lastprinttime_rp = micros();
        }
      }

      /*
      if (angle > 0 && angle < 180 && distance >= 10) 
      {
        Serial.print("Lidar Angle: "); Serial.print(angle);
        Serial.print(" | Distance: "); Serial.print(distance / 10); Serial.println(" cm");
        if (xcor/10 <= 300 && ycor/10 <= 500)
        {
          checkLidar++;
          Serial.print(" | x: "); Serial.print(xcor / 10); Serial.print(" cm"); Serial.print(" | y: ")Serial.print(ycor / 10); Serial.println(" cm");
        }
      }
      */

      // alarm
      if (micros() - startTime <= timeWindow) // micros() - startTime <= timeWindow
      {
        if (checkLidar >= checkpoint) 
        {
          Serial.println("LIDAR ALARM TRIGGERED!");
          digitalWrite(ALARM, HIGH);
          alarmstart = micros();
          checkLidar = 0;
          startTime = micros();
        }
      } else {
        checkLidar = 0;
        startTime = micros();
      }
      if(micros() - alarmstart > alarmlast) digitalWrite(ALARM, LOW);
    } else {
      analogWrite(RPLIDAR_MOTOR, 0);
      Serial.println("rplidar not available");
      rplidar_response_device_info_t info;
      
      if (IS_OK(lidar.getDeviceInfo(info, 100))) 
      {
        lidar.startScan();
        analogWrite(RPLIDAR_MOTOR, rplidarspeed);
        delay(1000); 
      }
      
      /*
      while(!IS_OK(lidar.waitPoint()))
      {
        delay(10);
      }
      */
    }

  //TF02 Pro processing
  currTime = micros(); // micro second
  if (currTime - prevTime >= interval) // interval : measure frequents
  {
    int currDist = measure();

    if (currDist != -1 && prevDist != -1) 
    {
      double duration = (currTime - prevTime) / 1000000.0; // second
      double distanceChange = prevDist - currDist; 
      double V = (distanceChange / duration) / 100; // m/s

      Serial.print("TF02 Distance1: "); Serial.println(prevDist);
      Serial.print("TF02 Distance2: "); Serial.println(currDist);
      Serial.print("Relative speed: "); Serial.print(V); Serial.println(" m/s");

      double T;
      bool Tavail;
      if (V >= 0.1) 
      {
        T = (prevDist / V) / 100;
        Tavail = 1;
      } else {
        Tavail = 0;
        T = 1000;
      }

      //print on lcd screen and serial monitor
      if(micros() - lastprinttime_tf >= printinterval_tf) 
      {
        float display = currDist * 0.07;
        if(display <= 70)
        {
          tft.fillRect(60, 150, 10, 5, ST77XX_BLACK);
          tft.fillRect(60, 85, 40, 80, ST77XX_BLACK);
          tft.drawRect(60, 85, 10, display, ST77XX_WHITE);
        } else {
          tft.drawRect(60, 85, 10, 65, ST77XX_WHITE);
          tft.fillRect(60, 150, 10, 5, ST77XX_RED);
        }
        Serial.print("TF02 previous distance = "); Serial.println(prevDist);
        Serial.print("TF02 current distance = "); Serial.println(currDist);
        Serial.print("duration = "); Serial.print(duration); Serial.println(" s");
        Serial.print("Relative speed : "); Serial.print(V); Serial.println(" m/s");  // 相對速度
        if (Tavail) {
          Serial.print("time to collision : "); 
          Serial.print(T); 
          Serial.println(" s");
        }
        Serial.println("---------");
      }

      // led light
      if (T <= 2.5) 
      {
        lightstart = currTime;
        digitalWrite(Light5v, HIGH);
      }
      if(currTime - lightstart >= lightlast) digitalWrite(Light5v, LOW);
    }

    //tft.fillRect(14, 10, 100, 51, ST77XX_BLACK); // clean display
    prevDist = currDist;
    prevTime = currTime;
  }

  // clean lcd screen 
  if (micros() - lastClearTime > lcdlast) {
    tft.fillRect(14, 10, 100, 52, ST77XX_BLACK);
    lastClearTime = micros();
  }
}
///////////////////////////////////////// loop ends here /////////////////////////////////////////

// TF02 Pro 測量函式
int measure() 
{
  Serial2.flush();
  int timeout = 1000;
  while (!Serial2.available() && timeout > 0) { // not available
    delay(1);
    timeout--;
  }

  if (timeout == 0) return -1;
  if (Serial2.read() == HEADER) 
  {
    uart[0] = HEADER;

    timeout = 1000;
    while (!Serial2.available() && timeout > 0) // not available
    {
      delay(1);
      timeout--;
    }

    if (timeout == 0) return -1;
    if (Serial2.read() == HEADER) 
    {
      uart[1] = HEADER;
      for (int i = 2; i < 9; i++) {
        uart[i] = Serial2.read();
      }

      check = 0;
      for (int i = 0; i < 8; i++) 
      {
        check += uart[i];
      }

      if (uart[8] == (check & 0xff)) 
      {
        dist = uart[2] + uart[3] * 256;
        strength = uart[4] + uart[5] * 256;
        return dist;
      } else {
        resetBuffer();
        return -1;
      }
    } else {
      resetBuffer();
      return -1;
    }
  } else {
    resetBuffer();
    return -1;
  }
}

void resetBuffer() 
{ // TF02 pro
  while (Serial2.available()) 
  {
    Serial2.read();
  }
  delay(10);
}

