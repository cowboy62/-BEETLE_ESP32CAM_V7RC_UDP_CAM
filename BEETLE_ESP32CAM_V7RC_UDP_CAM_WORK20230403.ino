/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
/*
UDP Code Here

https://www.alejandrowurts.com/projects/bb9e-v1-update-7-esp32-bilateral-coms/


*/

/*
RoboTW   


FB:  RoboTW 機器人論壇
FB:  https://www.facebook.com/groups/540271770146161/

這個程式是用來配合V7RC 手機APP 的，可以利用WIFI UDP 控制載具 並利用WIFI 進行即時影像接收。
This Program is used with V7RC app @ Apple Store and Google Play. It can be used to control Servos and DIOS with WIFI and get real time Video via WIFI link.

 V7RC 是非常好用的手機遙控工具軟體，也感謝嵐奕科技有限公司
 https://apps.apple.com/tw/app/v7rc/id1390983964
 https://play.google.com/store/apps/details?id=com.v7idea.v7rcliteandroidsdkversion&hl=zh_TW
 

 影像部分的程式是從下列網址copy 來用，感謝
 原作者
 https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

 
   如果喜歡，記得來FB 群組跟我們分享
   If you like this work, please come to our FB Group, and tell us what you made.

 allen54a0@gmail.com
  
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"          //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "esp_http_server.h"


int TongsPwmCH  = 4  ;
int TongsPin    = 12 ;
int WheelRPwmCH = 5  ;
int WheelRPin   = 13 ;
int WheelLPwmCH = 6  ;
int WheelLPin   = 14 ;


void servo_angle(int channel, int angle)  // 使用 PWM 控制馬達轉動角度

{

 // regarding the datasheet of sg90 servo, pwm period is 20 ms and duty is 1->2ms

  int SERVO_RESOLUTION = 16;

  float range = (pow(2,SERVO_RESOLUTION)-1)/10;  

  float minDuty = (pow(2,SERVO_RESOLUTION)-1)/40;

  

  uint32_t duty = (range)*(float)angle/180.0 + minDuty;

  ledcWrite(channel, duty);

  delay(10);

}

// Replace with your network credentials

#define DEVICE_NAME "RoboTW V7RC"
#define BLE_NAME DEVICE_NAME

const char *ssid = DEVICE_NAME;
const char *password = "12345678";

///PWM  PIN 2  12 13  14 (4 LCD)
///LCDC CH  2  4   3   7

WiFiUDP Udp; // Creation of wifi Udp instance

char packetBuffer[255];

unsigned int localPort = 6188;

 

static const int servosPins[5] = {12, 13, 14, 15};

void initServo()
{

  // Ai-Thinker: pins  12 , 13, 14 15 for PWM Servo


   ledcSetup(TongsPwmCH, 50, 16);
   ledcAttachPin(TongsPin, TongsPwmCH);  // gpio, channel 

   ledcSetup(WheelRPwmCH, 50, 16);
   ledcAttachPin(WheelRPin, WheelRPwmCH);

   ledcSetup(WheelLPwmCH , 50, 16);
   ledcAttachPin(WheelLPin, WheelLPwmCH );

   ledcSetup(7, 50, 16);
   ledcAttachPin(15, 7);
}

void SetServoPos(int ch, int pos)
{
  uint32_t duty = ((((float)pos / 180.0) * 2000) / 20000.0 * 65536.0) + 1634;

  ledcWrite(ch, duty);
  // set channel to pos
}

//HZ Control
#define HZ_SETTING 100
int mainLoop_count;
unsigned long fast_loopTimer; // Time in miliseconds of main control loop
const int hzCount = (1000 / HZ_SETTING) - 1;
const int timeRemind = 1000 / HZ_SETTING;

///////////////////////BLE --------------------------->

int datafromV7RC[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
 
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

 

void parseCommand();
int flagShowControl = 0;

 

int hexConvert2int(char highByte, char lowByte)
{

  int val = 0;
  int highB;
  int lowB;

  if (highByte >= 'A')
  {
    highB = highByte - 'A' + 10;
  }
  else if (highByte >= '0' && highByte <= '9')
  {
    highB = highByte - 0x30;
  }

  if (lowByte >= 'A')
  {
    lowB = lowByte - 'A' + 10;
  }
  else if (lowByte >= '0' && lowByte <= '9')
  {
    lowB = lowByte - 0x30;
  }

  val = highB * 16 + lowB;
  val = val * 10;
  return val;
}

///// V7RC Code
 
 

void parseCommand()
{
  char cmd = Serial.read();
  switch (cmd)
  {
  case 'D':

    dumpespLoraData();
    break;

  case '1':
    flagShowControl = 1;
    break;

  case '0':
    flagShowControl = 0;
    break;
  }
}

void dumpespLoraData()
{

  for (int i = 0; i < 8; i++)
  {

    Serial.print(map(datafromV7RC[i], 1000, 2000, -255, 255));
    Serial.print(",");
  }
  for (int i = 0; i < 6; i++)
  {

    Serial.print(map(datafromV7RC[i], 1000, 2000, -255, 255));
    Serial.print(",");
  }
  Serial.println("");
}

///////////////////////Servo --------------------------->

#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  while (true)
  {
    fb = esp_camera_fb_get();

    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {
      if (fb->width > 400)
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted)
          {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL};

  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  initServo();
  servo_angle(TongsPwmCH, 90);   
  servo_angle(WheelRPwmCH, 90);   
  servo_angle(WheelLPwmCH, 90);   
  servo_angle(7, 90);  
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {

    Serial.println("psramFound");

    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_VGA);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
  //WiFi.softAP(ssid );

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Camera Stream Ready! Connect to the ESP32 AP and go to: http://");
  Serial.println(IP);

  ////UDP Here ...
  Udp.begin(localPort);

  // Start streaming web server
  startCameraServer();
}
int SentDelayCnt = 0 ;
int SentCnt = 0 ;
void loop()
{
  int RWheel = 90 ;   //右輪的值
  int LWheel = 90 ;   //左輪的值  
  int FB = 0 ;        // 前進後退map 到-90~90 
  int RL = 0 ;        // 左右轉  map 到-90~90 
  
  if (Serial.available())
    parseCommand();

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    String rxData;
    String data;
    int len = Udp.read(packetBuffer, 255);
    if (len > 0)
      packetBuffer[len - 1] = 0;
    // Serial.println(packetBuffer);
    //Serial.println( len);

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.printf("received: ");
    Udp.printf(packetBuffer);
    Udp.printf("\r\n");
    Udp.endPacket();

    if (len > 0)
    {
      //  Serial.println("*********");
      //   Serial.print("Received Value: ");
      for (int i = 0; i < len; i++)
      {
        //   Serial.print(rxValue[i]);
        rxData += packetBuffer[i];
      }

      //   Serial.println();
      // Serial.println("*********");
    }

    ///// V7RC Code ---------------------------------------------------------------->>>
    if (packetBuffer[1] == 'R')
    {

      for (int i = 0; i < 4; i++)
      {
        data = rxData.substring(i * 4 + 3, i * 4 + 7);
        datafromV7RC[i] = data.toInt();
      }
    }
    else
    { //for SS8   CMD  (8 Servo)   //SS8 96 96 96 96 96 96 96 96#

      for (int i = 0; i < 8; i++)
      {

        datafromV7RC[i] = hexConvert2int(packetBuffer[i * 2 + 3], packetBuffer[i * 2 + 4]);
      }
    }

    ////debug Only, send to Vrep....

    if (flagShowControl == 1)
    {
      Serial.print(packetBuffer[2]); /// should be V / T / 8 (2 ch, 4 ch , 8 ch )
      Serial.print(",");

      for (int i = 0; i < 8; i++)
      {
        Serial.print(datafromV7RC[i]);
        Serial.print(",");
      }

    //  Serial.println(",");
    }
  }

  ///// V7RC Code ----------------------------------------------------------------<<<<<

  if (millis() - fast_loopTimer > hzCount) //100 HZ
  {
    fast_loopTimer = millis();
    mainLoop_count++;
    SentDelayCnt ++ ;
    if (SentDelayCnt > 99 ) {
      SentDelayCnt = 0 ;
        SentCnt ++ ;
      Serial.println(SentCnt);
    }



    //////Servo  Loop --------------------------------------------------------------------->
   //Serial.print(",");
    if (mainLoop_count % 2 == 0)
    {
      //Serial.print("  ");

      RWheel = 90 ;
      LWheel = 90 ;
      /*
      if (datafromV7RC[2] > 1700) {RWheel = 110 ;LWheel = 110 ;}
      if (datafromV7RC[2] < 1300) {RWheel = 70 ;LWheel = 70 ;}
      if (datafromV7RC[3] > 1700) {RWheel = 70 ;LWheel = 110 ;}
      if (datafromV7RC[3] < 1300) {RWheel = 110 ;LWheel = 70 ;}
      */
      /*  //定速
      if (datafromV7RC[2] > 1700) {RWheel = 110 ;LWheel = 70  ;}  // 前進
      if (datafromV7RC[2] < 1300) {RWheel = 70  ;LWheel = 110 ;}  // 後退
      if (datafromV7RC[3] > 1700) {RWheel = 110 ;LWheel = 110 ;}  // 右轉
      if (datafromV7RC[3] < 1300) {RWheel = 70  ;LWheel = 70  ;}   //左轉
      */
      
      FB = map(datafromV7RC[2], 1000, 2000, -90, 90);
      RL = map(datafromV7RC[3], 1000, 2000, -90, 90);
      RWheel = 90 + FB + RL ;
      LWheel = 90 - FB + RL ;
      RWheel = ( RWheel  > 180 ) ? 180 : RWheel ;
      RWheel = ( RWheel  < 0 )   ?   0 : RWheel ;
      LWheel = ( LWheel  > 180 ) ? 180 : LWheel ;
      LWheel = ( LWheel  < 0 )   ?   0 : LWheel ;
      
      
      

      servo_angle(7,map(datafromV7RC[0], 1000, 2000, 0, 180) );   //unuse
      servo_angle(TongsPwmCH,map(datafromV7RC[1], 1000, 2000, 40, 140) );   
      servo_angle(WheelRPwmCH,RWheel );   
      servo_angle(WheelLPwmCH,LWheel );  
      
      Serial.print(" TO:");
      Serial.print(map(datafromV7RC[1], 1000, 2000, 40, 140));
      Serial.print(" RW:");
      Serial.print(RWheel);
      Serial.print(" LW");
      Serial.println(LWheel);
      /*
      Serial.print(datafromV7RC[0]);
      Serial.print("  ");
      Serial.print(datafromV7RC[1]);
      Serial.print("  ");
      Serial.print(datafromV7RC[2]);
      Serial.print("  ");
      Serial.print(datafromV7RC[3]);
      Serial.print("  ");
      Serial.println(",");
      */
      /*
      servo_angle(4,random(0, 180));   
      servo_angle(5,random(0, 180));   
      servo_angle(6,random(0, 180));   
      servo_angle(7,random(0, 180)); 
      */  
      
    }
  }
}
