
#define BLYNK_TEMPLATE_ID "TMPL3If_ks6HW"
#define BLYNK_TEMPLATE_NAME "AWHM"


   #define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <I2S.h>

#include "MAX30105.h" //sparkfun MAX3010X library
#include "heartRate.h"
SimpleTimer timer;
MAX30105 particleSensor;

char auth[] = "DtXehfNbA3j1mbLe1VODxhIm-R79jZeX";
//DtXehfNbA3j1mbLe1VODxhIm-R79jZeX
//y-Zy3xBP6suqxe3PX_A8vhcDywf2888P  



 #define INTERVAL_MESSAGE2 60000
unsigned long time_2 = 0;
int period = 2000;
unsigned long time_now = 0;
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100; //calculate SpO2 by this sampling interval

int oxygen;
double ESpO2 = 95.0;    //initial value of estimated SpO2
double FSpO2 = 0.7;     //filter factor for estimated SpO2
double frate = 0.95;    //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0      //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5      //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 3000  // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 0.0

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
int sample = I2S.read();

#define USEFIFO





#include "esp_camera.h"
#include <WiFi.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "POCO M6 Pro 5G";  //GJRB_DCF_ECE_Guest_2G
const char* password = "hellohello";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

   Blynk.begin(auth, ssid, password);

    while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
//while (1);
  }


   Serial.println("Place your index finger on the sensor with steady pressure.");

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 255; // 0x7F Options: 0=Off to 255=50mA
  byte sampleAverage = 4;   //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400;     //1000 is best but needs processing power//Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;     //Options: 69, 118, 215, 411
  int adcRange = 16384;     //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.enableDIETEMPRDY();
  I2S.setAllPins(-1, 42, 41, -1, -1);
  I2S.begin(PDM_MONO_MODE, 16000, 16);
  timer.setInterval(500, sendUptime);


  
  

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}


void sendUptime()
{
  Blynk.virtualWrite(V5, oxygen);
  Blynk.virtualWrite(V0, beatAvg);
   Blynk.virtualWrite(V3, I2S.read());
}




void loop()
{
  Blynk.run();
  timer.run(); // Initiates SimpleTimer
  
  // Read I2S data


    
long irValue = particleSensor.getIR();

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    float beatsPerMinute = 60 / (delta / 1000.0);
    
    Serial.print("Heart Rate (BPM): ");
    Serial.println(beatsPerMinute);

    // Store the heart rate in the array
    rates[rateSpot++] = beatsPerMinute;
    rateSpot %= RATE_SIZE; // Wrap variable

    // Calculate the average heartbeat
    beatAvg = 0;
    for (byte i = 0; i < RATE_SIZE; i++) {
      beatAvg += rates[i];
;
    }
    beatAvg /= RATE_SIZE;

 

  }






  uint32_t ir, red, green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

#ifdef USEFIFO
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available())

  { //do we have new data
#ifdef MAX30105
red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
ir = particleSensor.getFIFOIR();   //Sparkfun's MAX30105
#else
red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif

i++;
fred = (double)red;
fir = (double)ir;
avered = avered * frate + (double)red * (1.0 - frate); //average red level by low pass filter
aveir = aveir * frate + (double)ir * (1.0 - frate);    //average IR level by low pass filter
sumredrms += (fred - avered) * (fred - avered);        //square sum of alternate component of red level
sumirrms += (fir - aveir) * (fir - aveir);             //square sum of alternate component of IR level
if ((i % SAMPLING) == 0)
{ //slow down graph plotting speed for arduino Serial plotter by thin out
  if (millis() > TIMETOBOOT)
  {
    if (ir < FINGER_ON)
      ESpO2 = MINIMUM_SPO2; //indicator for finger detached
    float temperature = particleSensor.readTemperatureF();
    if (ESpO2 <= -1)
    {
      ESpO2 = 0;
    }

    if (ESpO2 > 100)
    {
      ESpO2 = 100;
    }

    oxygen = ESpO2;

    Serial.print(" Oxygen % = ");
    Serial.println(oxygen);
  }
}
if ((i % Num) == 0)
{
  double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
  // Serial.println(R);
  SpO2 = -23.3 * (R - 0.4) + 100;               //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
  ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; //low pass filter
  //Serial.print(SpO2); Serial.print(","); Serial.println(ESpO2);
  sumredrms = 0.0;
  sumirrms = 0.0;
  i = 0;
  break;
}
particleSensor.nextSample(); //We're finished with this sample so move to next sample
//Serial.println(SpO2);
  }

  Serial.println(beatAvg);
  Serial.println(I2S.read());


  if (millis() > time_2 + INTERVAL_MESSAGE2 && oxygen < 93)

  {
time_2 = millis();


Serial.print("Alert called");
  }

#endif

}
