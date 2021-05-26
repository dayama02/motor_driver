#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "Koma-Koma-Spot";
const char* password = "Usagi10kurasu";

// Left motor
const int L_IN_1   = 23;
const int L_IN_2   = 4;
const int R_IN_1   = 16;
const int R_IN_2   = 17;
const int L_PWM_1  = 0;
const int L_PWM_2  = 1;
const int R_PWM_1  = 2;
const int R_PWM_2  = 3;
const int STNBY    = 21;

int state = 0;
int count = 0;

// Serial
String inputString = "";         // a String to hold incoming data
String cmd_str[2];
int i = 0;


double sign(double A){
    if(A>0) return 1;
    else if(A<0) return -1;
    else return 0;
}

void initializeGpio(){
  // Motor setup
  // PWMのチャンネル使いますよ宣言
  ledcSetup(0, 12800, 10);
  ledcSetup(1, 12800, 10);
  ledcSetup(2, 12800, 10);
  ledcSetup(3, 12800, 10);

  // PWMのチャンネルとGPIOの割当
  ledcAttachPin(L_IN_1, L_PWM_1);
  ledcAttachPin(L_IN_2, L_PWM_2);
  ledcAttachPin(R_IN_1, R_PWM_1);
  ledcAttachPin(R_IN_2, R_PWM_2);

  pinMode(STNBY, OUTPUT);
  digitalWrite(STNBY, HIGH);
}

void serialEvent() {
  while (Serial.available()) {

    char inChar = (char)Serial.read();

    if (inChar == ',') {
      cmd_str[i++]=inputString;
      inputString="";
    }else if(inChar == '\n') {
      cmd_str[i]=inputString;
      inputString="";
      i=0;
    }else{
      inputString += inChar;    
    }
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
      
  inputString.reserve(200);
  initializeGpio();
  

}

void loop() {
  ArduinoOTA.handle();
  serialEvent();
  float left_speed  = cmd_str[0].toFloat();
  float right_speed = cmd_str[1].toFloat();


  int left_pwm = 0;
  int right_pwm = 0;

  if(abs(left_speed)> 0){
    if(sign(left_speed) == 1){
      left_pwm  = left_speed  / 0.45 * 384 + 640;  
    }
    else{
      left_pwm  = left_speed  / 0.45 * 384 - 640;
    }
  }
  else{
    left_pwm = 0;
  }

  if(abs(right_speed) > 0){
    if(sign(right_speed) == 1){
      right_pwm = right_speed / 0.45 * 384 + (584);  
    }
    else{
      right_pwm = right_speed / 0.45 * 384 - (584);
    }
  }
  else{
    right_pwm = 0;
  }

  Serial.printf("left %1.1f(%d), right %1.1f(%d)\n", left_speed, left_pwm, right_speed,right_pwm);
  if(left_pwm > 0){
    ledcWrite(L_PWM_1,left_pwm);
    ledcWrite(L_PWM_2, 0);
  }
  else{
    ledcWrite(L_PWM_1, 0);
    ledcWrite(L_PWM_2, -left_pwm);
  }

  if(right_pwm > 0){
    ledcWrite(R_PWM_1,right_pwm);
    ledcWrite(R_PWM_2, 0);
  }
  else{
    ledcWrite(R_PWM_1, 0);
    ledcWrite(R_PWM_2, -right_pwm);
  }

  delay(100);
}
