#include <Arduino.h>
#include <WiFi.h>
#include "BluetoothSerial.h"
#include <esp_spp_api.h>
#include <string.h>
#include <driver/adc.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h> // 추가

// arduino ide, install esp32 https://github.com/espressif/arduino-esp32

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define spiltchar = " "
#define audio_buf 8000
#define recordcount 10
uint8_t audioflag = 0;

BluetoothSerial SerialBT;
HTTPClient http;

// audio section
uint8_t audioBuffer[audio_buf];
uint8_t transbuffer[audio_buf];
uint32_t bufferPointer = 0;
#define serverhost "http://34.64.147.110:80/" // 서버 주소
bool transmitNow = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// bluetooth wifi connect section

RTC_DATA_ATTR bool is_get_wifi = false;
RTC_DATA_ATTR char get_ssid[10];
RTC_DATA_ATTR char get_password[20];
bool bluetooth_disconnect = false;
enum Setup_state { NONE, POWER_CHECK, BLUETOOTH_CHECK, BLUETOOTH_MODE, BLUETOOTH_CONNECTED, 
        WIFI_SCAN, SCAN_COMPLETE, WIFI_CONNECT, WIFI_CONNECTED, SETUP_MIC, SERVER_ERROR};
enum Setup_state state = NONE;
enum SetLED {GREEN, BLUE, RED, PURPLE, OFF};
enum SetLED led_state = OFF;
String network_string;
String ssids_array[30];
String wifi_info_arr[2];
#define chk_scan "scan"
const char* pref_ssid = "";
const char* pref_pass = "";

// button section

#define b_button_pin 27 // bluetooth button
#define p_button_pin 25 // power button
#define button_time 3000
long chk_time;
bool chk_power_state = false;
bool chk_ble = false;

#define battery_check_pin 33;

// NeoPixel LED
#define LED_PIN 12
#define LED_COUNT 1
#define BRIGHTNESS 50

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

void split(String wifi_info){
  char *token;
  char wifi_char[50];
  wifi_info.toCharArray(wifi_char, wifi_info.length()+1);
  token = strtok(wifi_char, " ");
  int wifi_arr = 0;
  while(token != NULL){
    wifi_info_arr[wifi_arr] = token;
    token = strtok(NULL, " ");
    wifi_arr++;
  }
}

// wifi section

void wifiscan(){
  WiFi.mode(WIFI_STA);
  
  int n =  WiFi.scanNetworks();
  if (n == 0) {
    SerialBT.println("no networks found");
  } else {
    delay(1000);
    for (int i = 0; i < n; ++i) {
      ssids_array[i + 1] = WiFi.SSID(i);
      network_string = i + 1;
      network_string = network_string + ": " + WiFi.SSID(i) + " (Strength:" + WiFi.RSSI(i) + ")";
      SerialBT.println(network_string);
    }
  }
  state = SCAN_COMPLETE;
}

bool wificonnect(){
  int wifi_num = wifi_info_arr[0].toInt();
  if(is_get_wifi){
    pref_ssid = get_ssid;
    pref_pass = get_password;
    bluetooth_disconnect = false;
  }else{
    strcpy(get_ssid, ssids_array[wifi_num].c_str());
    strcpy(get_password, wifi_info_arr[1].c_str());
    pref_ssid = ssids_array[wifi_num].c_str(); // rtc or eeprom
    pref_pass = wifi_info_arr[1].c_str();
    bluetooth_disconnect = true;
  }
  Serial.println(pref_ssid);
  Serial.println(pref_pass);

  WiFi.begin(pref_ssid, pref_pass);
  long start_wifi_connect = millis();
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    strip.setPixelColor(0, 0, 0, 200);
    strip.show();
    delay(100);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
    delay(100);
    if(millis() - start_wifi_connect > 5000){
      is_get_wifi = false;
      WiFi.disconnect(true, true);
      strip.setPixelColor(0, 200, 100, 0);
      strip.show();
      return false;
    }
  }
  is_get_wifi = true;
  led_state = GREEN;
  ledstate();
  return true;
}

// audio section

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  int adcVal = adc1_get_voltage(ADC1_CHANNEL_4);
  /*if (adcVal > 3800){
    adcVal = 4095;
  }*/
  uint8_t value = map(adcVal, 0 , 4095, 0, 255);
  audioBuffer[bufferPointer] = value;
  bufferPointer++;

  if (bufferPointer == audio_buf) {
    bufferPointer = 0;
    memcpy(transbuffer, audioBuffer, audio_buf);
    transmitNow = true;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// server connect section

void setMic(){
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_11db);

  http.begin(serverhost);
  int response = http.GET();

  if(response > 0){
    state = SETUP_MIC;

    timer = timerBegin(0, 8, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 625, true);
    timerAlarmEnable(timer);
  }else{
    state = SERVER_ERROR;
    led_state = PURPLE;
    ledstate();
  }
}

// bluetooth and device callback section

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT && state == BLUETOOTH_MODE){
    state = BLUETOOTH_CONNECTED;
    chk_ble = false;
  }

  if(event == ESP_SPP_DATA_IND_EVT && state == BLUETOOTH_CONNECTED){
    led_state = GREEN;
    ledstate();
    String chk_s = SerialBT.readString();
    chk_s.trim();
    if(chk_s == chk_scan){
      Serial.println("wifi-scan start");
      state = WIFI_SCAN;
    }
  }

  if(event == ESP_SPP_DATA_IND_EVT && state == SCAN_COMPLETE){
    String wifi_info = SerialBT.readString(); // 확인용
    wifi_info.trim();
    split(wifi_info);
    state = WIFI_CONNECT;
    is_get_wifi = false;
  }
}

// bluetooth section

void bluetooth_Setup(){
  SerialBT.begin("HermesSensor");
  chk_ble = true;
  chk_time = millis();
  while(millis() - chk_time < 15000 && state != BLUETOOTH_CONNECTED){
    strip.setPixelColor(0, 0, 0, 200);
    strip.show();
    delay(100);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
    delay(100);
  }
  led_state = GREEN;
  ledstate();
  if (chk_ble){
    SerialBT.end();
    state = NONE;
    led_state = BLUE;
    ledstate();
  }
}

void disconnect_bluetooth(){
  delay(1000);
  SerialBT.flush();
  SerialBT.disconnect();
  SerialBT.end();
  bluetooth_disconnect = false;
}

// power button interrupt section

void IRAM_ATTR p_button_isr(){
  state = POWER_CHECK;
}

void check_power(){
  long chk_p_button = millis();
  delayMicroseconds(5000);
  while(digitalRead(p_button_pin) == HIGH){
    if(millis() - chk_p_button >= button_time){
      chk_power_state = !chk_power_state;
      break;
    }
  }
}

void check_power_state(){
  if(chk_power_state){
    Serial.println("power on"); // 전원 확인용
    led_state = GREEN;
    ledstate();
  }else{
    led_state = OFF;
    ledstate();
    Serial.println("power OFF");
    Serial.println("Going to sleep now");
    detachInterrupt(p_button_pin);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
    esp_deep_sleep_start();
  }
}

// bluetooth button interrupt section

void IRAM_ATTR b_button_isr(){
  state = BLUETOOTH_CHECK;
}

void chk_bluetooth(){
  long chk_b_button = millis();
  while(digitalRead(b_button_pin) == LOW && state != BLUETOOTH_MODE){
    if(millis() - chk_b_button >= button_time){
      state = BLUETOOTH_MODE;
    }
  }
}

bool chLED(){
  int getResponse = http.GET();
  if(http.getString() == "led"){
    return true;
  }else{
    return false;
  }
}

// LED state

void ledstate(){
  switch (led_state)
  {
    case GREEN: // led green
      strip.setPixelColor(0, 0, 200, 0);
      strip.show();
      break;
    case BLUE: // led blue blink
      strip.setPixelColor(0, 0, 0, 200);
      strip.show();
      break;
    case PURPLE:
      strip.setPixelColor(0, 200, 0, 150);
      strip.show();
      break;
    case RED:
      strip.setPixelColor(0, 200, 0, 0);
      strip.show();
      break;
    case OFF:
      strip.setPixelColor(0, 0, 0, 0);
      strip.show();
      break;
  }
}

// main setup section

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  pinMode(b_button_pin, INPUT_PULLUP);
  pinMode(p_button_pin, INPUT_PULLDOWN);
  attachInterrupt(b_button_pin, b_button_isr, FALLING);
  attachInterrupt(p_button_pin, p_button_isr, RISING);
  SerialBT.register_callback(callback);
}

// main loop

void loop() {
  switch (state)
  {
    case POWER_CHECK:
      check_power();
      check_power_state();
      if(is_get_wifi){
        state = WIFI_CONNECT;
      }else{
        state = NONE;
      }
      break;
    case BLUETOOTH_CHECK:
      chk_bluetooth();
      break;

    case BLUETOOTH_MODE:
      bluetooth_Setup();
      break;

    case WIFI_SCAN:
      wifiscan();
      break;

    case WIFI_CONNECT:
      if(wificonnect()){
        state = WIFI_CONNECTED;
        if(bluetooth_disconnect){
          disconnect_bluetooth();
        }
        setMic();
      }else{
        Serial.println("Wi-Fi connection failed");
        state = NONE;
      }
      break;
    
    case SETUP_MIC:
      if(transmitNow){
        transmitNow = false;
        int httpResponseCode = http.POST((uint8_t *)transbuffer, sizeof(transbuffer));
        audioflag++;
        if (audioflag == recordcount){
          audioflag = 0;
          bufferPointer = 0;
          timerStop(timer);
          delay(5000);
          while(chLED()){
            led_state = GREEN;
            ledstate();
            delay(1000);
            led_state = OFF;
            ledstate();
            delay(1000);
          }
          led_state = GREEN;
          ledstate();
          timerStart(timer);
        }
      }
      break;
    
    case SERVER_ERROR:
      break;
  }
}
