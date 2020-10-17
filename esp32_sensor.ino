#include <Arduino.h>
#include "WiFi.h"
#include "BluetoothSerial.h"
#include <esp_spp_api.h>
#include <string.h>
#include <driver/adc.h>
#include <Adafruit_NeoPixel.h>

// arduino ide, install esp32 https://github.com/espressif/arduino-esp32

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define spiltchar = " "
#define AUDIO_BUFFER_MAX 512

BluetoothSerial SerialBT;

// audio section

uint8_t audioBuffer[AUDIO_BUFFER_MAX];
uint8_t transmitBuffer[AUDIO_BUFFER_MAX];
uint32_t bufferPointer = 0;
const char* serverhost = "192.168.0.13"; // 서버 주소
const int port = 8888;
bool transmitNow = false;

WiFiClient client;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// bluetooth wifi connect section

RTC_DATA_ATTR bool is_get_wifi = false;
RTC_DATA_ATTR char get_ssid[10];
RTC_DATA_ATTR char get_password[20];
bool bluetooth_disconnect = false;
enum LED_state {POWER_ON, BLUETOOTH_ON, SERVER_CONNECTING, SERVER_CON_FAILED, LOW_BATTERY, DONE};
enum Setup_state { NONE, POWER_CHECK, BLUETOOTH_CHECK, BLUETOOTH_MODE, BLUETOOTH_CONNECTED, 
        WIFI_SCAN, SCAN_COMPLETE, WIFI_CONNECT, CONNECT_FAILED, WIFI_CONNECTED, SERVER_CONNECTED, SERVER_CONNECT_FAILED};
enum LED_state led = POWER_ON;
enum Setup_state state = NONE;
String network_string;
String ssids_array[50];
String wifi_ssid;
String wifi_password;
String wifi_info_arr[2];
String chk_scan = "scan";
long wifi_timeout = 5000;
const char* pref_ssid = "";
const char* pref_pass = "";

// button section

const uint8_t b_button_pin = 27; // bluetooth button
const uint8_t p_button_pin = 25; // power button
long b_button_timeout = 3000;
long p_button_time = 3000;
long chk_time;
bool chk_power_state = false;

const uint8_t battery_check_pin = 33;

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
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(ssids_array[i + 1]);
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
  }else{
    strcpy(get_ssid, ssids_array[wifi_num].c_str());
    strcpy(get_password, wifi_info_arr[1].c_str());
    pref_ssid = ssids_array[wifi_num].c_str(); // rtc or eeprom
    pref_pass = wifi_info_arr[1].c_str();
  }

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
    if(millis() - start_wifi_connect > wifi_timeout){
      is_get_wifi = false;
      WiFi.disconnect(true, true);
      strip.setPixelColor(0, 200, 100, 0);
      strip.show();
      return false;
    }
  }
  is_get_wifi = true;
  led = POWER_ON;
  ledstate();
  return true;
}

// audio section

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  int adcVal = adc1_get_voltage(ADC1_CHANNEL_4);
  if (adcVal > 3800){
    adcVal = 4095;
  }
  uint8_t value = map(adcVal, 0 , 4095, 0, 255);
  audioBuffer[bufferPointer] = value;
  bufferPointer++;

  if (bufferPointer == AUDIO_BUFFER_MAX) {
    bufferPointer = 0;
    memcpy(transmitBuffer, audioBuffer, AUDIO_BUFFER_MAX);
    transmitNow = true;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// server connect section

void serverConnect(){
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_11db);

  if (!client.connect(serverhost, port)){
    Serial.println("server connection failed"); // 서버연결 확인용
    state = SERVER_CONNECT_FAILED;
    led = SERVER_CON_FAILED;
  }else {
    state = SERVER_CONNECTED;
    led = POWER_ON;

    Serial.println("connected to server"); //서버연결 확인용

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 125, true);
    timerAlarmEnable(timer);
  }
}

// bluetooth and device callback section

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT && state == BLUETOOTH_MODE){
    Serial.println("Device Connected"); // 연결확인용
    state = BLUETOOTH_CONNECTED;
  }

  if(event == ESP_SPP_DATA_IND_EVT && state == BLUETOOTH_CONNECTED){
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
  }
}

// bluetooth section

void bluetooth_Setup(){
  SerialBT.begin("HermesSensor");
  ledstate();
}

void disconnect_bluetooth(){
  delay(1000);
  Serial.println("BT stopping");
  SerialBT.flush();
  SerialBT.disconnect();
  SerialBT.end();
  Serial.println("BT stopped");
  bluetooth_disconnect = false;
}

// power button interrupt section

void IRAM_ATTR p_button_isr(){
  state = POWER_CHECK;
}

void check_power(){
  long chk_p_button = millis();
  while(digitalRead(p_button_pin) == HIGH){
    if(millis() - chk_p_button >= 3000){
      chk_power_state = !chk_power_state;
      break;
    }
  }
}

void check_power_state(){
  if(chk_power_state){
    Serial.println("power on"); // 전원 확인용
    led = POWER_ON;
    ledstate();
  }else{
    Serial.println("power OFF");
    Serial.println("Going to sleep now");
    detachInterrupt(p_button_pin);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
    strip.setPixelColor(0, 0, 0, 0);
    strip.show();
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
    if(millis() - chk_b_button >= b_button_timeout){
      led = BLUETOOTH_ON;
      state = BLUETOOTH_MODE;
    }
  }
}

void check_bluetooth_state(){
  if(state != BLUETOOTH_CONNECTED){
    Serial.println("cannot find device");
    SerialBT.end();
    state = NONE;
  }
}

// LED state

void ledstate(){
  switch (led)
  {
    case POWER_ON: // led green
      strip.setPixelColor(0, 0, 200, 0);
      strip.show();
      led = DONE;
      break;
    case BLUETOOTH_ON: // led blue blink
      chk_time = millis();
      while(millis() - chk_time < 10000 && state != BLUETOOTH_CONNECTED){
        strip.setPixelColor(0, 0, 0, 200);
        strip.show();
        delay(100);
        strip.setPixelColor(0, 0, 0, 0);
        strip.show();
        delay(100);
      }
      check_bluetooth_state();
      led = POWER_ON;
      ledstate();
      break;
    case SERVER_CONNECTING: // led green blink
      break;
    case SERVER_CON_FAILED:
      strip.setPixelColor(0, 200, 0, 100);
      strip.show();
      break;
    case LOW_BATTERY:
      strip.setPixelColor(0, 200, 0, 0);
      strip.show();
      break;
    case DONE:
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
  if(bluetooth_disconnect){
    disconnect_bluetooth();
  }
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
      Serial.println("Scanning Wi-Fi networks");
      wifiscan();
      break;

    case WIFI_CONNECT:
      if(wificonnect()){
        String connectIP = "IP : " + WiFi.localIP().toString();
        SerialBT.println(connectIP);
        Serial.println(connectIP);
        state = WIFI_CONNECTED;
        bluetooth_disconnect = true;
        serverConnect();
      }else{
        state = CONNECT_FAILED;
      }
      break;
    
    case CONNECT_FAILED:
      Serial.println("Wi-Fi connection failed");
      state = NONE;
      break;

    case SERVER_CONNECT_FAILED:
      ledstate();
      break;
    
    case SERVER_CONNECTED:
      if(transmitNow){
        transmitNow = false;
        client.write((const uint8_t *)audioBuffer, sizeof(audioBuffer));
      }
      break;
  }
}
