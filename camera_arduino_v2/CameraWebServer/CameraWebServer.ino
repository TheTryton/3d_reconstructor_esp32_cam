#include "esp_camera.h"
#include <WiFi.h>
#include <EEPROM.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

struct wifi_cred
{
  char ssid[64];
  char password[64];
};

#define EEPROM_SIZE 128

void startCameraServer();

wifi_cred load_cred_from_eeprom()
{
    wifi_cred cred;
    for(int i=0;i<64;i++)
    {
       cred.ssid[i] = EEPROM.read(i);
       cred.password[i] = EEPROM.read(i + 64);
    }
    return cred;
}

void save_cred_to_eeprom(const wifi_cred& cred)
{
    for(int i=0;i<64;i++)
    {
       EEPROM.write(i, cred.ssid[i]);
       EEPROM.write(i + 64, cred.password[i]);
    }
    EEPROM.commit();
}

int read_until_eol(char* buf, int buf_size, unsigned long timeout = UINT_MAX)
{
  int i=0;
  unsigned long start = millis();
  while(i < buf_size && (millis() - start) < timeout)
  {
      if(Serial.available())
      {
          char c = Serial.read();
          if(c == '\r' || c == '\n') break;
          buf[i++] = c;
      }
  }
  return i;
}

wifi_cred get_wifi_cred()
{
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("If you want to change wifi ssid and password type Y. (you have 5 sec)");

    wifi_cred cred;

    char c[64];
    if(read_until_eol(c, 63, 5000) && c[0] == 'Y')
    {
        Serial.println("WiFi SSID:");
        cred.ssid[read_until_eol(cred.ssid, 63)] = '\0';
        Serial.println("WiFi PASSWORD:");
        cred.password[read_until_eol(cred.password, 63)] = '\0';
        save_cred_to_eeprom(cred);
        Serial.println("WiFi ssid and password saved for future boots.");
    }
    else
    {
        Serial.println("Timeout exceeded loading last saved wifi ssid and password.");
        cred = load_cred_from_eeprom();
    }
    Serial.println("Connecting to:");
    Serial.println(cred.ssid);
    
    return cred;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  wifi_cred cred = get_wifi_cred();

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
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
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
  s->set_framesize(s, (framesize_t) 7); // SVGA

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(cred.ssid, cred.password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to WiFi!");

  startCameraServer();

  Serial.print("Camera configuration under address: 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
}
