// Select ESP32 Wrover Module
#include <esp_camera.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h> //工具 管理程式庫:PubSubClient

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

/***************↓↓↓↓↓ 更改設定值 ↓↓↓↓↓***************/
// WIFI
#define SSID "classroom"    //wifi
#define WIFI_PWD "WDA89956399" //wifi

// MQTT
#define MQTT_SERVER "io.adafruit.com"
#define MQTT_PORT 1883
#define CLIENT_ID "juck30808_client"  //yourself
#define MQTT_USER "juck30808"
#define MQTT_PWD "aio_ULjq49JPIKn39uc2oUQTh9ImBXTo"

// TOPIC
#define TP_MOTOR "juck30808/feeds/feeds-group.mada-move"
#define TP_SETLAPS "juck30808/feeds/feeds-group.mada-count"
/***************↑↑↑↑↑ 更改設定值 ↑↑↑↑↑***************/

// 須依個人家中無線基地台的設定，修改以下 Static IP address
IPAddress local_IP(192, 168, 30, 40);
IPAddress gateway(192, 168, 30, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

void startCameraServer();

// 宣告控制腳位
int in3 = 12;
int in4 = 13;
int switchPin = 15;

extern int motorState = LOW; // 目前馬達運作狀態
int buttonState;             // 目前按鈕狀態
int lastButtonState = HIGH;  // 前次按鈕狀態

extern int setLaps = 1; // 設定出料盤旋轉圈數
int runLaps = 0;        // 目前馬達旋轉圈數

unsigned long lastDebounceTime = 0; // 機械彈跳判斷的起始時間
unsigned long debounceDelay = 5;    // 去除機械彈跳的判斷間隔時間

WiFiClient browser;
PubSubClient mqttClient(browser); // 宣告 mqttClient 物件

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  init_Pins();

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

  //init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
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

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_CIF);

  // 指定靜態IP請將註解移除
  //  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
  //  {
  //    Serial.println("STA Failed to configure");
  //  }

  Serial.printf("Connecting to %s ", SSID);
  WiFi.begin(SSID, WIFI_PWD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // MQTT連線設定
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT); // 連線 MQTT Broker
  mqttClient.setCallback(mqtt_callback);        // 當 ESP mcu 收到訂閱主題訊息時執行對應函式
  mqtt_connect();
  delay(200);

  // 傳送MQTT預設值
  mqttClient.publish(TP_MOTOR, inttochar(motorState));
  mqttClient.publish(TP_SETLAPS, inttochar(setLaps));
}

void loop()
{
  // 保持 MQTT 連結
  if (!mqttClient.connected())
  {
    mqtt_connect();
  }
  mqttClient.loop();

  // Debounce
  int reading = digitalRead(switchPin);
  if (reading != lastButtonState)
  {
    // Debounce 開始時間
    lastDebounceTime = millis();
  }

  // debounce 判斷的間隔時間
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    // 按鈕狀態確定改變
    if (reading != buttonState)
    {
      buttonState = reading;
      Serial.println(buttonState);

      // 開關為 INPUT_PULLUP，狀態值須反轉
      if (buttonState == HIGH)
      {
        runLaps++;
        if (runLaps >= setLaps && motorState == 1)
        {
          motorState = 0;
          mqttClient.publish(TP_MOTOR, inttochar(motorState));
          motor_stop();
          runLaps = 0;
        }
      }
    }
  }

  // 記錄開關最後狀態
  lastButtonState = reading;
}

char *inttochar(int i)
{
  static char c[5];
  sprintf(c, "%d", i);
  // itoa(i, c, 10);
  return c;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Serial.printf("Message from [%s]\n  Data Length: %d, Payload: ", topic, length);
  Serial.write(payload, length);
  Serial.println("");

  String pl = "";
  for (int i = 0; i < length; i++)
  {
    pl += (char)payload[i];
  }

  if (strcmp(topic, TP_MOTOR) == 0)
  {
    motorState = pl.toInt();
    if (motorState)
    {
      motor_forward();
    }
    else
    {
      motor_stop();
    }
  }
  else if (strcmp(topic, TP_SETLAPS) == 0)
  {
    setLaps = pl.toInt();
  }
}

void mqtt_connect()
{
  if (mqttClient.connect(CLIENT_ID, MQTT_USER, MQTT_PWD))
  {
    Serial.println("MQTT connected");
    mqttClient.subscribe(TP_MOTOR);
    mqttClient.subscribe(TP_SETLAPS);
  }
  else
  {
    Serial.print("MQTT connect failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 0.2 seconds");
    delay(200);
  }
}

// pins 初始化
void init_Pins()
{
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
}

void motor_forward()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void motor_backward()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void motor_stop()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  delay(200);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
