// Wiring ESP32 and TJC LCD
// ESP32<---->TJC
// GND--------GND
// 5V---------5V
// TX2(17)----RX
// RX2(16)----TX

#define TJC Serial2
#define FRAME_LENGTH 6

// wiring ESP32 and A4988
#define EXTRUDER_STEP 12
#define EXTRUDER_DIR 14
#define PULLER_STEP 27
#define PULLER_DIR 26
#define WINDER_STEP 25
#define WINDER_DIR 33

// wiring ESP32 and SSR
#define HEATER_PIN 19
#define FAN_PIN 18

const int steps_per_rev = 200;
int duration;
unsigned long nowtime;

bool heater_toggle = 0;
bool extruder_toggle = 0;
bool puller_toggle = 0;
bool winder_toggle = 0;
bool fan_toggle = 0;

void setup()
{
  pinMode(EXTRUDER_STEP, OUTPUT);
  pinMode(EXTRUDER_DIR, OUTPUT);
  pinMode(PULLER_STEP, OUTPUT);
  pinMode(PULLER_DIR, OUTPUT);
  pinMode(WINDER_STEP, OUTPUT);
  pinMode(WINDER_DIR, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  Serial.begin(115200);

  // 初始化串口
  TJC.begin(115200);

  // 如果不delay，就会卡在启动界面
  // 推测串口屏开机比ESP32慢，所以屏幕错过了跳转主页面的信号
  delay(2000);

  // 因为串口屏开机会发送88 ff ff ff,所以要清空串口缓冲区
  // 清空串口缓冲区
  while (TJC.read() >= 0)
    ;

  TJC.print("page main\xff\xff\xff"); // 发送命令让屏幕跳转到main页面
  nowtime = millis();                 // 获取当前已经运行的时间

  xTaskCreate(
      control_extruder,
      "control extruder",
      2048,
      NULL,
      2,
      NULL);

  xTaskCreate(
      control_puller,
      "contro puller",
      2048,
      NULL,
      2,
      NULL);

  xTaskCreate(
      control_winder,
      "control winder",
      2048,
      NULL,
      2,
      NULL);

  xTaskCreate(
      control_lcd,
      "control lcd",
      2048,
      NULL,
      2,
      NULL);
}

void control_extruder(void *pvParameters)
{
  for (;;)
  {
    if (extruder_toggle)
    {
      extruder_spinning();
    }
    delay(200);
  }
}

void control_puller(void *pvParameters)
{
  for (;;)
  {
    if (puller_toggle)
    {
      puller_spinning();
    }
    delay(200);
  }
}

void control_winder(void *pvParameters)
{
  for (;;)
  {
    if (winder_toggle)
    {
      winder_spinning();
    }
    delay(200);
  }
}

void loop()
{
}

void control_lcd(void *pvParameters)
{
  for (;;)
  {
    digitalWrite(FAN_PIN, fan_toggle);
    digitalWrite(HEATER_PIN, heater_toggle);

    char str[100];
    if (millis() >= nowtime + 1000)
    {
      nowtime = millis(); // 获取当前已经运行的时间

      // 用sprintf来格式化字符串，给num_duration(时长)的val属性赋值
      sprintf(str, "num_duration.val=%d\xff\xff\xff", duration);
      TJC.print(str);
      duration++;

      // 用sprintf来格式化字符串，给txt_status的txt属性赋值
      sprintf(str, "txt_status.txt=\"status messages\"\xff\xff\xff");
      TJC.print(str);

      sprintf(str, "txt_temp.txt=\"30 C\"\xff\xff\xff");
      TJC.print(str);

      sprintf(str, "txt_diameter.txt=\"1.8\"\xff\xff\xff");
      TJC.print(str);

      sprintf(str, "txt_length.txt=\"0 mm\"\xff\xff\xff");
      TJC.print(str);
    }

    // 串口数据格式：
    // 串口数据帧长度：6字节
    // 帧头      button编号  button状态    帧尾
    // 0x55      1字节      1字节         0xffffff
    // 例子1：上位机代码  printh 55 01 00 ff ff ff  含义：1号按钮关闭
    // 例子2：上位机代码  printh 55 04 01 ff ff ff  含义：4号按钮打开
    // 例子3：上位机代码  printh 55 00 01 ff ff ff  含义：0号按钮打开
    // 例子4：上位机代码  printh 55 04 00 ff ff ff  含义：4号按钮关闭

    // 当串口缓冲区大于等于6时
    while (TJC.available() >= FRAME_LENGTH)
    {
      unsigned char ubuffer[FRAME_LENGTH];
      // 从串口缓冲读取1个字节但不删除
      unsigned char frame_header = TJC.peek();
      // 当获取的数据是包头(0x55)时
      if (frame_header == 0x55)
      {
        // 从串口缓冲区读取6字节
        TJC.readBytes(ubuffer, FRAME_LENGTH);
        if (ubuffer[3] == 0xff && ubuffer[4] == 0xff && ubuffer[5] == 0xff)
        {
          sprintf(str, "msg.txt=\"button %d is %s\"\xff\xff\xff", ubuffer[1], ubuffer[2] ? "on" : "off");
          TJC.print(str);
          control_toggles(ubuffer[1], ubuffer[2]);
        }
      }
      else
      {
        TJC.read(); // 从串口缓冲读取1个字节并删除
      }
    }
    delay(200);
  }
}

void control_toggles(unsigned char button_id, unsigned char toggle)
{
  if (button_id == 0)
  {
    heater_toggle = toggle ? 1 : 0;
  }
  if (button_id == 1)
  {
    extruder_toggle = toggle ? 1 : 0;
  }
  if (button_id == 2)
  {
    puller_toggle = toggle ? 1 : 0;
  }
  if (button_id == 3)
  {
    winder_toggle = toggle ? 1 : 0;
  }
  if (button_id == 4)
  {
    fan_toggle = toggle ? 1 : 0;
  }
}

void extruder_spinning()
{
  digitalWrite(EXTRUDER_DIR, HIGH);
  // Serial.println("Spinning extruder...");

  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(EXTRUDER_STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(EXTRUDER_STEP, LOW);
    delayMicroseconds(2000);
  }
}

void puller_spinning()
{
  digitalWrite(PULLER_DIR, HIGH);
  // Serial.println("Spinning puller...");

  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(PULLER_STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(PULLER_STEP, LOW);
    delayMicroseconds(2000);
  }
}

void winder_spinning()
{
  digitalWrite(WINDER_DIR, HIGH);
  // Serial.println("Spinning winder...");

  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(WINDER_STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(WINDER_STEP, LOW);
    delayMicroseconds(2000);
  }
}
