// Wiring ESP32 and TJC LCD
// ESP32<---->TJC
// GND--------GND
// 5V---------5V
// TX2(17)----RX
// RX2(16)----TX

#include <AccelStepper.h>
#include <thermistor.h>

#define motorInterfaceType 1

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

// wiring ESP32 and thermistor
#define THERMISTOR_PIN 32

const int steps_per_rev = 200;
int duration;
unsigned long nowtime;
int length = 0;
float diameter = 1.8;

bool heater_toggle = 0;
bool extruder_toggle = 0;
bool puller_toggle = 0;
bool winder_toggle = 0;
bool fan_toggle = 0;

AccelStepper extruderStepper(motorInterfaceType, EXTRUDER_STEP, EXTRUDER_DIR);
AccelStepper pullerStepper(motorInterfaceType, PULLER_STEP, PULLER_DIR);
AccelStepper winderStepper(motorInterfaceType, WINDER_STEP, WINDER_DIR);

// Thermistor model reference
// https://github.com/miguel5612/ThermistorLibrary/blob/master/src/Configuration.h
thermistor therm1(THERMISTOR_PIN, 4);

void setup()
{
  extruderStepper.setMaxSpeed(1000);
  extruderStepper.moveTo(2000);

  pullerStepper.setMaxSpeed(1000);
  pullerStepper.moveTo(2000);

  winderStepper.setMaxSpeed(1000);
  winderStepper.moveTo(2000);

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  // Serial0 for debug
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
}

void loop()
{
  digitalWrite(FAN_PIN, fan_toggle);
  digitalWrite(HEATER_PIN, heater_toggle);
  control_steppers();
  control_lcd();
}

void control_lcd()
{
  char str[100];
  if (millis() >= nowtime + 1000)
  {
    nowtime = millis(); // 获取当前已经运行的时间

    // read temperature from thermistor
    int analogValue = analogRead(THERMISTOR_PIN);
    Serial.printf("ADC analog value = %d\n", analogValue);

    double temp = therm1.analog2temp();
    Serial.print("Temperature: ");
    Serial.println((String)temp);

    // update UI
    sprintf(str, "txt_temp.txt=\"%.1f\"\xff\xff\xff", temp);
    TJC.print(str);

    sprintf(str, "num_duration.val=%d\xff\xff\xff", duration);
    TJC.print(str);
    duration++;

    sprintf(str, "txt_status.txt=\"status messages\"\xff\xff\xff");
    TJC.print(str);

    sprintf(str, "txt_diameter.txt=\"%.1f\"\xff\xff\xff", diameter);
    TJC.print(str);

    length += 2;
    sprintf(str, "txt_length.txt=\"%d mm\"\xff\xff\xff", length);
    TJC.print(str);

    sprintf(str, "txt_preheat.txt=\"done\"\xff\xff\xff");
    TJC.print(str);

    sprintf(str, "temp_threshold.txt=\"110 C\"\xff\xff\xff");
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
        decode_toggles(ubuffer[1], ubuffer[2]);
      }
    }
    else
    {
      TJC.read(); // 从串口缓冲读取1个字节并删除
    }
  }
}

void decode_toggles(unsigned char button_id, unsigned char toggle)
{
  if (button_id == 0x0)
  {
    heater_toggle = toggle ? 1 : 0;
  }
  if (button_id == 0x1)
  {
    extruder_toggle = toggle ? 1 : 0;
  }
  if (button_id == 0x2)
  {
    puller_toggle = toggle ? 1 : 0;
  }
  if (button_id == 0x3)
  {
    winder_toggle = toggle ? 1 : 0;
  }
  if (button_id == 0x4)
  {
    fan_toggle = toggle ? 1 : 0;
  }
  if (button_id == 0x10)
  {
    heater_toggle = 1;
    extruder_toggle = 1;
    puller_toggle = 1;
    winder_toggle = 1;
    fan_toggle = 1;
  }
  if (button_id == 0x11)
  {
    heater_toggle = 0;
    extruder_toggle = 0;
    puller_toggle = 0;
    winder_toggle = 0;
    fan_toggle = 0;
  }
}

void control_steppers()
{
  extruderStepper.setSpeed(200);
  pullerStepper.setSpeed(100);
  winderStepper.setSpeed(50);

  if (extruder_toggle)
  {
    if (extruderStepper.distanceToGo() == 0)
      extruderStepper.moveTo(extruderStepper.currentPosition() + 2000);
    extruderStepper.runSpeed();
  }

  if (puller_toggle)
  {
    if (pullerStepper.distanceToGo() == 0)
      pullerStepper.moveTo(pullerStepper.currentPosition() + 2000);
    pullerStepper.runSpeed();
  }

  if (winder_toggle)
  {
    if (winderStepper.distanceToGo() == 0)
      winderStepper.moveTo(winderStepper.currentPosition() + 2000);
    winderStepper.runSpeed();
  }
}