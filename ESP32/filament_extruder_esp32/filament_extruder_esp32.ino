// Wiring ESP32 and TJC LCD
// ESP32<---->TJC
// GND--------GND
// 5V---------5V
// TX2(17)----RX
// RX2(16)----TX

#include <thermistor.h>

#define TJC Serial2
#define FRAME_LENGTH 6

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

// Thermistor model reference
// https://github.com/miguel5612/ThermistorLibrary/blob/master/src/Configuration.h
thermistor therm1(THERMISTOR_PIN, 4);

// wiring ESP32 and 74HC595
#define SHIFT_REGISTER_DATAPIN 19
#define SHIFT_REGISTER_LATCHPIN 18
#define SHIFT_REGISTER_CLOCKPIN 5

// wiring 74HC595 and four A4988
#define MOTOR_ONE_PUL 0
#define MOTOR_ONE_DIR 1
#define MOTOR_TWO_PUL 2
#define MOTOR_TWO_DIR 3
#define MOTOR_THR_PUL 4
#define MOTOR_THR_DIR 5
#define MOTOR_FOR_PUL 6
#define MOTOR_FOR_DIR 7

// wiring ESP32 and SSR
#define HEATER_PIN 1
#define FAN_PIN 2

// Variable that stores what is sent to the 74HC595 Shift Register
byte motorOutput;

class stepperMotor
{
public:
  void stop(void)
  {
    enable = 0;
  }

  void start(void)
  {
    enable = 1;
  }

  unsigned long steps(void)
  {
    return stepCount;
  }

  void init(int _pulsePin, int _dirPin, unsigned long _delayTime, bool _direction)
  {
    pulsePin = _pulsePin;
    dirPin = _dirPin;
    delayTime = _delayTime;
    direction = _direction;
    togglePulse = LOW;
    enable = 0;
    changeDirection(direction);
  }

  void control(void)
  {
    currentTime = micros();
    if (enable == 1)
    {
      if ((currentTime - deltaTime) > delayTime)
      {
        togglePulse = togglePulse == LOW ? HIGH : LOW;
        pulseCount++;
        if (pulseCount % 2 == 0)
        {
          stepCount++;
        }
        bitWrite(motorOutput, pulsePin, togglePulse);
        deltaTime = currentTime;
      }
    }
  }

  void changeDirection(bool _direction)
  {
    direction = _direction;
    bitWrite(motorOutput, dirPin, direction);
  }

  void changeSpeed(unsigned long _speed)
  {
    delayTime = _speed;
  }

private:
  unsigned long delayTime, currentTime;
  unsigned long deltaTime = 0;
  unsigned long pulseCount = 0;
  unsigned long stepCount = 0;
  int pulsePin, dirPin;
  bool direction, togglePulse, enable;
};

stepperMotor stepperOne, stepperTwo, stepperThree, stepperFour;

void setup()
{
  pinMode(SHIFT_REGISTER_DATAPIN, OUTPUT);
  pinMode(SHIFT_REGISTER_LATCHPIN, OUTPUT);
  pinMode(SHIFT_REGISTER_CLOCKPIN, OUTPUT);
  digitalWrite(SHIFT_REGISTER_LATCHPIN, HIGH);

  stepperOne.init(MOTOR_ONE_PUL, MOTOR_ONE_DIR, 500, LOW);
  stepperTwo.init(MOTOR_TWO_PUL, MOTOR_TWO_DIR, 600, LOW);
  stepperThree.init(MOTOR_THR_PUL, MOTOR_THR_DIR, 1000, LOW);
  stepperFour.init(MOTOR_FOR_PUL, MOTOR_FOR_DIR, 2000, LOW);

  stepperOne.start();
  stepperTwo.start();
  stepperThree.start();
  stepperFour.start();

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
  stepperOne.changeSpeed(1000);
  stepperTwo.changeSpeed(2000);
  stepperThree.changeSpeed(3000);
  stepperFour.changeSpeed(4000);

  if (extruder_toggle)
  {
    stepperOne.control();
  }

  if (puller_toggle)
  {
    stepperThree.control();
  }

  if (winder_toggle)
  {
    stepperFour.control();
  }

  // stepperTwo is not wired now
  // stepperTwo.control();

  // This updates the Shift Register Values in each loop
  digitalWrite(SHIFT_REGISTER_LATCHPIN, LOW);
  shiftOut(SHIFT_REGISTER_DATAPIN, SHIFT_REGISTER_CLOCKPIN, MSBFIRST, motorOutput);
  digitalWrite(SHIFT_REGISTER_LATCHPIN, HIGH);
}