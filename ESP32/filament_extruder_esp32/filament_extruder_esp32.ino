// Wiring ESP32 and TJC LCD
// ESP32<---->TJC
// GND--------GND
// 5V---------5V
// TX2(17)----RX
// RX2(16)----TX

#include <thermistor.h>

#define TJC Serial2
#define FRAME_LENGTH 7

// wiring ESP32 and thermistor
#define THERMISTOR_PIN 36

int duration;
unsigned long nowtime;
int length = 0;
float diameter = 1.8;

bool heater_toggle = 0;
bool extruder_toggle = 0;
bool puller_toggle = 0;
bool winder_toggle = 0;
bool stepper4_toggle = 0;

bool fan_toggle = 0;

int speed_extruder = 1000;
bool direction_extruder = 0;
int speed_puller = 1000;
bool direction_puller = 0;
int speed_winder = 1000;
bool direction_winder = 0;
int speed_stepper4 = 1000;
bool direction_stepper4 = 0;

// Thermistor model reference
// https://github.com/miguel5612/ThermistorLibrary/blob/master/src/Configuration.h
thermistor therm1(THERMISTOR_PIN, 4);

// wiring ESP32 and 74HC595
// SER (Serial Input) = Data pin
// RCLK (Register Clock) = Latch pin
// SRCLK (Shift Register Clock) = Clock pin
#define SHIFT_REGISTER_DATAPIN 26
#define SHIFT_REGISTER_LATCHPIN 25
#define SHIFT_REGISTER_CLOCKPIN 33

// wiring 74HC595 and four A4988
#define EXTRUDER_PUL 7
#define EXTRUDER_DIR 6
#define PULLER_PUL 5
#define PULLER_DIR 4
#define WINDER_PUL 3
#define WINDER_DIR 2
#define STEPPER4_PUL 1
#define STEPPER4_DIR 0

// wiring ESP32 and SSR
#define HEATER_PIN 0
#define FAN_PIN 13

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
      if ((currentTime - lastTime) > delayTime)
      {
        togglePulse = togglePulse == LOW ? HIGH : LOW;
        pulseCount++;
        if (pulseCount % 2 == 0)
        {
          stepCount++;
        }
        bitWrite(motorOutput, pulsePin, togglePulse);
        lastTime = currentTime;
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
  unsigned long lastTime = 0;
  unsigned long pulseCount = 0;
  unsigned long stepCount = 0;
  int pulsePin, dirPin;
  bool direction, togglePulse, enable;
};

stepperMotor extruder, puller, winder, stepper4;

void setup()
{
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  
  pinMode(SHIFT_REGISTER_DATAPIN, OUTPUT);
  pinMode(SHIFT_REGISTER_LATCHPIN, OUTPUT);
  pinMode(SHIFT_REGISTER_CLOCKPIN, OUTPUT);
  digitalWrite(SHIFT_REGISTER_LATCHPIN, HIGH);

  extruder.init(EXTRUDER_PUL, EXTRUDER_DIR, 500, LOW);
  puller.init(PULLER_PUL, PULLER_DIR, 600, LOW);
  winder.init(WINDER_PUL, WINDER_DIR, 1000, LOW);
  stepper4.init(STEPPER4_PUL, STEPPER4_DIR, 2000, LOW);

  extruder.start();
  puller.start();
  winder.start();
  stepper4.start();

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
    // Serial.printf("ADC analog value = %d\n", analogValue);

    double temp = therm1.analog2temp();
    // Serial.print("Temperature: ");
    // Serial.println((String)temp);

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

  // 当串口缓冲区大于等于7时
  while (TJC.available() >= FRAME_LENGTH)
  {
    unsigned char ubuffer[FRAME_LENGTH];
    // 从串口缓冲读取1个字节但不删除
    unsigned char frame_header = TJC.peek();
    // 当获取的数据是包头(0x55)时
    if (frame_header == 0x55)
    {
      // 从串口缓冲区读取7字节
      TJC.readBytes(ubuffer, FRAME_LENGTH);
      if (ubuffer[4] == 0xff && ubuffer[5] == 0xff && ubuffer[6] == 0xff)
      {
        int a = ubuffer[2];
        int b = ubuffer[3];
        int param = (b << 8) + a;
        sprintf(str, "msg.txt=\"object=0x%X, param=%d\"\xff\xff\xff", ubuffer[1], param);
        TJC.print(str);
        decode_operations(ubuffer[1], param);
      }
    }
    else
    {
      TJC.read(); // 从串口缓冲读取1个字节并删除
    }
  }
}

// 串口数据格式：
// 串口数据帧长度：7字节
// 帧头      lcd_obj_id   lcd_obj_parameter    帧尾
// 0x55      1字节        2字节                0xffffff
// Main
// 上位机代码  printh 55 10 01 00 ff ff ff  含义：start按钮打开
// 上位机代码  printh 55 11 01 00 ff ff ff  含义：stop按钮打开
// Debug
// 上位机代码  printh 55 00 01 00 ff ff ff  含义：heater按钮打开
// 上位机代码  printh 55 00 00 00 ff ff ff  含义：heater按钮关闭
// 上位机代码  printh 55 04 01 00 ff ff ff  含义：fan按钮打开
// 上位机代码  printh 55 04 00 00 ff ff ff  含义：fan按钮关闭
// debug_extruder
// 上位机代码  printh 55 20 01 00 ff ff ff  含义：extruder按钮打开
// 上位机代码  printh 55 20 00 00 ff ff ff  含义：extruder按钮关闭
// 上位机代码  printh 55 21 01 00 ff ff ff  含义：direction按钮打开
// 上位机代码  printh 55 21 00 00 ff ff ff  含义：direction按钮关闭
// 上位机代码  printh 55 22 xx xx ff ff ff  含义：滑块弹起时的val
// debug_puller
// 上位机代码  printh 55 23 01 00 ff ff ff  含义：puller按钮打开
// 上位机代码  printh 55 23 00 00 ff ff ff  含义：puller按钮关闭
// 上位机代码  printh 55 24 01 00 ff ff ff  含义：direction按钮打开
// 上位机代码  printh 55 24 00 00 ff ff ff  含义：direction按钮关闭
// 上位机代码  printh 55 25 xx xx ff ff ff  含义：滑块弹起时的val
// debug_winder
// 上位机代码  printh 55 26 01 00 ff ff ff  含义：winder按钮打开
// 上位机代码  printh 55 26 00 00 ff ff ff  含义：winder按钮关闭
// 上位机代码  printh 55 27 01 00 ff ff ff  含义：direction按钮打开
// 上位机代码  printh 55 27 00 00 ff ff ff  含义：direction按钮关闭
// 上位机代码  printh 55 28 xx xx ff ff ff  含义：滑块弹起时的val
// debug_stepper4
// 上位机代码  printh 55 29 01 00 ff ff ff  含义：stepper4按钮打开
// 上位机代码  printh 55 29 00 00 ff ff ff  含义：stepper4按钮关闭
// 上位机代码  printh 55 30 01 00 ff ff ff  含义：direction按钮打开
// 上位机代码  printh 55 30 00 00 ff ff ff  含义：direction按钮关闭
// 上位机代码  printh 55 31 xx xx ff ff ff  含义：滑块弹起时的val

void decode_operations(unsigned char lcd_obj_id, int lcd_obj_parameter)
{
  if (lcd_obj_id == 0x0)
  {
    heater_toggle = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x4)
  {
    fan_toggle = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x10)
  {
    heater_toggle = 1;
    extruder_toggle = 1;
    puller_toggle = 1;
    winder_toggle = 1;
    fan_toggle = 1;
  }
  if (lcd_obj_id == 0x11)
  {
    heater_toggle = 0;
    extruder_toggle = 0;
    puller_toggle = 0;
    winder_toggle = 0;
    fan_toggle = 0;
  }
  // Extruder
  if (lcd_obj_id == 0x20)
  {
    extruder_toggle = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x21)
  {
    direction_extruder = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x22)
  {
    speed_extruder = lcd_obj_parameter;
  }
  // Puller
  if (lcd_obj_id == 0x23)
  {
    puller_toggle = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x24)
  {
    direction_puller = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x25)
  {
    speed_puller = lcd_obj_parameter;
  }
  // Winder
  if (lcd_obj_id == 0x26)
  {
    winder_toggle = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x27)
  {
    direction_winder = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x28)
  {
    speed_winder = lcd_obj_parameter;
  }
  // Stepper4
  if (lcd_obj_id == 0x29)
  {
    stepper4_toggle = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x30)
  {
    direction_stepper4 = lcd_obj_parameter ? 1 : 0;
  }
  if (lcd_obj_id == 0x31)
  {
    speed_stepper4 = lcd_obj_parameter;
  }
}

void control_steppers()
{
  extruder.changeSpeed(speed_extruder);
  puller.changeSpeed(speed_puller);
  winder.changeSpeed(speed_winder);
  stepper4.changeSpeed(speed_stepper4);

  extruder.changeDirection(direction_extruder);
  puller.changeDirection(direction_puller);
  winder.changeDirection(direction_winder);
  stepper4.changeDirection(direction_stepper4);

  if (extruder_toggle)
  {
    extruder.control();
  }

  if (puller_toggle)
  {
    puller.control();
  }

  if (winder_toggle)
  {
    winder.control();
  }

  if (stepper4_toggle)
  {
    stepper4.control();
  }

  // This updates the Shift Register Values in each loop
  digitalWrite(SHIFT_REGISTER_LATCHPIN, LOW);
  shiftOut(SHIFT_REGISTER_DATAPIN, SHIFT_REGISTER_CLOCKPIN, MSBFIRST, motorOutput);
  digitalWrite(SHIFT_REGISTER_LATCHPIN, HIGH);
}