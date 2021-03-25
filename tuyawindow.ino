//改LGT328芯片
//上报信息给涂鸦模组，测试正常，下一步是1.写手动开窗代码：关窗状态检测到右开关低电平则开窗（MOTO1_1高电平，MOTO1_2低电平），开窗状态检测到左边低电平则关窗（MOTO1_2高电平，MOTO1_1低电平）！（测试正常待硬件测试了）
//2.实现APP控制代码(基本搞清楚了)！接入433模块代码，实现433控制开关窗！运行正常，代码实现收到5836257关窗5836258开窗
//新增I2C模块tsl2561光线模块，如果能兼容应该其它i2C模块也没问题！开始接入A4-SDA,A5-SCL,demo测试OK，开始写代码
//3.实现百分比控制，校准的运行时长数据和百分比数据作为全局变量，收到百分比控制数据后和已存在的百分比数据对比（开关控制时注意，关百分比数据存为0，开百分比数据存为100）
//如果下发数据大于现存的百分比数据则开窗百分之它们想减之后的值*总校准秒数！反之就关窗这么多！
//天气逻辑怎么实现呢？每5分钟同步一下时间，到早上9点如果空气质量还行，没有雨雪，风不大则开窗？，晚上6点如果窗户还开着就关窗？或者和光线传感器联动

#include "mcu_api.h"
#include "protocol.h"
#include "wifi.h"
#include "EEPROM.h"
#include <SparkFunTSL2561.h>
#include <Wire.h>
#include <Bounce2.h>
#include <MsTimer2.h>
#include <RCSwitch.h>

#define LEFT_MAX_PIN  D4        //左边限位，拉高，低电平是开窗状态
#define RIGHT_MAX_PIN  D5       //右边限位，拉高，低电平是关窗状态
#define MOTO1_1 A1              //电机控制引脚，output
#define MOTO1_2 A2
#define BATTERY_SENSE_PIN  A0  // 电量检测A0
#define VMIN 1.8
#define VMAX 3.1

float lastVcc = 3.3 ;           //电池电压
uint16_t lastlux = 0;            //光线
int batteryPcnt ;
int ss = 0;                          //计秒变量
extern int moto_sta; //电机开关状态0=停1=开窗2=关窗
extern int control_sta;//窗户开关状态0=关1=开
extern int percent_sta; //窗户百分比状态
extern int weather_sta ;//天气状况是否适合开窗
extern int wind_sta;//风力状况
extern int aqi_sta;//aqi状况
extern int pm25_sta;//pm25状况
extern double lux;    // 光线
extern int time_int;  //校准时间
extern int percent_yun; //云端百分比，在main和本地百分比状态同步

// Instantiate a Bounce object
Bounce debouncer_left = Bounce(); 
Bounce debouncer_right = Bounce(); 

RCSwitch mySwitch = RCSwitch();
SFE_TSL2561 light;
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds
unsigned int data0, data1;
boolean light_in;

extern void mcu_open_weather(void);
extern void mcu_get_system_time(void);
extern void request_weather_serve(void);

void one_s() {
  
  ss++;
  light_in = false;
  //Serial.print(ss);
  //Serial.print("\n");
}

void setup() {
  pinMode(MOTO1_1,OUTPUT);
  pinMode(MOTO1_2,OUTPUT);
  pinMode(LEFT_MAX_PIN,INPUT_PULLUP);
  pinMode(RIGHT_MAX_PIN,INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  analogReference(INTERNAL);    //使用1.1V为基准电压
  // After setting up the pins, setup debouncer
  debouncer_left.attach(LEFT_MAX_PIN);
  debouncer_left.interval(500);
  debouncer_right.attach(RIGHT_MAX_PIN);
  debouncer_right.interval(500);
  
  wifi_protocol_init();
  digitalWrite(MOTO1_1, LOW);
  digitalWrite(MOTO1_2, LOW);
  //cali = EEPROM.read(0);  //这里预留读内存取校准数据放入全局变量
  //byte bit1 = EEPROM.read(1);
  
  mySwitch.enableReceive(0);  // 外部中断 0 => 433模块接引脚D2
  light.begin();
  unsigned char ID;
  
  if (light.getID(ID))
  {
    Serial.print("Got factory ID: 0X");
    Serial.print(ID,HEX);
    Serial.println(", should be 0X5X");
  } else {
    byte error = light.getError();
    printError(error);
  }

  gain = 0;
  unsigned char time = 2;
  light.setTiming(gain,time,ms);
  light.setPowerUp();
  MsTimer2::set(1000, one_s); // 用Timer2计时一秒钟，执行一次one_s函数
  MsTimer2::start();
  mcu_open_weather();//打开天气服务
}

void loop() {
  double lux;    // Resulting lux value
  boolean good;  // True if neither sensor is saturated
  unsigned int data0, data1;
  // Update the debouncer
  boolean stateChanged_left = debouncer_left.update();
  boolean stateChanged_right = debouncer_right.update();
 // Get the update value
 //先更新百分比和窗户状态，左开关低电平百分比100，窗户状态开1，停电机状态0；右开关低电平百分比0停电机0,
 int value_left = debouncer_left.read();//左开关状态，低电平则为开窗状态，检测到状态改变为高电平，且电机为停止0状态则执行关窗动作，电机状态2
 int value_right = debouncer_right.read();//右开关状态，低电平则为关窗状态，检测到状态改变为高电平，且电机为停止0状态则执行开窗动作，电机状态1
  wifi_uart_service();
  myserialEvent();      // 串口接收处理
  if ( stateChanged_left ) { 
    if (value_left == LOW){ //左开关状态改变低电平证明窗户开到100%了，停电机，上报状态！
      control_sta = 1;
      mcu_dp_enum_update(DPID_CONTROL,0x01);
      percent_sta = 100;
      mcu_dp_value_update(DPID_PERCENT_CONTROL,percent_sta); 
      digitalWrite(MOTO1_1, LOW);
      digitalWrite(MOTO1_2, LOW);
      moto_sta = 0;
      //Serial.print("窗户状态变为打开。\n");
    }else if(value_left == HIGH && moto_sta == 0){ //左开关状态改变为高电平且电机为停止状态，开电机关窗，电机状态变2
      digitalWrite(MOTO1_1, LOW);
      digitalWrite(MOTO1_2, HIGH);
      moto_sta = 2;
      //Serial.print("开始关窗。\n");
    }
  }
  if ( stateChanged_right ) { 
    if (value_right == LOW){ //右开关状态改变低电平证明窗户关了，停电机，上报状态！
      control_sta = 0;
      mcu_dp_enum_update(DPID_CONTROL,0x02);
      percent_sta = 0;
      mcu_dp_value_update(DPID_PERCENT_CONTROL,percent_sta); 
      digitalWrite(MOTO1_1, LOW);
      digitalWrite(MOTO1_2, LOW);
      moto_sta = 0;
      //Serial.print("窗户状态变为关闭。\n");
    }else if(value_right == HIGH && moto_sta == 0){ //右开关状态改变为高电平且电机为停止状态，开电机开窗，电机状态变1
      digitalWrite(MOTO1_1, HIGH);
      digitalWrite(MOTO1_2, LOW);
      moto_sta = 1;
      //Serial.print("开始开窗。\n");
    }
  }

  if (mySwitch.available()) {
    long code433 = mySwitch.getReceivedValue();
    if (code433 == 5836257){
      digitalWrite(MOTO1_1, LOW);
      digitalWrite(MOTO1_2, HIGH);
      moto_sta = 2;
      //Serial.print("开始关窗。\n");
    }else if (code433 == 5836258){
      digitalWrite(MOTO1_1, HIGH);
      digitalWrite(MOTO1_2, LOW);
      moto_sta = 1;
      //Serial.print("开始开窗。\n");
    }
    //Serial.print("Received ");
    Serial.print(code433);
    //Serial.print(" / ");
    //Serial.print( mySwitch.getReceivedBitlength() );
    //Serial.print("bit ");
    //Serial.print("Protocol: ");
    //Serial.println( mySwitch.getReceivedProtocol() );
       
    mySwitch.resetAvailable();
  }
  if (ss % 120 == 0 && light_in == false) {   //5分钟检测一次光线
    light_in = true;
    if (light.getData(data0,data1))
  {
    //Serial.print("data0: ");
    //Serial.print(data0);
    //Serial.print(" data1: ");
    //Serial.print(data1);
    boolean good;  // True if neither sensor is saturated
    good = light.getLux(gain,ms,data0,data1,lux);
    Serial.print("lux: ");
    Serial.println(lux);
    //if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
  }
  else
  {
  
    byte error = light.getError();
    printError(error);
  }

  mcu_get_system_time();  //同步时间
  
   } else if (ss % 180 == 0) { //30分钟获取一次天气，这个不好用
    request_weather_serve();
   }
}

void myserialEvent() {
  if (Serial.available()) {
    unsigned char ch = (unsigned char)Serial.read();
    uart_receive_input(ch);
    //Serial.print(ch);
  }
}

void reportWindowControl(){
  unsigned char window_con = 0x01;     //open的16进制码 close为0x02
  mcu_dp_enum_update(DPID_CONTROL,window_con);       //枚举型数据上报;
  mcu_dp_enum_update(DPID_LEFT_CONTROL,0x02);       //枚举型数据上报;
  mcu_dp_value_update(DPID_RESIDUAL_ELECTRICITY,88); //VALUE型数据上报;
  mcu_dp_value_update(DPID_PERCENT_CONTROL,100); //VALUE型数据上报;
  mcu_dp_bool_update(DPID_CALIBRATION,true); //BOOL型数据上报;
}

//电池检测
void batterylevel(void)
{
  unsigned char batbit;
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  float batteryV = sensorValue * 0.003363075;
  float v = (float) (batteryV - lastVcc);
  if ((batteryV > VMIN ) && ( batteryV < VMAX)) {
    batteryPcnt = int(((batteryV - VMIN) / (VMAX - VMIN)) * 100.);
  }
  else {
    if (batteryV < VMIN ) {
      batteryPcnt = 0 ;
    }
    else {
      batteryPcnt = 100;
    }
  }
  if ((lastVcc != batteryV) && (abs(v) >= 0.1)) {
    mcu_dp_value_update(DPID_RESIDUAL_ELECTRICITY, long(batteryPcnt)); //电池电量
    
    //Serial.print("电池电量: ");
    //Serial.print(batteryPcnt);
    //Serial.print("%\n");
    //Serial.print("电池电压: ");
    //Serial.print(batteryV);
    //Serial.print("V\n");
    lastVcc = batteryV;
  }
}

void key_reset(void)
{
  //mcu_network_start();
  mcu_reset_wifi();
  delay(50);

}
void printError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");
  
  switch(error)
  {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}
