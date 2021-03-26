#define MOTO1_1 A1              //电机控制引脚，output
#define MOTO1_2 A2
int moto_sta; //电机开关状态0=停1=开窗2=关窗
int control_sta;//窗户开关状态0=关1=开
int percent_sta; //窗户百分比状态
int weather_sta = 0;//天气状况是否适合开窗
int wind_sta = 0;//风力状况
int aqi_sta = 0;//aqi状况
int pm25_sta = 0;//pm25状况
int time_int;  //校准时间,存入eeprom
int percent_yun; //云端百分比，在main和本地百分比状态同步
int cali_int;  //校准状态，0-无动作，1-校准准备，2-开始校准，3-校准中
byte auto_mode;//自动模式，0-false,1-true
byte localtime_int;                  //存放同步的本地时间小时数
