#define MOTO1_1 A1              //电机控制引脚，output
#define MOTO1_2 A2
int moto_sta; //电机开关状态0=停1=开窗2=关窗
int control_sta;//窗户开关状态0=关1=开
int percent_sta; //窗户百分比状态
int weather_sta = 0;//天气状况是否适合开窗
int wind_sta = 0;//风力状况
int aqi_sta = 0;//aqi状况
int pm25_sta = 0;//pm25状况
double lux;    // 光线
int time_int;  //校准时间
int percent_yun; //云端百分比，在main和本地百分比状态同步
