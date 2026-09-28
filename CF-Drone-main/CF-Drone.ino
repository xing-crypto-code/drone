// 主程序
// 使用Arduino IDE 2.3.x以上版本编译，不支持老版本的1.8.19！
// 必须安装ESP32开发板核心库（即esp32 by Espressif Systems）和依赖（MAVLINK等）

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "board_config.h"

// WiFi 和 Web 遥控器开关由 board_config.h 按芯片自动设置：
// 如需手动覆盖，在此处 #undef 后重新 #define
#define WIFI_ENABLED    BOARD_WIFI_ENABLED
#define WEB_RC_ENABLED  BOARD_WEB_RC_ENABLED

float t = NAN; // 当前步进时间，单位：秒
float dt; // 与上一步进的时间差，单位：秒
float controlRoll, controlPitch, controlYaw, controlThrottle; // 飞手输入指令，范围 [-1, 1]
float controlMode = NAN;
Vector gyro; // 陀螺仪数据
Vector acc; // 加速度计数据，单位：m/s/s
Vector rates; // 滤波后的角速度，单位：rad/s
Quaternion attitude; // 估计出的姿态（四元数）
bool landed; // are we landed and stationary

void setup() {
	Serial.begin(115200); // 初始化串口，波特率115200
	disableBrownOut(); // 禁用ESP32低压复位检测，防止电机启动瞬间电压跌落导致误复位
	print("琛光科技 CF-Drone Flight Controller\n"); // 品牌标识（LICENSE 附加条款第二条要求固件启动输出保留品牌名称）
	print("程序开始初始化！\n");
	setupParameters(); // 从Flash加载参数（未存储时使用默认值）
	setupLED(); // 初始化状态指示灯
	setupMotors(); // 初始化电机输出（PWM/DShot）
	setLED(true); // 点亮LED，提示正在初始化
#if WIFI_ENABLED
	setupWiFi(); // 初始化WiFi（用于Web遥控/MAVLink等）
#endif
#if WEB_RC_ENABLED
	setupWebRC();  // 初始化Web遥控器
#endif
	setupIMU(); // 初始化IMU（陀螺仪/加速度计）
	setupRC(); // 初始化遥控接收机（SBUS/ELRS等协议）
	setLED(false); // 熄灭LED，提示初始化完成
	print("程序初始化完成！\n");
	print("================================\n");
}

void loop() {
	readIMU(); // 读取IMU原始数据（陀螺仪/加速度计），并完成校准与坐标旋转
	step(); // 计算主循环步进时间 t 与时间差 dt，并统计循环频率
	readRC(); // 读取遥控接收机输入
#if WEB_RC_ENABLED
	readWebRC();  // 读取Web遥控器输入
	processConsoleCommandQueue(); // 将网页命令放到主循环执行，避免阻塞HTTP回调
#endif
	estimate(); // 姿态与状态估计（互补滤波融合IMU数据）
	updateBatteryVoltage(); // 更新电池电压采样与低电压保护判断
	control(); // 飞控核心：姿态环PID解算，输出电机控制量
	sendMotors(); // 将电机控制量输出到电机（PWM/DShot）
	handleInput(); // 处理串口/Web控制台输入命令
#if WIFI_ENABLED
	processMavlink(); // 处理MAVLink通信
#endif
	logData(); // 记录飞行日志数据
	syncParameters(); // 参数变更后延迟写入Flash，避免频繁擦写
	updateLED(); // 根据当前飞行状态刷新LED指示效果
}
