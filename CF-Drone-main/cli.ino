// Implementation of command line interface 

// ==================== 串口命令行界面（CLI）====================
// 这个文件实现"用串口/网页控制台跟飞控对话"的功能。
// 你在串口监视器里敲的那些命令（help / p / ps / ca / arm / log ...，
// 完整清单见下面的 motd 字符串）全部由这里处理。
//
// 命令从哪来（两个入口，最终汇入 handleInput() 解析）：
//   1) 串口监视器  Serial（USB 调试口，波特率 115200）
//   2) 网页控制台  web_rc.ino 的 processConsoleCommandQueue()
//
// 为什么必须有它：
//   飞控装进机身后基本不可能再插线重烧，所以必须能在线操作：
//   • 查看 / 修改参数（p <参数名> <值>）—— 调 PID 就靠它
//   • 校准（ca 校准加速度计、cr 校准遥控器）
//   • 解锁 / 上锁（arm / disarm）
//   • 看飞行日志（log / log dump）
//   可以说，CLI 是做调试和调参的主要手段，比反复烧录高效得多。
//
// ★ 下面 print() 的实现有个值得学的细节：
//   用「栈上小缓冲 + 超长时改用堆内存」的两段式写法，
//   避免固定长度缓冲区把超长内容（比如开机菜单 motd）静默截断。

#include "pid.h"
#include "vector.h"
#include "util.h"
#include "lpf.h"

extern LowPassFilter<Vector> gyroBiasFilter;

#if WEB_RC_ENABLED
extern bool webConsoleEnabled;
extern void webLog(const char* msg);
#endif

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern const int RAW, ACRO, STAB, AUTO;
extern float t, dt, loopRate;
extern float controlTime;
extern uint16_t channels[16];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float motors[4];
extern int mode;
extern bool armed;

const char* motd =
"CLI命令菜单，输入相应命令，回车后执行:\n"
"help - 帮助\n"
"p - 显示所有参数\n"
"p <name> - 显示指定参数\n"
"p <name> <value> - 设置参数\n"
"p MOT_PIN_FL 14 - 参数设置示例，前左电机引脚为14\n"
"preset - 重置参数存储，设置参数后运行此命令\n"
"mfr, mfl, mrr, mrl - 测试马达 (马达不受算法影响运转，为了安全不要装桨叶！！！)\n"
"ca - 校准陀螺仪加速度计\n"
"ps - 显示pitch/roll/yaw姿态\n"
"cr - 校准RC遥控器\n"
"rc - 显示RC遥控数据\n"
"wifi - 显示WiFi信息\n"
"ap <ssid> <password> - 配置AP模式SSID和密码\n"
"sta <ssid> <password> - 配置STA客户端模式\n"
"raw/stab/acro/auto - 飞行模式设定\n"
"arm - 解锁无人机\n"
"disarm - 锁定无人机\n"
"psq - 显示姿态四元数\n"
"imu - 显示IMU数据\n"
"time - 显示时间信息\n"
"mot - 显示motor输出\n"
"sys - 显示系统info信息\n"
"log [dump] - 打印日志\n"
"reboot - 重启无人机\n"
"reset - 重置无人机\n";

void print(const char* format, ...) {
	// 固定 1000 字节缓冲区 + vsnprintf 会静默截断超长内容（例如开机菜单 motd），且截断后连换行符都可能丢失，
	// 导致后续打印内容拼接到同一行。这里先用栈上小缓冲区尝试格式化，若实际所需长度超过缓冲区，
	// 再按精确所需大小临时用堆内存重新格式化，避免任何长度的内容被静默截断。
	char stackBuf[512];
	va_list args, argsCopy;
	va_start(args, format);
	va_copy(argsCopy, args);
	int needed = vsnprintf(stackBuf, sizeof(stackBuf), format, argsCopy);
	va_end(argsCopy);
	if (needed < 0) needed = 0; // 编码错误兵底，避免负数导致后续内存分配异常

	char *buf = stackBuf;
	bool heapAllocated = false;
	if (needed >= (int)sizeof(stackBuf)) {
		buf = (char*)malloc(needed + 1);
		if (buf) {
			vsnprintf(buf, needed + 1, format, args);
			heapAllocated = true;
		} else {
			buf = stackBuf; // 内存不足兵底：退回已截断的栈缓冲区内容
		}
	}
	va_end(args);

	Serial.print(buf);
#if WIFI_ENABLED
	mavlinkPrint(buf);
#endif
#if WEB_RC_ENABLED
	if (webConsoleEnabled) webLog(buf);
#endif
	if (heapAllocated) free(buf);
}

void pause(float duration) {
	float start = t;
	while (t - start < duration) {
		readIMU(); // 长时间阻塞命令（ca/cr）期间也需要持续刷新IMU/姿态，否则打印信息会定格在进入pause前的旧值
		step();
		estimate();
		handleInput();
#if WIFI_ENABLED
		processMavlink();
#endif
#if WEB_RC_ENABLED
		readWebRC(); // 保持 HTTP 服务器在长时间命令（ca/cr）期间持续响应
#endif
		delay(50);
	}
}

void doCommand(String str, bool echo = false) {
	// parse command
	String command, arg0, arg1;
	splitString(str, command, arg0, arg1);
	if (command.isEmpty()) return;

	// echo command
	if (echo) {
		print("> %s\n", str.c_str());
	}

	command.toLowerCase();

	// execute command
	if (command == "help" || command == "motd") {
		print("%s\n", motd);
	} else if (command == "p" && arg0 == "") {
		printParameters();
	} else if (command == "p" && arg0 != "" && arg1 == "") {
		print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
	} else if (command == "p") {
		bool success = setParameter(arg0.c_str(), arg1.toFloat());
		if (success) {
			print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
		} else {
			print("Parameter not found: %s\n", arg0.c_str());
		}
	} else if (command == "preset") {
		resetParameters();
	} else if (command == "time") {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (command == "ps") {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (command == "psq") {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (command == "imu") {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (command == "arm") {
		extern bool imuOK;
		if (!imuOK) { print("IMU故障，禁止解锁！\n"); }
		else armed = true;
	} else if (command == "disarm") {
		armed = false;
	} else if (command == "raw") {
		mode = RAW;
	} else if (command == "stab") {
		mode = STAB;
	} else if (command == "acro") {
		mode = ACRO;
	} else if (command == "auto") {
		mode = AUTO;
	} else if (command == "rc") {
		print("channels: ");
		for (int i = 0; i < 16; i++) {
			print("%u ", channels[i]);
		}
		print("\nroll: %g pitch: %g yaw: %g throttle: %g mode: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("time: %.1f\n", controlTime);
		print("mode: %s\n", getModeName());
		print("armed: %d\n", armed);
	} else if (command == "wifi") {
#if WIFI_ENABLED
		printWiFiInfo();
#endif
	} else if (command == "ap") {
#if WIFI_ENABLED
		configWiFi(true, arg0.c_str(), arg1.c_str());
#endif
	} else if (command == "sta") {
#if WIFI_ENABLED
		configWiFi(false, arg0.c_str(), arg1.c_str());
#endif
	} else if (command == "mot") {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (command == "log") {
		printLogHeader();
		if (arg0 == "dump") printLogData();
	} else if (command == "cr") {
		calibrateRC();
	} else if (command == "ca") {
		calibrateAccel();
	} else if (command == "mfr") {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (command == "mfl") {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (command == "mrr") {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (command == "mrl") {
		testMotor(MOTOR_REAR_LEFT);
	} else if (command == "sys") {
#ifdef ESP32
		print("Chip: %s\n", ESP.getChipModel());
		print("Temperature: %.1f °C\n", temperatureRead());
		print("Free heap: %d\n", ESP.getFreeHeap());
		// Print tasks table
		print("Num  Task                Stack  Prio  Core  CPU%%\n");
		int taskCount = uxTaskGetNumberOfTasks();
		TaskStatus_t *systemState = new TaskStatus_t[taskCount];
		uint32_t totalRunTime;
		uxTaskGetSystemState(systemState, taskCount, &totalRunTime);
		for (int i = 0; i < taskCount; i++) {
			String core = systemState[i].xCoreID == tskNO_AFFINITY ? "*" : String(systemState[i].xCoreID);
			int cpuPercentage = systemState[i].ulRunTimeCounter / (totalRunTime / 100);
			print("%-5d%-20s%-7d%-6d%-6s%d\n",systemState[i].xTaskNumber, systemState[i].pcTaskName,
				systemState[i].usStackHighWaterMark, systemState[i].uxCurrentPriority, core.c_str(), cpuPercentage);
		}
		delete[] systemState;
#endif
	} else if (command == "reset") {
		attitude = Quaternion();
		gyroBiasFilter.reset();
	} else if (command == "reboot") {
		ESP.restart();
	} else {
		print("Invalid command: %s\n", command.c_str());
	}
}

void handleInput() {
	static bool showMotd = true;
	static String input;

	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n') {
			doCommand(input);
			input.clear();
		} else {
			input += c;
		}
	}
}
