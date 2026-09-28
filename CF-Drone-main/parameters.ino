// 参数存储在闪存
// Parameters storage in flash memory

#include <Preferences.h>
#include <nvs_flash.h> // nvs_flash_init()/nvs_flash_erase()：当 Preferences.begin() 失败时用于擦除重建 NVS
#include "util.h"

extern float channelZero[16];
extern float channelMax[16];
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
extern int wifiMode, udpLocalPort, udpRemotePort;
extern int motorPins[4];
extern int pwmFrequency, pwmResolution, pwmStop, pwmMin, pwmMax;
extern float motThrMin;
extern float motThrMax;
extern int mavlinkSysId;
extern Rate telemetrySlow, telemetryFast;
extern float rcLossTimeout, descendTime;
extern int flightModes[3];
extern Vector accBias, accScale;
extern Vector imuRotation;
extern LowPassFilter<Vector> gyroBiasFilter;
extern int rcRxPin;
extern int rcProtocol;   // 遥控协议：0=SBUS, 1=CRSF(ELRS)
extern int rcTxPin;      // 遥测回传 TX 引脚，-1=不启用
extern int rcBaud;       // 串口波特率
extern float trimRoll, trimPitch;         // 软件配平参数，定义于 control.ino
extern float levelGateThreshold;         // 摇杆门控阈值，定义于 estimate.ino
extern float levelBiasGain;              // Mahony I 项增益，定义于 estimate.ino
extern Vector levelGyroBias;             // Mahony 虚拟陀螺偏置，定义于 estimate.ino

Preferences storage;

struct Parameter {
	const char *name; // max length is 15 (Preferences key limit)
	bool integer;
	union { float *f; int *i; };
	float cache; // what's stored in flash
	void (*callback)(); // called after parameter change
	Parameter(const char *name, float *variable, void (*callback)() = nullptr) : name(name), integer(false), f(variable), cache(0), callback(callback) {};
	Parameter(const char *name, int *variable, void (*callback)() = nullptr) : name(name), integer(true), i(variable), cache(0), callback(callback) {};
	float getValue() const { return integer ? *i : *f; }
	void setValue(const float value) { if (integer) *i = (int)value; else *f = value; }
};

Parameter parameters[] = {
	// ===== 控制（角速率内环 PID）=====
	// 角速率环直接控制机身旋转速度，是最内层控制回路。
	// P：比例增益，值越大响应越快，过大会振荡；
	// I：积分增益，消除稳态误差（如电机差异），过大会产生积分饱和；
	// D：微分增益，抑制超调，过大会放大高频噪声；
	// WU（windup）：积分饱和限制幅值，防止 I 项累积过大。
	{"CTL_R_RATE_P",  &rollRatePID.p},      // 横滚角速率 P 增益（rad/s → 力矩）
	{"CTL_R_RATE_I",  &rollRatePID.i},      // 横滚角速率 I 增益
	{"CTL_R_RATE_D",  &rollRatePID.d},      // 横滚角速率 D 增益
	{"CTL_R_RATE_WU", &rollRatePID.windup}, // 横滚角速率积分限幅（±windup）
	{"CTL_P_RATE_P",  &pitchRatePID.p},     // 俯仰角速率 P 增益
	{"CTL_P_RATE_I",  &pitchRatePID.i},     // 俯仰角速率 I 增益
	{"CTL_P_RATE_D",  &pitchRatePID.d},     // 俯仰角速率 D 增益
	{"CTL_P_RATE_WU", &pitchRatePID.windup},// 俯仰角速率积分限幅
	{"CTL_Y_RATE_P",  &yawRatePID.p},       // 偏航角速率 P 增益（偏航惯量小，通常需更大值）
	{"CTL_Y_RATE_I",  &yawRatePID.i},       // 偏航角速率 I 增益
	{"CTL_Y_RATE_D",  &yawRatePID.d},       // 偏航角速率 D 增益

	// ===== 控制（角度外环 PID）=====
	// 角度环将目标姿态角误差转换为角速率指令，输出给内环。
	// I 项用于自动补偿物理不对称引起的固定悬停偏差，需同时配置 WU 积分限幅才能生效。
	{"CTL_R_P",  &rollPID.p},      // 横滚角度 P 增益（角度误差 rad → 角速率指令 rad/s）
	{"CTL_R_I",  &rollPID.i},      // 横滚角度 I 增益，用于补偿物理不对称固定偏差（默认 0.5）
	{"CTL_R_D",  &rollPID.d},      // 横滚角度 D 增益（一般为 0）
	{"CTL_R_WU", &rollPID.windup}, // 横滚角度积分限幅（rad/s），I 项非 0 时必须配置，防止积分发散
	{"CTL_P_P",  &pitchPID.p},     // 俯仰角度 P 增益
	{"CTL_P_I",  &pitchPID.i},     // 俯仰角度 I 增益（默认 0.5）
	{"CTL_P_D",  &pitchPID.d},     // 俯仰角度 D 增益（一般为 0）
	{"CTL_P_WU", &pitchPID.windup},// 俯仰角度积分限幅（rad/s）
	{"CTL_Y_P",  &yawPID.p},       // 偏航角度 P 增益

	// ===== 控制（限制值）=====
	{"CTL_P_RATE_MAX", &maxRate.y}, // 俯仰最大角速率限制（rad/s），超出摇杆指令会被截断
	{"CTL_R_RATE_MAX", &maxRate.x}, // 横滚最大角速率限制（rad/s）
	{"CTL_Y_RATE_MAX", &maxRate.z}, // 偏航最大角速率限制（rad/s）
	{"CTL_TILT_MAX",   &tiltMax},   // 稳定模式最大允许倾斜角（rad），超出时角度外环被限幅

	// ===== 控制（软件配平）=====
	// 用于补偿飞机重心偏移或电机推力不一致导致的悬停偏移，无需物理调整。
	{"CTL_TRIM_ROLL",  &trimRoll},  // 横滚配平角（rad），正值向右滚，调整步长建议 0.005 rad
	{"CTL_TRIM_PITCH", &trimPitch}, // 俯仰配平角（rad），正值向后仰，调整步长建议 0.005 rad

	// ===== 控制（飞行模式）=====
	// 遥控器模式通道（RC_MODE）将摇杆三挡位映射到对应飞行模式枚举值：
	// 0=RAW（直通），1=ACRO（特技），2=STAB（自稳），3=ALTHOLD（气压计定高），4=AUTO
	{"CTL_FLT_MODE_0", &flightModes[0]}, // 模式通道第 0 挡对应的飞行模式
	{"CTL_FLT_MODE_1", &flightModes[1]}, // 模式通道第 1 挡对应的飞行模式
	{"CTL_FLT_MODE_2", &flightModes[2]}, // 模式通道第 2 挡对应的飞行模式

	// ===== IMU（传感器安装旋转补偿）=====
	// 若 IMU 安装方向与机体坐标系不一致，在此设置旋转角（rad）进行软件补偿。
	{"IMU_ROT_ROLL",  &imuRotation.x}, // IMU 绕机体 X 轴（横滚轴）的安装偏转角（rad）
	{"IMU_ROT_PITCH", &imuRotation.y}, // IMU 绕机体 Y 轴（俯仰轴）的安装偏转角（rad）
	{"IMU_ROT_YAW",   &imuRotation.z}, // IMU 绕机体 Z 轴（偏航轴）的安装偏转角（rad）

	// ===== IMU（陀螺仪偏置滤波）=====
	// 陀螺仪静态偏置通过低通滤波器（指数移动平均）在上电初始化时估计。
	// alpha 越小滤波越强（收敛越慢），越大收敛越快但对振动噪声更敏感。
	{"IMU_GYRO_BIAS_A", &gyroBiasFilter.alpha}, // 陀螺偏置低通滤波器系数 alpha（0~1）

	// ===== IMU（加速度计标定）=====
	// 出厂或重新安装后需对加速度计进行六面标定，结果存入以下参数。
	// 标定公式：acc_corrected = (acc_raw - bias) * scale
	{"IMU_ACC_BIAS_X", &accBias.x},  // 加速度计 X 轴零偏（m/s²）
	{"IMU_ACC_BIAS_Y", &accBias.y},  // 加速度计 Y 轴零偏（m/s²）
	{"IMU_ACC_BIAS_Z", &accBias.z},  // 加速度计 Z 轴零偏（m/s²）
	{"IMU_ACC_SCALE_X", &accScale.x}, // 加速度计 X 轴增益校正因子（无量纲，理想值为 1.0）
	{"IMU_ACC_SCALE_Y", &accScale.y}, // 加速度计 Y 轴增益校正因子
	{"IMU_ACC_SCALE_Z", &accScale.z}, // 加速度计 Z 轴增益校正因子

	// ===== 姿态估计 =====
	// 采用互补滤波：陀螺仪积分提供高频动态，加速度计提供低频重力修正。
	{"EST_ACC_WEIGHT",   &accWeight},          // 加速度计对姿态的修正权重（0~1），越大修正越强，但对振动越敏感
	{"EST_RATES_LPF_A",  &ratesFilter.alpha},  // 角速率低通滤波器系数 alpha（0~1），越小截止频率越低（约 40Hz@0.2）
	{"EST_LVL_GATE_THR", &levelGateThreshold}, // 摇杆门控阈值（0~1），摇杆偏转超过此比例时 applyLevel 权重渐变为零，防止打杆后松杆漂移
	{"EST_LVL_BIAS_GAIN",&levelBiasGain},      // Mahony I 项增益：重力误差积分进虚拟陀螺偏置的速率，越大收敛越快（约 30s@0.00002）

	// ===== 电机（引脚配置）=====
	// 修改后立即调用 setupMotors() 重新初始化 LEDC 通道，无需重启。
	{"MOT_PIN_FL", &motorPins[MOTOR_FRONT_LEFT],  setupMotors}, // 左前电机 GPIO 引脚号
	{"MOT_PIN_FR", &motorPins[MOTOR_FRONT_RIGHT], setupMotors}, // 右前电机 GPIO 引脚号
	{"MOT_PIN_RL", &motorPins[MOTOR_REAR_LEFT],   setupMotors}, // 左后电机 GPIO 引脚号
	{"MOT_PIN_RR", &motorPins[MOTOR_REAR_RIGHT],  setupMotors}, // 右后电机 GPIO 引脚号

	// ===== 电机（PWM 配置）=====
	// 直驱 MOSFET 模式：pwmMax=-1，pwmStop/pwmMin=0，使用纯占空比控制。
	// ESC 模式：pwmMax 设为脉宽上限（μs），pwmMin 设为脉宽下限，pwmFrequency 降至 400Hz。
	{"MOT_PWM_FREQ", &pwmFrequency, setupMotors}, // PWM 频率（Hz），MOSFET 直驱建议 25000，ESC 建议 400
	{"MOT_PWM_RES",  &pwmResolution, setupMotors},// PWM 分辨率（bit），决定 LEDC 计数上限为 2^n-1，常用 10bit
	{"MOT_PWM_STOP", &pwmStop},                   // 电机停转时的 PWM 值；纯占空比模式下为 0，ESC 模式下为怠速脉宽（μs）
	{"MOT_PWM_MIN",  &pwmMin},                    // 电机最小 PWM 值；纯占空比模式下为 0，ESC 模式下为最低脉宽（μs）
	{"MOT_PWM_MAX",  &pwmMax},                    // 电机最大 PWM 值；-1=纯占空比模式（MOSFET），正整数=ESC 脉宽上限（μs）

	// ===== 电机（推力映射）=====
	// 将归一化油门（0~1）线性映射到电机推力输出范围，保留余量供姿态修正使用。
	{"MOT_THR_MIN", &motThrMin}, // 推力下限（0~1），摇杆最低位时的电机输出，建议≥0.05 防失速
	{"MOT_THR_MAX", &motThrMax}, // 推力上限（0~1），摇杆最高位时的电机输出，建议≤0.9 保留姿态修正余量

	// ===== 遥控（通道校准）=====
	// 原始 RC 信号经校准转换为归一化值（-1~1 或 0~1）。
	// channelZero[n]：摇杆中立点原始值；channelMax[n]：摇杆最大行程原始值。
	// 校准公式：normalized = (raw - zero) / max
	{"RC_ZERO_0", &channelZero[0]}, // 通道 0 中立点（校准用）
	{"RC_ZERO_1", &channelZero[1]}, // 通道 1 中立点
	{"RC_ZERO_2", &channelZero[2]}, // 通道 2 中立点
	{"RC_ZERO_3", &channelZero[3]}, // 通道 3 中立点
	{"RC_ZERO_4", &channelZero[4]}, // 通道 4 中立点
	{"RC_ZERO_5", &channelZero[5]}, // 通道 5 中立点
	{"RC_ZERO_6", &channelZero[6]}, // 通道 6 中立点
	{"RC_ZERO_7", &channelZero[7]}, // 通道 7 中立点
	{"RC_MAX_0",  &channelMax[0]},  // 通道 0 行程幅值（校准用）
	{"RC_MAX_1",  &channelMax[1]},  // 通道 1 行程幅值
	{"RC_MAX_2",  &channelMax[2]},  // 通道 2 行程幅值
	{"RC_MAX_3",  &channelMax[3]},  // 通道 3 行程幅值
	{"RC_MAX_4",  &channelMax[4]},  // 通道 4 行程幅值
	{"RC_MAX_5",  &channelMax[5]},  // 通道 5 行程幅值
	{"RC_MAX_6",  &channelMax[6]},  // 通道 6 行程幅值
	{"RC_MAX_7",  &channelMax[7]},  // 通道 7 行程幅值

	// ===== 遥控（通道功能映射）=====
	// 指定各功能轴使用的物理通道编号（0-based 整数）。
	{"RC_ROLL",     &rollChannel},     // 横滚通道编号
	{"RC_PITCH",    &pitchChannel},    // 俯仰通道编号
	{"RC_THROTTLE", &throttleChannel}, // 油门通道编号
	{"RC_YAW",      &yawChannel},      // 偏航通道编号
	{"RC_MODE",     &modeChannel},     // 飞行模式切换通道编号（三挡拨杆）

	// ===== 遥控（串口硬件配置）=====
	// 四个参数均绑定 setupRC 回调：修改后立即重新初始化 RC 串口，无需重启即可生效。
	{"RC_RX_PIN",   &rcRxPin,    setupRC}, // 遥控接收器数据输入 GPIO 引脚（UART RX）
	{"RC_PROTOCOL", &rcProtocol, setupRC}, // 遥控协议：0=SBUS（电平反相），1=CRSF/ELRS（正逻辑 420000 baud）
	{"RC_TX_PIN",   &rcTxPin,    setupRC}, // 遥测回传 GPIO 引脚（UART TX）；-1=不启用回传
	{"RC_BAUD",     &rcBaud,     setupRC}, // 遥控串口波特率（bps）；SBUS=100000，CRSF=420000

	// ===== WiFi =====
	{"WIFI_MODE",     &wifiMode},      // WiFi 工作模式：0=关闭，1=STA（连接已有热点），2=AP（自建热点）
	{"WIFI_LOC_PORT", &udpLocalPort},  // 本地 UDP 监听端口（地面站发送到此端口）
	{"WIFI_REM_PORT", &udpRemotePort}, // 远端 UDP 目标端口（飞控主动发送到此端口）

	// ===== MAVLink 遥测 =====
	{"MAV_SYS_ID",    &mavlinkSysId},       // MAVLink 系统 ID（1~254），区分多机时需唯一
	{"MAV_RATE_SLOW", &telemetrySlow.rate}, // 慢速遥测发送频率（Hz），用于心跳、电池等低频数据
	{"MAV_RATE_FAST", &telemetryFast.rate}, // 快速遥测发送频率（Hz），用于姿态、角速率等高频数据

	// ===== 故障保护 =====
	{"SF_RC_LOSS_TIME",  &rcLossTimeout}, // RC 信号丢失超时阈值（秒），超时后进入自动下降模式
	{"SF_DESCEND_TIME",  &descendTime},   // 自动下降至油门归零所需时间（秒），越小下降越快
};

void setupParameters() {
	print("Setup parameters\n");
	if (!storage.begin("flix", false)) {
		// Preferences.begin() 内部 nvs_flash_init() 失败会直接返回 false（常见于分区表变更/NVS数据版本不兼容/异常断电导致的NVS损坏），
		// 此时按 ESP-IDF 标准做法擦除分区后重试，否则会永久静默失败（所有参数都无法持久化）。
		print("[NVS] Preferences.begin 失败，尝试擦除 NVS 分区并重新初始化...\n");
		nvs_flash_erase();
		esp_err_t err = nvs_flash_init();
		print("[NVS] nvs_flash_init 重试结果: err=%d\n", err);
		if (!storage.begin("flix", false)) {
			print("[NVS] 二次初始化仍然失败，参数将无法持久化，请检查分区表/硬件！\n");
		} else {
			print("[NVS] 擦除重建成功，参数将从代码默认值重新开始（旧数据已丢失，需重新执行 ca/cr 等标定）\n");
		}
	}
	// Read parameters from storage
	for (auto &parameter : parameters) {
		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, parameter.getValue()); // store default value
		}
		float stored = storage.getFloat(parameter.name, parameter.getValue());
		// 整型参数若读到 NaN（flash 损坏），回退到代码默认值并重写
		if (parameter.integer && !isfinite(stored)) {
			stored = (float)parameter.getValue();
			storage.putFloat(parameter.name, stored);
		}
		parameter.setValue(stored);
		parameter.cache = parameter.getValue();
	}
}

int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return parameters[index].getValue();
}

float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			return parameter.getValue();
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			if (parameter.integer && !isfinite(value)) return false; // can't set integer to NaN or Inf
			parameter.setValue(value);
			if (parameter.callback) parameter.callback();
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // sync once per second
	if (motorsActive()) return; // don't use flash while flying, it may cause a delay

	for (auto &parameter : parameters) {
		if (parameter.getValue() == parameter.cache) continue;
		if (isnan(parameter.getValue()) && isnan(parameter.cache)) continue; // handle NAN != NAN
		size_t written = storage.putFloat(parameter.name, parameter.getValue());
		if (written != sizeof(float)) continue; // 写入失败时不更新cache，保留旧值，下一轮 1Hz 周期自动重试
		parameter.cache = parameter.getValue();
	}
}

void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, parameter.getValue());
	}
}

void resetParameters() {
	storage.clear();
	ESP.restart();
}
