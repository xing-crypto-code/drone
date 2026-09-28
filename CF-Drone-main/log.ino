// RAM日志记录
// In-RAM logging
//
// ==================== 这个文件是干什么的 ====================
// 飞行数据的"黑匣子"。在内存里滚动记录飞行过程中的关键量，
// 出问题后可以用 CLI 命令 `log dump` 导出分析。
// 例：松杆后飞机往左漂？看日志里 rates 和 attitude 就能判断是
//     PID 参数问题，还是机械不对称（重心偏了/电机推力不均）。
//
// ==================== 关键设计：只存在内存，绝不写 Flash ====================
// 为什么不写 Flash：
//   • Flash 擦写寿命有限（约 10 万次），100Hz 采样几秒就能写爆它
//   • 写 Flash 会阻塞主循环，飞控一卡就可能摔机
// 所以用环形缓冲区 logBuffer，写满就从头覆盖，只保留最近 N 秒数据。
// 缓冲时长由 board_config.h 按芯片决定：
//   ESP32 / ESP32-S3 → 8 秒（约占 45KB RAM）
//   ESP32-C3         → 4 秒（省约 22KB RAM，因为 C3 内存紧张）
//
// ==================== 记录什么 ====================
// 见下面的 logEntries[] 表。核心是「目标值 vs 实际值」两组：
//   rates         实际三轴角速度     ratesTarget     目标三轴角速度
//   attitude      实际姿态角         attitudeTarget  目标姿态角
// 有了这两组数据，就能看出 PID 是"不够力"还是"过头振荡"。
//
// 采样率：LOG_RATE = 100Hz，与 1kHz 主循环解耦（用 Rate 限速器控制）
// 只在解锁后（armed）记录，避免地面待机时白白把缓冲区写满。

#include "vector.h"
#include "util.h"

#include "board_config.h"

#define LOG_RATE 100
#define LOG_DURATION BOARD_LOG_DURATION  // 缓冲时长：C3=4秒（节省~22KB RAM）/ ESP32&S3=8秒
#define LOG_SIZE LOG_DURATION * LOG_RATE

Vector attitudeEuler;
Vector attitudeTargetEuler;

struct LogEntry {
	const char *name;
	float *value;
};

LogEntry logEntries[] = {
	{"t", &t},
	{"rates.x", &rates.x},
	{"rates.y", &rates.y},
	{"rates.z", &rates.z},
	{"ratesTarget.x", &ratesTarget.x},
	{"ratesTarget.y", &ratesTarget.y},
	{"ratesTarget.z", &ratesTarget.z},
	{"attitude.x", &attitudeEuler.x},
	{"attitude.y", &attitudeEuler.y},
	{"attitude.z", &attitudeEuler.z},
	{"attitudeTarget.x", &attitudeTargetEuler.x},
	{"attitudeTarget.y", &attitudeTargetEuler.y},
	{"attitudeTarget.z", &attitudeTargetEuler.z},
	{"thrustTarget", &thrustTarget}
};

const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
float logBuffer[LOG_SIZE][logColumns];

void prepareLogData() {
	attitudeEuler = attitude.toEuler();
	attitudeTargetEuler = attitudeTarget.toEuler();
}

void logData() {
	if (!armed) return;
	static int logPointer = 0;
	static Rate period(LOG_RATE);
	if (!period) return;

	prepareLogData();

	for (int i = 0; i < logColumns; i++) {
		logBuffer[logPointer][i] = *logEntries[i].value;
	}

	logPointer++;
	if (logPointer >= LOG_SIZE) {
		logPointer = 0;
	}
}

void printLogHeader() {
	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}
}

void printLogData() {
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // skip empty records
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
