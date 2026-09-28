//实用函数
// Utility functions
//
// ==================== 这个文件是干什么的 ====================
// 各种被全项目到处用的小工具，按照「从简单到复杂」排列：
//   mapf()           → 把一个区间的数值线性映射到另一个区间
//   valid/invalid()  → 判断浮点数是不是有效值（不是 NaN/Inf）
//   wrapAngle()      → 把角度规整到 [-π, π)
//   disableBrownOut()→ 关掉 ESP32 的欠压复位（防止电机启动瞬间掉电重启）
//   splitString()    → 把一行命令按空格拆成最多 3 段（CLI 命令解析用）
//   Rate             → 定时器：控制"每秒执行 N 次"
//   Delay            → 延迟确认：一个信号要"持续成立"够久才算真

#pragma once

#include <math.h>
// 下面这段是"按芯片型号"选择不同的欠压复位寄存器访问方式：
//   经典 ESP32  → 直接操作 RTC_CNTL 寄存器
//   ESP32-S3/C3 → 调用 ESP-IDF 提供的 esp_brownout_disable()
// CONFIG_IDF_TARGET_xxx 是编译时由 Arduino-ESP32 自动定义的宏，
// 整个项目就是靠它来区分不同芯片的（见 board_config.h）
#ifdef CONFIG_IDF_TARGET_ESP32
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#include "esp_private/brownout.h"
#endif

const float ONE_G = 9.80665;  // 标准重力加速度（m/s²），用于加速度计校准与"落地判断"
extern float t;               // 全局当前时间（秒），由 time.ino 的 step() 更新

// 线性映射：把 x 从 [in_min, in_max] 等比例映射到 [out_min, out_max]
// 例：mapf(0.5, 0, 1, 1000, 2000) = 1500
// 与 Arduino 自带的 map() 区别：专门给 float 用，不会做整数截断
// 本项目用途：油门摇杆值 → 推力值（control.ino）、PWM 占空比换算（motors.ino）
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 判断数值是否无效（NaN 或 无穷大）
// isfinite(x) = "x 是有限值"，取反就是"无效"
// 本项目大量用 invalid() 来表示"这个量当前不该参与控制"，
// 例如 attitudeTarget.invalidate() 就是把四元数置成 NaN 来禁用姿态环
bool invalid(float x) {
	return !isfinite(x);
}

// 判断数值是否有效（有限值）
bool valid(float x) {
	return isfinite(x);
}

// 把角度规整到 [-π, π) 区间
// 为什么需要：偏航角可以无限累积（比如转 10 圈 = 20π），
// 但误差应该取"最近的那一边"。
// 例：目标 179°，实际 -179°，直接相减 = 358°（看着要转一大圈），
//     规整后 = -2°（其实只需转一点点）。
float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * PI);
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}

// 关闭「欠压复位」（Brown-out Reset）
// 为什么需要：4 个电机同时启动瞬间，电池电压会被拉低。
// 如果 ESP32 的欠压检测还开着，它会误以为供电不足 → 直接复位重启 → 空中失控炸机。
// 所以开机第一件事就是关掉它（见 CF-Drone.ino 的 setup()）。
// 代价：真的电压过低时芯片不会自动保护，靠软件的低电保护逻辑兜底（safety.ino / battery.ino）
void disableBrownOut() {
#ifdef CONFIG_IDF_TARGET_ESP32
	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
	esp_brownout_disable();
#endif
}

// 把字符串按空格切分成最多 3 段（CLI 命令解析用）
// 例：输入 "p CTL_R_P 6" → token0="p", token1="CTL_R_P", token2="6"
// 注意用的是 C 的 strtok()，必须先把 String 转成 char 数组（strtok 会原地修改内容）
void splitString(String& str, String& token0, String& token1, String& token2) {
	str.trim();
	char chars[str.length() + 1];
	str.toCharArray(chars, str.length() + 1);
	token0 = strtok(chars, " ");
	token1 = strtok(NULL, " "); // String(NULL) creates empty string
	token2 = strtok(NULL, "");
}

// ==================== 限速器：控制"每秒最多执行 N 次" ====================
// 用法（典型写法）：
//     static Rate periodic(10);   // 10Hz
//     if (periodic) { ... }       // 每 100ms 才进去一次，其余时间自动跳过
// 原理：记录上次触发的时刻 last，距今满 1/rate 秒就返回 true 并刷新 last。
//
// 本项目用途：
//   • MAVLink 遥测降速      → mavlink.ino 的 telemetrySlow / telemetryFast
//   • 日志采样限速          → log.ino 的 Rate period(100)
//
// ★ 坑点：它是"非阻塞"的，靠 operator bool() 隐式转换实现，
//   所以能直接写在 if 条件里，看起来很简洁但读代码时容易看漏。
class Rate {
public:
	float rate;       // 期望频率（Hz）
	float last = 0;   // 上次触发的时间戳
	Rate(float rate) : rate(rate) {}

	// 隐式转换为 bool：到时间了返回 true（并把 last 更新为当前时间）
	operator bool() {
		if (t - last >= 1 / rate) {
			last = t;
			return true;
		}
		return false;
	}
};

// ==================== 延迟确认：信号必须"持续成立"够久才算数 ====================
// 为什么要它：单一时刻的读数不可信（噪声、偶发毛刺）。
// 比如"飞机是否已静止落地"——抖动一下不能算落地，得稳定持续 2 秒才认。
//
// 用法（典型写法）：
//     static Delay landedDelay(2);           // 需要连续成立 2 秒
//     if (!landedDelay.update(landed)) return;  // 还没满 2 秒 → 直接返回
//
// 本项目用途：imu.ino 的 calibrateGyroOnce() 用它确保飞机真的静止了才校准陀螺零偏。
//
// 内部状态 start = NaN 表示"当前信号不成立、计时未开始"
class Delay {
public:
	float delay;        // 需要持续成立的时间（秒）
	float start = NAN;  // 开始计时的时刻；NAN = 未开始计时
	Delay(float delay) : delay(delay) {}

	bool update(bool on) {
		if (!on) {
			// 信号断了 → 计时清零，下次要重新累计（必须是"连续"成立）
			start = NAN;
			return false;
		} else if (isnan(start)) {
			start = t;  // 信号刚变成立，开始计时
		}
		return t - start >= delay;  // 累计时间是否已达标
	}
};
