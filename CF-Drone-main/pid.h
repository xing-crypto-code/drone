// PID控制器实现
// PID controller implementation
//
// ==================== 这个文件是干什么的 ====================
// 飞控的"大脑"之一：根据「目标值 - 实际值」的误差，算出一个修正量。
//
// 本项目用的是「串级 PID」（两级串联），定义在 control.ino：
//   外环（角度环）：姿态角误差  → 目标角速率      controlAttitude()
//   内环（角速率环）：角速率误差 → 电机力矩        controlRates()
//   ─────────────────────────────────────────────────────────
//   为什么串两级？外环只管"飞机躺平没有"，内环只管"转多快"。
//   内环跑得快、抗扰动强，外环再稳的飞机也飞得稳，这是多旋翼标准做法。
//
// ==================== P / I / D 三项分别在干嘛 ====================
//   P（比例）proportional
//       误差越大，修正越猛。反应最快。
//       缺点：只有 P 会留下"稳态误差"（永远差一点），且 P 太大容易来回振荡。
//
//   I（积分）integral
//       把历史误差"累加"起来，专门消除那个"永远差一点"的稳态误差。
//       缺点：会累积过头（叫「积分饱和 / windup」），松杆后飞机会自己漂。
//       所以本项目必须配 windup 限幅（见下方说明），否则很危险。
//
//   D（微分）derivative
//       看误差的"变化速度"，起阻尼作用、抑制超调（让它别冲过头）。
//       缺点：会把传感器噪声放大成剧烈抖动 —— 所以本项目给 D 项加了低通滤波。
//
// ==================== 为什么到处都要乘 dt ====================
// 误差要按「时间」累积而不是按「循环次数」。如果不乘 dt，
// 主循环频率一变（改代码/CPU 负载变化），同一组 PID 参数表现就完全不同。
// 本项目主循环约 1kHz，dt 由 time.ino 的 step() 计算，全局变量 t 是当前时刻。

#pragma once

#include "lpf.h"

class PID {
public:
	float p, i, d;      // 三个增益。全部可在飞控运行时在线修改（参数名 CTL_*_P / CTL_*_I / CTL_*_D）
	float windup;       // 积分输出限幅。把 I 项限制在 ±windup 之内，防止积分饱和（炸机常见原因）
	float dtMax;        // dt 上限。两次调用间隔超过它就判定"数据异常"，清空积分与微分

	float derivative = 0;   // 最近一次算出的 D 项数值（调试用，log.ino 会记录）
	float integral = 0;     // 当前累计的积分量（调试用）

	LowPassFilter<float> lpf; // D 项低通滤波：滤掉噪声，避免 D 把振动放大成电机抖动

	// 构造函数
	//   p, i, d  → 三个增益
	//   windup   → 积分限幅幅值。★注意：默认值 0 会让 I 项恒为 0（详见 update() 里的说明）
	//   dAlpha   → D 项低通滤波系数，1 = 不滤波；越小越平滑但延迟越大
	//   dtMax    → 允许的最大步进间隔（秒），默认 0.1s（即间隔超过 100ms 就认为异常）
	PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1) :
		p(p), i(i), d(d), windup(windup), lpf(dAlpha), dtMax(dtMax) {}

	// ==================== 核心函数：输入误差，输出修正量 ====================
	float update(float error) {
		// t 是全局时间（定义在 CF-Drone.ino，由 time.ino 的 step() 更新）
		// prevTime 是上一次调用本函数的时刻，相减 = 本次步进的时间间隔
		float dt = t - prevTime;

		// dt 合理性检查：
		//   dt <= 0  → 首次调用，或 micros() 回绕
		//   dt >= dtMax → 主循环卡顿了（如 WiFi 阻塞）
		// 这两种情况都跳过积分/微分，只用 P 项，避免算出离谱数值让电机乱转
		if (dt > 0 && dt < dtMax) {
			integral += error * dt;                             // 积分累加：误差 × 时间
			derivative = lpf.update((error - prevError) / dt);  // 微分 = 误差变化率，再低通滤波
		} else {
			integral = 0;
			derivative = 0;
		}

		prevError = error;
		prevTime = t;

		// 最终输出 = P项 + I项(限幅后) + D项
		//
		// ★★ 重要坑点（读代码时容易困惑的地方）★★
		// constrain(x, -windup, windup) 在 windup == 0 时，区间变成 [0, 0]，
		// 无论 x 是多少都会被压成 0 —— 也就是「I 项被彻底关闭」。
		// 所以构造时若没传 windup（用默认值 0），即使 i 不为 0，I 项也完全不生效。
		// 例：control.ino 里 `PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);`
		//     没传 windup → 偏航角速率的 I 增益实际不起作用。
		return p * error + constrain(i * integral, -windup, windup) + d * derivative; // PID
	}

	// 复位：清空全部历史状态
	// 用在"切换飞行模式"、"遥控失联自动降落"等场景，
	// 防止上一次积累的积分/微分继续影响新的控制目标（见 safety.ino 的 descend()）
	void reset() {
		prevError = NAN;
		prevTime = NAN;
		integral = 0;
		derivative = 0;
		lpf.reset();
	}

private:
	float prevError = NAN;  // 上一次的误差（用来算微分）
	float prevTime = NAN;   // 上一次调用的时间戳（用来算 dt；NAN 保证首次 dt 判定失败，从而跳过积分）
};
