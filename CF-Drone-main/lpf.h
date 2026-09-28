// 低通滤波器实现
// Low pass filter implementation
//
// ==================== 这个文件是干什么的 ====================
// 把跳动剧烈的信号"抹平"，滤掉高频噪声。
// 本项目哪里用到它：
//   • IMU 的陀螺仪读数  → estimate.ino 里的 ratesFilter（40Hz 低通）
//   • IMU 的陀螺仪零偏  → imu.ino 里的 gyroBiasFilter（很慢的滤波，用于估计零漂）
//   • PID 的 D 项        → pid.h 里每个 PID 对象自带的 lpf（D 项最怕噪声）
//
// ==================== 代价：延迟 ====================
// 滤波越强 → 越平滑，但信号"滞后"越严重。
// 飞控里延迟 = 飞行品质变差，所以不能无脑调狠，要权衡。
//
// ==================== 参数 alpha 怎么理解 ====================
//   alpha = 1    → 不滤波，原样输出（相当于关闭）
//   alpha 越小   → 越平滑，延迟越大
//   也可以用 setCutOffFrequency() 按"截止频率"来算 alpha，比手调 alpha 直观得多。
//
// ==================== 公式原理 ====================
//   output += alpha * (input - output)
// 意思是：每次只朝新采样值"挪一小步"，alpha 决定这一步迈多大。
// 抛开形式，它就是经典的一阶 IIR 低通 / 指数移动平均（EMA）。
//
// ==================== 为什么写成模板 template ====================
// 同一套逻辑要同时服务两种数据类型：
//   T = float   → 滤单个数值（如 PID 的 D 项）
//   T = Vector  → 滤三维向量（如陀螺仪三轴读数）
// 所以用模板泛型实现。

#pragma once

template <typename T> // Using template to make the filter usable for scalar and vector values
class LowPassFilter {
public:
	float alpha; // 平滑系数（0~1），1 表示不滤波
	T output;    // 当前滤波输出值

	LowPassFilter(float alpha): alpha(alpha) {};

	// 喂入一个新采样值，返回滤波后的结果
	T update(const T input) {
		if (alpha == 1) { // alpha=1 → 不滤波，直接透传
			return input;
		}

		// 首次调用时没有历史值，直接用输入当初始输出。
		// 否则输出会从 0 开始慢慢爬向真值，产生一段明显的错误初值（像"上电要等几秒才准"）
		if (!initialized) {
			output = input;
			initialized = true;
		}

		// 核心：朝输入值挪一小步，alpha 决定步长
		return output += alpha * (input - output);
	}

	// 按「截止频率 + 采样周期」反算 alpha。
	// 例：setCutOffFrequency(40, 0.001) → 只保留 40Hz 以下的信号（1kHz 主循环下）
	// 推导：一阶低通 alpha = 1 - e^(-2π·fc·dt)
	void setCutOffFrequency(float cutOffFreq, float dt) {
		alpha = 1 - exp(-2 * PI * cutOffFreq * dt);
	}

	// 复位：丢掉历史值，下次 update() 会重新用输入初始化
	// （PID::reset() 里会调用它，切模式/降落时清干净）
	void reset() {
		initialized = false;
	}

private:
	bool initialized = false;  // 是否已经有过历史输出值
};
