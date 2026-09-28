// 基于陀螺仪和加速度计的姿态计算
// Attitude estimation from gyro and accelerometer

#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

float accWeight = 0.003;

// ============== 水平修正 P 项 ==============
float levelWeight = 0;  // 水平修正 P 项权重（关闭，无法区分陀螺温漂与机械不对称时会起负作用）
float levelMaxTilt = radians(30); // rad, level correction fades out at this tilt angle (matches TILT_MAX)
// 摇杆感知门控阈值
// 当飞手摇杆偏转量（横滚或俯仰取最大值，范围 0~1）超过此阈值时，
// applyLevel 的修正权重线性衰减至 0，避免与 PID 积分项产生耦合导致松杆后漂移。
// 调整方法：
//   - 值越小，门控越灵敏（轻微打杆即停止修正），悬停抑漂效果减弱。
//   - 值越大，打杆时 applyLevel 干扰更久，现象三改善减弱。
//   - 推荐范围：0.2（灵敏）~ 0.4（宽松），默认 0.2（摇杆 20% 行程时权重归零）。
//   - 可通过 CLI 在线修改：p EST_LVL_GATE_THR <值>
float levelGateThreshold = 0.2f; // 摇杆门控阈值，参数名：EST_LVL_GATE_THR

// ============== Mahony 风格水平修正 I 项 ==============
// levelBiasGain：重力误差向虚拟陀螺偏置的积分增益（I 项）。
// 积分量在 applyGyro() 中从陀螺读数中减去，不直接修改 attitude 四元数，
// 因此 PID 控制回路感知不到修正过程，消除了与内环 I 项的耦合振荡。
// 调整方法：越大收敛越快，但可能过积分；约 30s 收敛至稳态。
// 参数名：EST_LVL_BIAS_GAIN
float levelBiasGain = 0;  // Mahony I 项增益（关闭，无法区分陀螺温漂与机械不对称时会起负作用）
Vector levelGyroBias(0, 0, 0); // 由 applyLevel() 估计的虚拟陀螺偏置（rad/s）

extern float controlRoll, controlPitch; // 飞手摇杆输入，定义于 CF-Drone.ino
LowPassFilter<Vector> ratesFilter(0.2); // cutoff frequency ~ 40 Hz

void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
}

void applyGyro() {
	// Mahony 风格水平修正 I 项 从陀螺读数中减去 Mahony I 项估计的虚拟偏置，再滤波积分
	// 这样水平修正的长期影响通过偏置路径而非 attitude 直接反映，PID 不感知
	rates = ratesFilter.update(gyro - levelGyroBias);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// test should we apply accelerometer gravity correction
	float accNorm = acc.norm();
	landed = !motorsActive() && abs(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// calculate accelerometer correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

// 水平修正（Mahony 风格：P 项直接修正 attitude，I 项积分进虚拟陀螺偏置）
// 架构说明：
//   - P 项（levelWeight）：对 attitude 施加小修正，提供快速响应，量级已减半以降低踢扰。
//   - I 项（levelBiasGain）：将重力参考误差积分到 levelGyroBias，
//     在 applyGyro() 中从陀螺读数减去。修正路径绕开 PID 误差输入，
//     消除原来直接改 attitude 导致的内环 I 项耦合振荡。
void applyLevel() {
	if (landed) {
		levelGyroBias = Vector(0, 0, 0); // 落地后清零偏置，下次起飞重新学习
		return;
	}

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acos(constrain(up.z, -1.0f, 1.0f));

	// 极近水平时跳过：tilt→0 时叉积方向对振动噪声极敏感，方向随机会引入无规律漂移
	if (tilt < radians(0.1f)) return;

	// P 项权重：倾角越接近 levelMaxTilt，权重越小
	float dynamicWeight = levelWeight * constrain(1.0f - tilt / levelMaxTilt, 0.0f, 1.0f);

	// ---- 摇杆感知门控 ----
	float stickDeflection = max(abs(controlRoll), abs(controlPitch));
	float stickGate = constrain(1.0f - stickDeflection / levelGateThreshold, 0.0f, 1.0f);
	dynamicWeight *= stickGate;
	// ------------------------------------

	// 重力参考误差：机体 Z 轴与世界 Z 轴之间的旋转向量
	Vector error = Vector::rotationVectorBetween(Vector(0, 0, 1), up);

	// P 项：直接修正 attitude（量级极小，保留快速漂移抑制响应）
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(error * dynamicWeight));

	// I 项：积分进虚拟陀螺偏置（Mahony 风格）
	// 打杆期间（stickGate=0）暂停积分，避免积分方向因主动操纵而错误累积
	levelGyroBias += error * (levelBiasGain * stickGate);

	// 限幅：等效最大补偿 3 deg/s，防止偏置发散
	float biasNorm = levelGyroBias.norm();
	const float biasLimit = radians(3.0f);
	if (biasNorm > biasLimit) levelGyroBias = levelGyroBias * (biasLimit / biasNorm);
}
