// 轻量级旋转四元数库
// Lightweight rotation quaternion library
//
// ==================== 这个文件是干什么的 ====================
// 用「四元数」表示飞行器的三维姿态（朝向）。
//
// ==================== 为什么不用欧拉角(俯仰/横滚/偏航) ====================
// 欧拉角虽然直观，但有个致命缺陷叫「万向节死锁」：
//   飞机机头竖直向上时，俯仰=90°，此时横滚和偏航会退化成同一个轴，无法区分。
// 四元数用 4 个数 (w,x,y,z) 描述旋转，没有奇点，且插值/复合都更简单。
// 飞控内部全程用四元数，只在"要给用户看"或"要算摇杆目标角"时才转成欧拉角。
//
// ==================== 四元数速记 ====================
//   w = cos(θ/2)              θ 是旋转角
//   (x,y,z) = 旋转轴 × sin(θ/2)
//   单位四元数满足 w² + x² + y² + z² = 1
//   • 四元数相乘 = 旋转的复合（本文件重载了 operator *）
//   • 四元数逆   = 反向旋转（inversed()）
//   • 用四元数旋转向量叫「共轭」（conjugate / conjugateInversed）
//
// ==================== 本项目里谁在用 ====================
//   estimate.ino  → 陀螺仪积分、加速度计修正，都在算 attitude 这个四元数
//   control.ino   → 比较"当前姿态"与"目标姿态"，算出需要修正多少
//   imu.ino       → 用四元数把 IMU 读数从"芯片坐标系"转到"机体坐标系"
//
// ★ 重要约定：本项目用 FLU 机体坐标系
//   x=前  y=左  z=上；attitude 表示"从机体系转到世界系"的旋转

#pragma once

#include "vector.h"

class Quaternion : public Printable {
public:
	float w, x, y, z;   // w 是实部（旋转角的一半的余弦），(x,y,z) 是虚部（旋转轴方向）

	// 默认构造 = 单位四元数 = 没有任何旋转（飞机水平朝前）
	Quaternion(): w(1), x(0), y(0), z(0) {};

	Quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {};

	// 由「旋转轴 + 旋转角」构造四元数
	// 公式：w=cos(θ/2)，(x,y,z)=轴单位向量×sin(θ/2)
	// 注意用半角（halfAngle）：四元数只覆盖 720°，所以一律除以 2
	static Quaternion fromAxisAngle(const Vector& axis, float angle) {
		float halfAngle = angle * 0.5;
		float sin2 = sin(halfAngle);
		float cos2 = cos(halfAngle);
		float sinNorm = sin2 / axis.norm();  // 直接除以轴长度，顺带完成归一化
		return Quaternion(cos2, axis.x * sinNorm, axis.y * sinNorm, axis.z * sinNorm);
	}

	// ★ 本项目最常用的构造方式
	// 由「旋转向量」构造四元数：向量的方向 = 旋转轴，向量的长度 = 旋转角
	// 配合 Vector::rotationVectorBetween() 使用，是姿态修正的黄金搭档：
	//     Quaternion::fromRotationVector(errorVector)  →  一小步姿态修正量
	// 例：estimate.ino 里 `Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt))`
	static Quaternion fromRotationVector(const Vector& rotation) {
		if (rotation.zero()) {
			return Quaternion();  // 零旋转 → 单位四元数
		}
		return Quaternion::fromAxisAngle(rotation, rotation.norm());  // 长度就是角度
	}

	// 由「欧拉角」构造四元数（参数顺序：x=横滚Roll, y=俯仰Pitch, z=偏航Yaw）
	// 内部按 Z-Y-X 顺序复合（即先偏航、再俯仰、最后横滚），与 toEuler() 互为逆运算。
	// 用途：把飞手摇杆对应的期望角度转换成姿态目标。见 control.ino：
	//     attitudeTarget = Quaternion::fromEuler(Vector(controlRoll*tiltMax, controlPitch*tiltMax, yawTarget))
	static Quaternion fromEuler(const Vector& euler) {
		float cx = cos(euler.x / 2);
		float cy = cos(euler.y / 2);
		float cz = cos(euler.z / 2);
		float sx = sin(euler.x / 2);
		float sy = sin(euler.y / 2);
		float sz = sin(euler.z / 2);

		return Quaternion(
			cx * cy * cz + sx * sy * sz,
			sx * cy * cz - cx * sy * sz,
			cx * sy * cz + sx * cy * sz,
			cx * cy * sz - sx * sy * cz);
	}

	// 求「把向量 u 旋转到向量 v」的四元数
	// 目前项目里没直接用到，是备用的数学工具（Vector::rotationVectorBetween() 更常用）
	static Quaternion fromBetweenVectors(const Vector& u, const Vector& v) {
		float dot = u.x * v.x + u.y * v.y + u.z * v.z;
		float w1 = u.y * v.z - u.z * v.y;
		float w2 = u.z * v.x - u.x * v.z;
		float w3 = u.x * v.y - u.y * v.x;

		// 巧妙的构造：直接用点积和叉积拼出四元数，省去一次开方和三角函数
		Quaternion ret(
			dot + sqrt(dot * dot + w1 * w1 + w2 * w2 + w3 * w3),
			w1,
			w2,
			w3);
		ret.normalize();
		return ret;
	}

	// ---- 有效性判断（与 Vector 里那套完全一致，用 NaN 表示"无效"）----
	bool finite() const {
		return isfinite(w) && isfinite(x) && isfinite(y) && isfinite(z);
	}

	bool valid() const {
		return finite();
	}

	bool invalid() const {
		return !valid();
	}

	// 标记为无效（置 NaN）→ 让姿态相关控制环节"跳过"
	// 例：ACRO 模式下调 attitudeTarget.invalidate()，就是告诉外环"这轮别管姿态角"
	void invalidate() {
		w = NAN;
		x = NAN;
		y = NAN;
		z = NAN;
	}


	// 四元数的模（理想情况下恒为 1；因浮点累积误差会慢慢偏离）
	float norm() const {
		return sqrt(w * w + x * x + y * y + z * z);
	}

	// 归一化，修正浮点误差，防止模长漂移导致姿态慢慢"歪掉"
	void normalize() {
		float n = norm();
		w /= n;
		x /= n;
		y /= n;
		z /= n;
	}

	// 四元数 → 「旋转轴 + 旋转角」
	// 反推公式：θ = 2·acos(w)，轴 = (x,y,z)/sin(θ/2)
	// ★ 注意：当旋转角≈0 时 sin(θ/2)≈0，这里会产生除零（结果为 inf/NaN）。
	//   所以调用方一般先判断是否为单位四元数（见下面 toRotationVector 的做法）
	void toAxisAngle(Vector& axis, float& angle) const {
		angle = acos(w) * 2;
		axis.x = x / sin(angle / 2);
		axis.y = y / sin(angle / 2);
		axis.z = z / sin(angle / 2);
	}

	// 四元数 → 旋转向量（fromRotationVector 的逆运算）
	Vector toRotationVector() const {
		if (w == 1 && x == 0 && y == 0 && z == 0) return Vector(0, 0, 0); // 单位四元数 → 零旋转（避免上面的除零）
		float angle;
		Vector axis;
		toAxisAngle(axis, angle);
		return angle * axis;
	}

	// ★ 四元数 → 欧拉角（x=横滚Roll, y=俯仰Pitch, z=偏航Yaw，单位 rad）
	//
	// 这是全项目最常用的"取角度"函数：getRoll/getPitch/getYaw 都调它。
	// 但它是"重"操作（3 次三角函数的反函数），所以不要在 1kHz 主循环里乱用。
	//
	// 为什么代码这么复杂：欧拉角有万向节死锁，俯仰接近 ±90° 时横滚/偏航无法分离。
	// 这里的 sarg 就是 sin(pitch)，专门判断有没有逼近死锁区：
	//   |sarg| > 0.99999 → 进入死锁区，只能固定一组解（横滚强行置 0，偏航合并计算）
	//   否则 → 正常解算
	// 算法来源：ROS tf2 库（原始链接见下），属于业界标准实现
	Vector toEuler() const {
		// https://github.com/ros/geometry2/blob/589caf083cae9d8fae7effdb910454b4681b9ec1/tf2/include/tf2/impl/utils.h#L87
		Vector euler;
		float sqx = x * x;
		float sqy = y * y;
		float sqz = z * z;
		float sqw = w * w;
		// Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
		float sarg = -2 * (x * z - w * y) / (sqx + sqy + sqz + sqw);  // = sin(pitch)
		if (sarg <= -0.99999) {
			// 死锁区：俯仰 = -90°（机头垂直朝下）
			euler.x = 0;
			euler.y = -0.5 * PI;
			euler.z = -2 * atan2(y, x);
		} else if (sarg >= 0.99999) {
			// 死锁区：俯仰 = +90°（机头垂直朝上）
			euler.x = 0;
			euler.y = 0.5 * PI;
			euler.z = 2 * atan2(y, x);
		} else {
			// 正常情况：标准 Z-Y-X 欧拉角解算
			euler.x = atan2(2 * (y * z + w * x), sqw - sqx - sqy + sqz);
			euler.y = asin(sarg);
			euler.z = atan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz);
		}
		return euler;
	}

	// ---- 取单个姿态角（内部都转成欧拉角，开销相同）----
	// 注意：每次调用都会完整算一遍 toEuler()，要取多个角应自己存下 toEuler() 结果
	float getRoll() const {
		return toEuler().x;
	}

	float getPitch() const {
		return toEuler().y;
	}

	// 偏航角。用途很广：自稳模式下锁定机头朝向、失控降落时保持朝向（safety.ino）
	float getYaw() const {
		return toEuler().z;
	}

	// ---- 单独修改某一个姿态角，其余两个保持不变 ----
	// 实现方式：先转成欧拉角 → 改其中一个 → 再转回四元数
	// 用途：失控降落时逐帧跟随实际偏航（safety.ino 的 descend()）
	void setRoll(float roll) {
		Vector euler = toEuler();
		*this = Quaternion::fromEuler(Vector(roll, euler.y, euler.z));
	}

	void setPitch(float pitch) {
		Vector euler = toEuler();
		*this = Quaternion::fromEuler(Vector(euler.x, pitch, euler.z));
	}

	void setYaw(float yaw) {
		Vector euler = toEuler();
		*this = Quaternion::fromEuler(Vector(euler.x, euler.y, yaw));
	}

	// 四元数乘法 = 旋转的复合
	// 语义：先做 q 的旋转，再做 this 的旋转（等价于两个旋转依次叠加）
	// 用途：把"当前姿态"再叠加一个小旋转 → 得到新姿态
	//   例：estimate.ino 里 `attitude = Quaternion::rotate(attitude, 小旋转)`
	Quaternion operator * (const Quaternion& q) const {
		return Quaternion(
			w * q.w - x * q.x - y * q.y - z * q.z,
			w * q.x + x * q.w + y * q.z - z * q.y,
			w * q.y + y * q.w + z * q.x - x * q.z,
			w * q.z + z * q.w + x * q.y - y * q.x);
	}

	bool operator == (const Quaternion& q) const {
		return w == q.w && x == q.x && y == q.y && z == q.z;
	}

	bool operator != (const Quaternion& q) const {
		return !(*this == q);
	}

	// 四元数的逆 = 反向旋转（把飞机转回原来的朝向）
	// 单位四元数的逆就是共轭：实部不变、虚部取反。
	// 这里除以模长平方，是为了兼容"模长不恰好为 1"的情况
	Quaternion inversed() const {
		float normSqInv = 1 / (w * w + x * x + y * y + z * z);
		return Quaternion(
			w * normSqInv,
			-x * normSqInv,
			-y * normSqInv,
			-z * normSqInv);
	}

	// 用四元数旋转一个向量：结果为 this * v * this⁻¹
	// 直观理解：把 v 看作"某个方向"，按 this 姿态做一次旋转
	// ★ 注意：本项目实际调用的都是下面的 rotateVector()（旋转方向与这个**相反**），
	//   本函数未在项目中直接使用，保留作为数学工具。
	Vector conjugate(const Vector& v) const {
		Quaternion qv(0, v.x, v.y, v.z);
		Quaternion res = (*this) * qv * inversed();
		return Vector(res.x, res.y, res.z);
	}

	// 与 conjugate 方向相反的旋转：结果为 this⁻¹ * v * this
	// ★ 这才是本项目真正在用的那一个（由下面的 rotateVector() 间接调用）
	//   用在：imu.ino 转换 IMU 安装朝向、estimate.ino 做重力修正、control.ino 算姿态误差
	Vector conjugateInversed(const Vector& v) const {
		Quaternion qv(0, v.x, v.y, v.z);
		Quaternion res = inversed() * qv * (*this);
		return Vector(res.x, res.y, res.z);
	}

	// ★★ 另一个高频使用的工具：四元数 a 再叠加旋转 b，返回新四元数 ★★
	// normalize 默认 true —— 每次复合都归一化，防止浮点误差累积导致姿态慢慢失准
	// 用途：陀螺仪积分推进姿态（estimate.ino 的 applyGyro）
	static Quaternion rotate(const Quaternion& a, const Quaternion& b, const bool normalize = true) {
		Quaternion rotated = a * b;
		if (normalize) {
			rotated.normalize();
		}
		return rotated;
	}

	// ★★ 用四元数旋转向量（项目中到处在用）★★
	// 理解要点：q 是"飞机的姿态"，v 通常是"某个方向向量"，
	// 返回 v 在该姿态下的方向。
	// 典型用法（control.ino 姿态环）：
	//     Vector upActual = Quaternion::rotateVector(Vector(0,0,1), attitude);       // 当前"上方向"
	//     Vector upTarget = Quaternion::rotateVector(Vector(0,0,1), attitudeTarget); // 目标"上方向"
	//     然后求两者之间的旋转向量 → 就是姿态误差
	static Vector rotateVector(const Vector& v, const Quaternion& q) {
		return q.conjugateInversed(v);
	}

	// 求 a 相对于 b 的增量旋转（数学上 = a × b⁻¹）
	// 若 a 与 b 相同，结果是单位四元数。本项目未直接调用，属备用数学工具
	static Quaternion between(const Quaternion& a, const Quaternion& b, const bool normalize = true) {
		Quaternion q = a * b.inversed();
		if (normalize) {
			q.normalize();
		}
		return q;
	}

	// 供 Serial.print(quat) 使用，同样打印 15 位小数
	size_t printTo(Print& p) const {
		size_t r = 0;
		r += p.print(w, 15) + p.print(" ");
		r += p.print(x, 15) + p.print(" ");
		r += p.print(y, 15) + p.print(" ");
		r += p.print(z, 15);
		return r;
	}
};
