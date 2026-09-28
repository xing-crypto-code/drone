// 轻量级矢量库
// Lightweight vector library
//
// ==================== 这个文件是干什么的 ====================
// 一个三维向量类（x, y, z），是整个飞控的数学基础。
// 本项目里所有"三轴量"都用它表示：
//   gyro   陀螺仪角速度 (rad/s)     acc    加速度 (m/s²)
//   rates  滤波后的角速度           torqueTarget 目标力矩
//   attitudeTarget 的姿态误差 等
//
// ★ 关键概念：本项目使用「NED/FLU 机体坐标系」
//   x = 前(Forward)   y = 左(Left)   z = 上(Up)     —— 即 FLU
//   正号约定：绕 x 正转 = 右滚(Roll)，绕 y 正转 = 抬头(Pitch)，绕 z 正转 = 左偏航(Yaw)
//   所以代码里常看到 `-controlYaw`——因为摇杆右推要让飞机顺时针转，与 z 正向相反。
//
// ★ 另一个关键设计：用 NaN 表示"这个量当前无效"
//   invalidate() 把三个分量都置成 NaN，配合 valid()/invalid() 判断，
//   用来"禁用某一级控制"（例如 ACRO 模式跳过姿态外环，就调 attitudeTarget.invalidate()）。
//   NaN 有个好处：任何与 NaN 的运算结果还是 NaN，不会悄悄算出错误数值。

#pragma once

// 继承 Printable 是为了能用 Serial.print(vec) 直接打印（见文件末尾的 printTo）
class Vector : public Printable {
public:
	float x, y, z;

	Vector(): x(0), y(0), z(0) {};

	Vector(float x, float y, float z): x(x), y(y), z(z) {};

	// 是否是零向量（三个分量都恰好为 0，浮点精确比较）
	bool zero() const {
		return x == 0 && y == 0 && z == 0;
	}

	// 三个分量是否都是有限值（排除 NaN / Inf）
	bool finite() const {
		return isfinite(x) && isfinite(y) && isfinite(z);
	}

	// 数据是否有效（本项目中"有效"就等于"有限"）
	bool valid() const {
		return finite();
	}

	// 数据是否无效（含 NaN，通常意味着"该控制环节被主动禁用"）
	bool invalid() const {
		return !valid();
	}

	// 主动标记为无效：把三个分量置为 NaN
	// 用法：想让某一级控制"跳过"，就 invalidate() 对应的目标量
	void invalidate() {
		x = NAN;
		y = NAN;
		z = NAN;
	}


	// 向量的模（长度）= √(x²+y²+z²)
	// 例：加速度计读数取 norm() 后≈9.8，用来判断"是否静止"（见 estimate.ino）
	float norm() const {
		return sqrt(x * x + y * y + z * z);
	}

	// 就地归一化：把向量变成单位向量（长度 = 1），方向不变
	void normalize() {
		float n = norm();
		x /= n;
		y /= n;
		z /= n;
	}

	// ---------- 运算符重载 ----------
	// 注意 ★ 这里有两种"乘法"，读代码时最容易搞混，务必区分：

	// 向量 + 标量：三个分量各加同一个数
	Vector operator + (const float b) const {
		return Vector(x + b, y + b, z + b);
	}

	// 向量 × 标量：整体缩放（最常用，如 `rates * dt`）
	Vector operator * (const float b) const {
		return Vector(x * b, y * b, z * b);
	}

	// 向量 ÷ 标量
	Vector operator / (const float b) const {
		return Vector(x / b, y / b, z / b);
	}

	// 向量 + 向量：逐分量相加
	Vector operator + (const Vector& b) const {
		return Vector(x + b.x, y + b.y, z + b.z);
	}

	// 向量 - 向量：逐分量相减（算误差最常用，如 `ratesTarget - rates`）
	Vector operator - (const Vector& b) const {
		return Vector(x - b.x, y - b.y, z - b.z);
	}

	Vector& operator += (const Vector& b) {
		return *this = *this + b;
	}

	Vector& operator -= (const Vector& b) {
		return *this = *this - b;
	}

	// ★ 向量 × 向量 = 逐分量相乘（不是点积、也不是叉积！）
	// Element-wise multiplication
	// 例：accScale 用三个轴的缩放系数去修正加速度计读数
	Vector operator * (const Vector& b) const {
		return Vector(x * b.x, y * b.y, z * b.z);
	}

	// 向量 ÷ 向量：逐分量相除（如 `(acc - accBias) / accScale`）
	// Element-wise division
	Vector operator / (const Vector& b) const {
		return Vector(x / b.x, y / b.y, z / b.z);
	}

	bool operator == (const Vector& b) const {
		return x == b.x && y == b.y && z == b.z;
	}

	bool operator != (const Vector& b) const {
		return !(*this == b);
	}

	// ---------- 静态数学工具 ----------

	// 点积（内积）：a·b = |a||b|cosθ，结果是标量
	// 几何意义：反映两个向量"方向有多一致"（同向为正、垂直为 0、反向为负）
	static float dot(const Vector& a, const Vector& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	// 叉积（外积）：结果是垂直于 a、b 的新向量，方向用右手定则判定
	// 几何意义：轴线的方向就靠它算出来（姿态修正的核心）
	static Vector cross(const Vector& a, const Vector& b) {
		return Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	// 两向量夹角（弧度），范围 [0, π]
	// constrain 把点积除以模长的结果夹到 [-1,1]：
	//   因为浮点误差可能算出 1.0000001，直接传给 acos() 会返回 NaN
	static float angleBetween(const Vector& a, const Vector& b) {
		return acos(constrain(dot(a, b) / (a.norm() * b.norm()), -1, 1));
	}

	// ★★ 本项目最重要的数学工具 ★★
	// 求「把向量 a 旋转到向量 b」所需的**旋转向量**（方向=旋转轴，长度=旋转角）
	// 用途：姿态误差修正
	//   - estimate.ino：把加速度计测到的"上方向"与姿态估计的"上方向"对齐
	//   - control.ino ：把当前姿态的"上方向"转到目标姿态的"上方向"
	// 返回值直接喂给 Quaternion::fromRotationVector() 就能得到四元数增量，非常方便。
	static Vector rotationVectorBetween(const Vector& a, const Vector& b) {
		float an = a.norm();
		float bn = b.norm();
		// 任一向量长度≈0（无意义），直接返回零旋转
		if (an < 1e-6 || bn < 1e-6) {
			return Vector(0, 0, 0);
		}
		Vector direction = cross(a, b);
		if (direction.norm() < 1e-6) { // 叉积≈0 → 两向量平行（共线），需要特殊处理
			if (dot(a, b) > 0) { // 同向：压根不用转
				return Vector(0, 0, 0);
			}
			// 反向（差 180°）：此时旋转轴不唯一，随便取一个与 a 垂直的方向即可
			Vector perp = cross(a, Vector(1, 0, 0));
			if (perp.norm() < 1e-6) { // a 恰好与 x 轴共线时换一个参考轴
				perp = cross(a, Vector(0, 1, 0));
			}
			perp.normalize();
			return perp * PI; // 转 180° = π
		}
		// 一般情况：轴为叉积方向（归一化），角度为两向量夹角
		direction.normalize();
		float angle = angleBetween(a, b);
		return direction * angle;
	}

	// 供 Serial.print(vec) 使用（继承 Printable 的要求）
	// 15 表示打印 15 位小数，方便调试姿态/角速度这类小数值
	size_t printTo(Print& p) const {
		return
			p.print(x, 15) + p.print(" ") +
			p.print(y, 15) + p.print(" ") +
			p.print(z, 15);
	}
};

// 支持「标量在左」的写法：2.0 * vec（否则只有 vec * 2.0 能用）
Vector operator * (const float a, const Vector& b) { return b * a; }
Vector operator + (const float a, const Vector& b) { return b + a; }
