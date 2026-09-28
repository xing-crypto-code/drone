// control.h
//
// ==================== 这个文件是干什么的 ====================
// 头文件，本身不含实现，只做两件事：
//   1. 定义「飞行模式」枚举
//   2. 声明 control.ino 里的函数原型，供其他文件调用
//
// ★ 注意：本文件只在 WEB_RC_ENABLED 时才被 control.ino include
//   （见 control.ino 开头的 `#if WEB_RC_ENABLED`），
//   所以里面的函数声明主要是给 Web 遥控相关代码用。
//
// ==================== 控制流程速览（配合 control.ino 阅读）====================
//   control()                       每帧主入口
//     ├── interpretControls()       解析遥控器摇杆 → 飞行模式 / 解锁 / 推力 / 姿态目标
//     ├── interpretWebRC()          解析网页遥控输入（可选）
//     ├── failsafe()                故障保护（失联、低电、倒置）← 定义在 safety.ino
//     ├── controlAttitude()         【外环】角度误差 → 目标角速率
//     ├── controlRates()            【内环】角速率误差 → 目标力矩
//     └── controlTorque()           力矩 → 4 个电机推力（含混控与解饱和）

#ifndef CONTROL_H
#define CONTROL_H

// 飞行模式定义
// 数值大小有讲究：遥控器三挡开关的归一化值 0~1 会被切成三段来选模式
// （见 control.ino 的 interpretControls()：<0.25 → 第0挡，<=0.75 → 第1挡，>0.75 → 第2挡）
// 具体每挡对应哪个模式由参数 CTL_FLT_MODE_0/1/2 决定，默认三挡都是 STAB
enum FlightMode {
    MODE_MANUAL = 0,   // 手动直通（本项目实际使用的是下文的 RAW/ACRO/STAB 常量）
    MODE_ACRO = 1,     // 特技模式：摇杆直接控制角速度，松杆不自动回平，可翻跟头
    MODE_STAB = 2,     // 自稳模式：摇杆控制倾角，松杆自动回水平（新手用这个）
    MODE_ALTHOLD = 3,  // 气压计定高（油门=升降速率），本版本仅有占位实现
    MODE_AUTO = 4      // 自动模式：飞控接管（如失控自动降落），此模式下飞手摇杆失效
};

// 控制状态结构
// ★ 注意：这是一个"数据结构定义"，当前代码主要用全局变量传参（见 control.ino 里的
//   controlRoll / thrustTarget 等），这个结构体本身在项目里并没有被实际使用，
//   可以当作"设计意图的记录"来看。
struct ControlState {
    int mode;            // 当前飞行模式
    bool armed;          // 是否已解锁（电机才会转）
    float thrustTarget;  // 目标推力 0~1
    float rollTarget;    // 目标横滚角（rad）
    float pitchTarget;   // 目标俯仰角（rad）
    float yawTarget;     // 目标偏航角（rad）
};

// 函数声明（实现全在 control.ino）
void control();            // 每帧主入口，按顺序调用下面全部环节
void interpretControls();  // 解释遥控器输入：模式选择、解锁手势、推力映射、姿态目标
void interpretWebRC();     // 解释网页遥控输入（与遥控器共用 control* 变量）
void combineInputs();      // 合并多路输入源（当前版本未使用）
void controlAttitude();    // 【外环】姿态角 PID → 目标角速率
void controlRates();       // 【内环】角速率 PID → 目标力矩
void controlTorque();      // 力矩 → 四电机推力（X 型混控 + 解饱和 + 限幅）

// 辅助函数
const char* getModeName(int mode);  // 模式编号 → 可读名称，用于日志/控制台显示

// 外部函数声明（实现在 web_rc.ino，供 control.ino 调用）
bool isUsingWebRC();               // 当前是否正在用网页遥控控制（决定解锁手势是否生效）
void setWebRCWarn(const char* msg); // 往网页界面推送一条警告提示（如"电量低 禁止解锁"）

#endif // CONTROL_H
