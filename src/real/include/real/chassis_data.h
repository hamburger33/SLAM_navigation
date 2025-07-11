#ifndef REAL_CHASSIS_DATA_H
#define REAL_CHASSIS_DATA_H

#include <cstdint>

namespace real {

// 底盘状态枚举
enum class ChassisStatus : uint8_t {
    IDLE = 0,       // 空闲
    STOPPED = 1,    // 停止
    MOVING = 2,     // 运动中
    BRAKE = 3,      // 刹车
    ERROR = 4       // 错误状态
};

// CAN数据更新结构体
struct Update_chassis {
    ChassisStatus status;    // 底盘状态枚举
    float linear_velocity;   // 前进线速度 (m/s)
    float angular_velocity;  // 角速度 (rad/s)
};

// 底盘编码器数据结构体（用于里程计计算）
struct ChassisEncoderData {
    int32_t left_encoder;    // 左轮编码器计数
    int32_t right_encoder;   // 右轮编码器计数
    uint64_t timestamp_us;   // 时间戳 (微秒)
};

// 底盘参数配置
struct ChassisParams {
    float wheel_base;        // 轮距 (m)
    float wheel_radius;      // 轮半径 (m)
    int encoder_resolution;  // 编码器分辨率 (脉冲/圈)
};

} // namespace real

#endif // REAL_CHASSIS_DATA_H
