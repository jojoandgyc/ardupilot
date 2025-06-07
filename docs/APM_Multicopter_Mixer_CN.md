# APM 多旋翼混控器方案及运行原理

本文档基于 `AP_MotorsMatrix` 源码，对 ArduPilot (APM) 在多旋翼无人机上的混控器方案进行简要分析，解释其核心原理与实现方式。

## 1. 混控器的作用

混控器根据飞控控制器输出的滚转（Roll）、俯仰（Pitch）、偏航（Yaw）以及油门（Throttle）指令，计算各个电机应输出的推力比例。APM 在多旋翼上采用 `AP_MotorsMatrix` 类实现矩阵混控，通过预先设定的各个电机对姿态和油门的影响系数，将姿态指令按比例分配到各电机。

## 2. 电机因子与归一化

`AP_MotorsMatrix` 在初始化时为每个电机配置 **滚转因子**、**俯仰因子**、**偏航因子**和 **油门因子**。这些因子在 `normalise_rpy_factors()` 中被归一化到 `±0.5` 范围，以保证所有电机的姿态响应均匀。

```cpp
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;
    float throttle_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            roll_fac = MAX(roll_fac,fabsf(_roll_factor[i]));
            pitch_fac = MAX(pitch_fac,fabsf(_pitch_factor[i]));
            yaw_fac = MAX(yaw_fac,fabsf(_yaw_factor[i]));
            throttle_fac = MAX(throttle_fac,MAX(0.0f,_throttle_factor[i]));
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
            }
            if (!is_zero(throttle_fac)) {
                _throttle_factor[i] = MAX(0.0f,_throttle_factor[i] / throttle_fac);
            }
        }
    }
}
```
【F:libraries/AP_Motors/AP_MotorsMatrix.cpp†L1318-L1367】

## 3. 电机配置

每种机型在 `setup_quad_matrix()` 等函数中定义电机的布置与旋转方向，以 `MotorDef` 结构体保存角度和偏航方向。例如四旋翼 X 架构的定义如下：

```cpp
static const AP_MotorsMatrix::MotorDef motors[] {
    {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
    { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
    {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
    {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
};
add_motors(motors, ARRAY_SIZE(motors));
```
【F:libraries/AP_Motors/AP_MotorsMatrix.cpp†L573-L592】

`AP_MotorsMatrix` 还提供 `add_motor_raw()` 用于设置不对称机型时的滚转、俯仰因子：

```cpp
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees,
float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}
```
【F:libraries/AP_Motors/AP_MotorsMatrix.cpp†L530-L549】

## 4. 姿态指令混合过程

在 `output_armed_stabilizing()` 中，混控器首先根据电压补偿系数计算滚转、俯仰、偏航和油门的推力输入，然后为每个电机叠加各轴的贡献，计算出 `thrust_rpyt_out` 数组。

```cpp
const float compensation_gain = thr_lin.get_compensation_gain();
const float roll_thrust  = (_roll_in + _roll_in_ff) * compensation_gain;
const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
float throttle_thrust = get_throttle() * compensation_gain;
```
【F:libraries/AP_Motors/AP_MotorsMatrix.cpp†L218-L231】

随后根据当前油门和姿态指令计算可用的偏航空间，保证在姿态控制与油门输出间取得平衡：

```cpp
float yaw_allowed = 1.0f; // amount of yaw we can fit in
for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
    if (motor_enabled[i]) {
        _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
        if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index)) {
            const float thrust_rp_best_throttle = throttle_thrust_best_rpy + _thrust_rpyt_out[i];
            float motor_room = is_positive(yaw_thrust * _yaw_factor[i]) ?
                                1.0 - thrust_rp_best_throttle : thrust_rp_best_throttle;
            const float motor_yaw_allowed = MAX(motor_room, 0.0)/fabsf(_yaw_factor[i]);
            yaw_allowed = MIN(yaw_allowed, motor_yaw_allowed);
        }
    }
}
```
【F:libraries/AP_Motors/AP_MotorsMatrix.cpp†L296-L330】

最后将偏航、滚转和俯仰的合成值与油门进行缩放，得到各电机最终的推力比例：

```cpp
for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
    if (motor_enabled[i]) {
        _thrust_rpyt_out[i] = (throttle_thrust_best_plus_adj * _throttle_factor[i]) +
                              (rpy_scale * _thrust_rpyt_out[i]);
    }
}
```
【F:libraries/AP_Motors/AP_MotorsMatrix.cpp†L364-L369】

此时 `thrust_rpyt_out` 数组的值将转换为 PWM 信号发送到各电调，实现姿态与油门的综合控制。

## 5. 总结

APM 的多旋翼混控器通过预设的电机矩阵配置，将控制器输出的姿态和油门指令转换为各电机的推力值。其核心流程包括：

1. 依据机架类型设置各电机对滚转、俯仰、偏航的影响因子。
2. 对因子进行归一化处理，保证姿态控制不受机型差异影响。
3. 在实时循环中根据电压补偿及油门限制，计算姿态指令可用范围。
4. 将各轴指令按比例叠加到各电机上，得到最终输出。

通过这些步骤，APM 能在不同多旋翼机型上实现稳定的飞行控制。
