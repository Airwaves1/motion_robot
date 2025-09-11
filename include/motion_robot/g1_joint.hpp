#pragma once

#include <vector>
#include <array>

constexpr int G1_NUM_MOTOR = 29;

constexpr int LEFT_HIP_PITCH = 0;
constexpr int LEFT_HIP_ROLL = 1;
constexpr int LEFT_HIP_YAW = 2;
constexpr int LEFT_KNEE = 3;
constexpr int LEFT_ANKLE_PITCH = 4;
constexpr int LEFT_ANKLE_ROLL = 5;

constexpr int RIGHT_HIP_PITCH = 6;
constexpr int RIGHT_HIP_ROLL = 7;
constexpr int RIGHT_HIP_YAW = 8;
constexpr int RIGHT_KNEE = 9;
constexpr int RIGHT_ANKLE_PITCH = 10;
constexpr int RIGHT_ANKLE_ROLL = 11;

constexpr int WAIST_YAW = 12;
constexpr int WAIST_ROLL = 13;
constexpr int WAIST_PITCH = 14;

constexpr int LEFT_SHOULDER_PITCH = 15;
constexpr int LEFT_SHOULDER_ROLL = 16;
constexpr int LEFT_SHOULDER_YAW = 17;
constexpr int LEFT_ELBOW = 18;
constexpr int LEFT_WRIST_ROLL = 19;
constexpr int LEFT_WRIST_PITCH = 20;
constexpr int LEFT_WRIST_YAW = 21;

constexpr int RIGHT_SHOULDER_PITCH = 22;
constexpr int RIGHT_SHOULDER_ROLL = 23;
constexpr int RIGHT_SHOULDER_YAW = 24;          
constexpr int RIGHT_ELBOW = 25;
constexpr int RIGHT_WRIST_ROLL = 26;
constexpr int RIGHT_WRIST_PITCH = 27;
constexpr int RIGHT_WRIST_YAW = 28;

// 关节限位（弧度），按关节索引顺序排列 [min, max]
static const std::vector<std::array<float, 2>> G1_JOINT_LIMIT = {
    {-2.5307f,  2.8798f},      // 0  LEFT_HIP_PITCH
    {-0.5236f,  2.9671f},      // 1  LEFT_HIP_ROLL
    {-2.7576f,  2.7576f},      // 2  LEFT_HIP_YAW
    {-0.087267f,2.8798f},      // 3  LEFT_KNEE
    {-0.87267f, 0.5236f},      // 4  LEFT_ANKLE_PITCH
    {-0.2618f,  0.2618f},      // 5  LEFT_ANKLE_ROLL
    {-2.5307f,  2.8798f},      // 6  RIGHT_HIP_PITCH
    {-2.9671f,  0.5236f},      // 7  RIGHT_HIP_ROLL
    {-2.7576f,  2.7576f},      // 8  RIGHT_HIP_YAW
    {-0.087267f,2.8798f},      // 9  RIGHT_KNEE
    {-0.87267f, 0.5236f},      // 10 RIGHT_ANKLE_PITCH
    {-0.2618f,  0.2618f},      // 11 RIGHT_ANKLE_ROLL
    {-2.618f,   2.618f},       // 12 WAIST_YAW
    {-0.52f,    0.52f},        // 13 WAIST_ROLL
    {-0.52f,    0.52f},        // 14 WAIST_PITCH
    {-3.0892f,  2.6704f},      // 15 LEFT_SHOULDER_PITCH
    {-1.5882f,  2.2515f},      // 16 LEFT_SHOULDER_ROLL
    {-2.618f,   2.618f},       // 17 LEFT_SHOULDER_YAW
    {-1.0472f,  2.0944f},      // 18 LEFT_ELBOW
    {-1.972222054f, 1.972222054f}, // 19 LEFT_WRIST_ROLL
    {-1.614429558f, 1.614429558f}, // 20 LEFT_WRIST_PITCH
    {-1.614429558f, 1.614429558f}, // 21 LEFT_WRIST_YAW
    {-3.0892f,  2.6704f},      // 22 RIGHT_SHOULDER_PITCH
    {-2.2515f,  1.5882f},      // 23 RIGHT_SHOULDER_ROLL
    {-2.618f,   2.618f},       // 24 RIGHT_SHOULDER_YAW
    {-1.0472f,  2.0944f},      // 25 RIGHT_ELBOW
    {-1.972222054f, 1.972222054f}, // 26 RIGHT_WRIST_ROLL
    {-1.614429558f, 1.614429558f}, // 27 RIGHT_WRIST_PITCH
    {-1.614429558f, 1.614429558f}  // 28 RIGHT_WRIST_YAW
};