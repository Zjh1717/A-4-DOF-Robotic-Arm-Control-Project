#ifndef ARM_DRIVER_SERVO_PROTOCOL_H
#define ARM_DRIVER_SERVO_PROTOCOL_H

#include <array>
#include <cmath>
#include <string>

namespace arm_driver {

constexpr double kPi = 3.14159265358979323846;

// 运动学关节数量（底座/肩/肘/腕）；夹爪不参与运动学。
constexpr int kKinematicJoints = 4;
// 串口协议里的关节总数（4 个运动学关节 + 1 个夹爪）。
constexpr int kTotalJoints = 5;
// 夹爪在串口协议里的关节 id。
constexpr int kGripperJointId = 4;

// ============================================================================
// 单位 / 零位换算（集中处理，唯一权威位置）
//
// 依据（以实际文件为准，勿臆测）：
//  - 固件（下位机）关节角单位是【度】；URDF / 控制器内部使用【弧度】。
//  - 固件零位：所有关节为 0 时机械臂竖直向上（inv_geo.py 注释）。
//  - URDF 零位：joint2 原点 rpy=0，即连杆水平向前（arm_4dof.urdf 注释）。
//  - 平面关节旋转方向：固件“逆时针为正”（forward_3dof 中 x = -Σ L·sinθ，
//    等价于绕 -Y 轴），而 URDF 平面关节轴为 +Y（绕 +Y 为正）。因此肩/肘/腕
//    三个平面关节在换算时需要取反号。
//  - 底座旋转：固件与 URDF 均绕 +Z，方向一致，不取反。
//
// 换算公式（度）：
//   q0_urdf =  q0_code
//   q1_urdf = -q1_code + joint2_offset_deg   （joint2_offset_deg = -90）
//   q2_urdf = -q2_code
//   q3_urdf = -q3_code
// 反变换对称。
//
// 夹爪（id4）不经过本模块，按固件原单位（0~30 等）透传。
// ============================================================================

// 肩关节（SDK id1 / URDF joint2）的固件零位 -> URDF 零位偏移（度）。
// 固件零位=竖直向上、URDF 零位=水平向前，故 code->urdf 偏移为 -90°。
// 已由 forward_3dof 与 URDF 正解数值比对确认；若真机姿态相反，改此默认值。
constexpr double kJoint2CodeToUrdfOffsetDeg = -90.0;

inline double degToRad(double deg) { return deg * kPi / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / kPi; }

// 固件角度（度，前 4 个运动学关节）-> URDF 角度（弧度）。
inline void codeKinematicToUrdf(const std::array<double, kKinematicJoints>& code_deg,
                                std::array<double, kKinematicJoints>& urdf_rad,
                                double joint2_offset_deg = kJoint2CodeToUrdfOffsetDeg) {
    urdf_rad[0] = degToRad(code_deg[0]);                            // 底座：不变号
    urdf_rad[1] = degToRad(-code_deg[1] + joint2_offset_deg);       // 肩：取反 + 零位偏移
    urdf_rad[2] = degToRad(-code_deg[2]);                           // 肘：取反
    urdf_rad[3] = degToRad(-code_deg[3]);                           // 腕：取反
}

// URDF 角度（弧度，前 4 个运动学关节）-> 固件角度（度）。
inline void urdfKinematicToCode(const std::array<double, kKinematicJoints>& urdf_rad,
                                std::array<double, kKinematicJoints>& code_deg,
                                double joint2_offset_deg = kJoint2CodeToUrdfOffsetDeg) {
    code_deg[0] = radToDeg(urdf_rad[0]);                            // 底座：不变号
    code_deg[1] = -radToDeg(urdf_rad[1]) + joint2_offset_deg;       // 肩：取反 + 零位偏移
    code_deg[2] = -radToDeg(urdf_rad[2]);                           // 肘：取反
    code_deg[3] = -radToDeg(urdf_rad[3]);                           // 腕：取反
}

// ============================================================================
// MicroPython REPL 文本命令（依据 python_sdk.py 的 class Arm）
// ============================================================================
inline std::string makeGetJointsCommand() {
    return "print(arm.joints[0].lastVal,arm.joints[1].lastVal,"
           "arm.joints[2].lastVal,arm.joints[3].lastVal,arm.joints[4].lastVal)";
}

inline std::string makeLockCommand() { return "arm.bundle.lock()"; }
inline std::string makeFreeCommand() { return "arm.bundle.free()"; }

}  // namespace arm_driver

#endif  // ARM_DRIVER_SERVO_PROTOCOL_H
