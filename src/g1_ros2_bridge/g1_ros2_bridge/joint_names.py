"""G1 29-DOF joint index → URDF joint name mapping (matches g1_29dof URDF)."""


class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


JOINT_INDEX_TO_NAME = {
    G1JointIndex.LeftHipPitch:       "left_hip_pitch_joint",
    G1JointIndex.LeftHipRoll:        "left_hip_roll_joint",
    G1JointIndex.LeftHipYaw:         "left_hip_yaw_joint",
    G1JointIndex.LeftKnee:           "left_knee_joint",
    G1JointIndex.LeftAnklePitch:     "left_ankle_pitch_joint",
    G1JointIndex.LeftAnkleRoll:      "left_ankle_roll_joint",
    G1JointIndex.RightHipPitch:      "right_hip_pitch_joint",
    G1JointIndex.RightHipRoll:       "right_hip_roll_joint",
    G1JointIndex.RightHipYaw:        "right_hip_yaw_joint",
    G1JointIndex.RightKnee:          "right_knee_joint",
    G1JointIndex.RightAnklePitch:    "right_ankle_pitch_joint",
    G1JointIndex.RightAnkleRoll:     "right_ankle_roll_joint",
    G1JointIndex.WaistYaw:           "waist_yaw_joint",
    G1JointIndex.WaistRoll:          "waist_roll_joint",
    G1JointIndex.WaistPitch:         "waist_pitch_joint",
    G1JointIndex.LeftShoulderPitch:  "left_shoulder_pitch_joint",
    G1JointIndex.LeftShoulderRoll:   "left_shoulder_roll_joint",
    G1JointIndex.LeftShoulderYaw:    "left_shoulder_yaw_joint",
    G1JointIndex.LeftElbow:          "left_elbow_joint",
    G1JointIndex.LeftWristRoll:      "left_wrist_roll_joint",
    G1JointIndex.LeftWristPitch:     "left_wrist_pitch_joint",
    G1JointIndex.LeftWristYaw:       "left_wrist_yaw_joint",
    G1JointIndex.RightShoulderPitch: "right_shoulder_pitch_joint",
    G1JointIndex.RightShoulderRoll:  "right_shoulder_roll_joint",
    G1JointIndex.RightShoulderYaw:   "right_shoulder_yaw_joint",
    G1JointIndex.RightElbow:         "right_elbow_joint",
    G1JointIndex.RightWristRoll:     "right_wrist_roll_joint",
    G1JointIndex.RightWristPitch:    "right_wrist_pitch_joint",
    G1JointIndex.RightWristYaw:      "right_wrist_yaw_joint",
}

JOINT_INDICES = sorted(JOINT_INDEX_TO_NAME.keys())
JOINT_NAMES = [JOINT_INDEX_TO_NAME[i] for i in JOINT_INDICES]
