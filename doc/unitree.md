关节电机
G1 关节采用了 Unitree 自研电机，具备出色的性能和特点。电机的最大扭矩为 120N.m，采用了中空轴线的设计，使得电机在结构上更加轻量化、紧凑化。电机还配备了双编码器，提供更准确的位置和速度反馈，以满足高精度控制的需求。

关节顺序名称与关节限位
关节序号	关节名称	限位(弧度)
0	L_LEG_HIP_PITCH	-2.5307~2.8798
1	L_LEG_HIP_ROLL	-0.5236~2.9671
2	L_LEG_HIP_YAW	-2.7576~2.7576
3	L_LEG_KNEE	-0.087267~2.8798
4	L_LEG_ANKLE_PITCH	-0.87267~0.5236
5	L_LEG_ANKLE_ROLL	-0.2618~0.2618
6	R_LEG_HIP_PITCH	-2.5307~2.8798
7	R_LEG_HIP_ROLL	-2.9671~0.5236
8	R_LEG_HIP_YAW	-2.7576~2.7576
9	R_LEG_KNEE	-0.087267~2.8798
10	R_LEG_ANKLE_PITCH	-0.87267~0.5236
11	R_LEG_ANKLE_ROLL	-0.2618~0.2618
12	WAIST_YAW	-2.618~2.618
13	WAIST_ROLL	-0.52~0.52
14	WAIST_PITCH	-0.52~0.52
15	L_SHOULDER_PITCH	-3.0892~2.6704
16	L_SHOULDER_ROLL	-1.5882~2.2515
17	L_SHOULDER_YAW	-2.618~2.618
18	L_ELBOW	-1.0472~2.0944
19	L_WRIST_ROLL	-1.972222054~1.972222054
20	L_WRIST_PITCH	-1.614429558~1.614429558
21	L_WRIST_YAW	-1.614429558~1.614429558
22	R_SHOULDER_PITCH	-3.0892~2.6704
23	R_SHOULDER_ROLL	-2.2515~1.5882
24	R_SHOULDER_YAW	-2.618~2.618
25	R_ELBOW	-1.0472~2.0944
26	R_WRIST_ROLL	-1.972222054~1.972222054
27	R_WRIST_PITCH	-1.614429558~1.614429558
28	R_WRIST_YAW	-1.614429558~1.614429558


  <actuator>
    <motor name="left_hip_pitch" joint="left_hip_pitch_joint" ctrlrange="-88 88" />
    <motor name="left_hip_roll" joint="left_hip_roll_joint" ctrlrange="-88 88" />
    <motor name="left_hip_yaw" joint="left_hip_yaw_joint" ctrlrange="-88 88" />
    <motor name="left_knee" joint="left_knee_joint" ctrlrange="-139 139" />
    <motor name="left_ankle_pitch" joint="left_ankle_pitch_joint" ctrlrange="-50 50" />
    <motor name="left_ankle_roll" joint="left_ankle_roll_joint" ctrlrange="-50 50" />

    <motor name="right_hip_pitch" joint="right_hip_pitch_joint" ctrlrange="-88 88" />
    <motor name="right_hip_roll" joint="right_hip_roll_joint" ctrlrange="-88 88" />
    <motor name="right_hip_yaw" joint="right_hip_yaw_joint" ctrlrange="-88 88" />
    <motor name="right_knee" joint="right_knee_joint" ctrlrange="-139 139" />
    <motor name="right_ankle_pitch" joint="right_ankle_pitch_joint" ctrlrange="-50 50" />
    <motor name="right_ankle_roll" joint="right_ankle_roll_joint" ctrlrange="-50 50" />

    <motor name="waist_yaw" joint="waist_yaw_joint" ctrlrange="-88 88" />
    <motor name="waist_roll" joint="waist_roll_joint" ctrlrange="-50 50" />
    <motor name="waist_pitch" joint="waist_pitch_joint" ctrlrange="-50 50" />

    <motor name="left_shoulder_pitch" joint="left_shoulder_pitch_joint" ctrlrange="-25 25" />
    <motor name="left_shoulder_roll" joint="left_shoulder_roll_joint" ctrlrange="-25 25" />
    <motor name="left_shoulder_yaw" joint="left_shoulder_yaw_joint" ctrlrange="-25 25" />
    <motor name="left_elbow" joint="left_elbow_joint" ctrlrange="-25 25" />
    <motor name="left_wrist_roll" joint="left_wrist_roll_joint" ctrlrange="-25 25" />
    <motor name="left_wrist_pitch" joint="left_wrist_pitch_joint" ctrlrange="-5 5" />
    <motor name="left_wrist_yaw" joint="left_wrist_yaw_joint" ctrlrange="-5 5" />

    <motor name="right_shoulder_pitch" joint="right_shoulder_pitch_joint" ctrlrange="-25 25" />
    <motor name="right_shoulder_roll" joint="right_shoulder_roll_joint" ctrlrange="-25 25" />
    <motor name="right_shoulder_yaw" joint="right_shoulder_yaw_joint" ctrlrange="-25 25" />
    <motor name="right_elbow" joint="right_elbow_joint" ctrlrange="-25 25" />
    <motor name="right_wrist_roll" joint="right_wrist_roll_joint" ctrlrange="-25 25" />
    <motor name="right_wrist_pitch" joint="right_wrist_pitch_joint" ctrlrange="-5 5" />
    <motor name="right_wrist_yaw" joint="right_wrist_yaw_joint" ctrlrange="-5 5" />
  </actuator>