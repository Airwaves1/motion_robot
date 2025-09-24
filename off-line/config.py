import os

# 离线播放器配置（修改这里无需敲命令行）

# 数据文件路径（.npy，形状 Nx36，前7根姿态，后29关节弧度）
DATA_PATH = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "assets",
        "data",
        "Walk_1_Skeleton0_g1_29dof_rev_1_0_Tpose_new.npy",
    )
)

# 发布频率（Hz）
RATE_HZ = 250.0

# 是否循环播放
LOOP = True

# 关节角全局缩放
SCALE = 1.0

# 是否整体取反号（若npy与VRPN符号相反，可设为True）
NEGATE = False

# 是否将角度包裹到[-pi, pi]
CLAMP_PI = True


