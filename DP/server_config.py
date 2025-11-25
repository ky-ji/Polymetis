"""
推理服务器配置文件

支持的训练配置:
1. train_cogact.sh (默认) - 320x180 图像, 全部 ~153 episodes
2. train_cogact_clean.sh (清洁) - 640x480 图像, episodes 65+ (~89 episodes)

服务器会自动检测模型期望的图像分辨率并调整输入图像。
"""

# 服务器配置
SERVER_IP = "0.0.0.0"              # 监听所有网卡
SERVER_PORT = 8006                 # 推理服务器端口

# 模型配置
# 示例路径 (请根据实际训练输出更新):
# - train_cogact.sh 输出: data/outputs/YYYY.MM.DD/HH.MM.SS_train_diffusion_transformer_hybrid_cogact_robot_7d/checkpoints/
# - train_cogact_clean.sh 输出: data/outputs/YYYY.MM.DD/HH.MM.SS_train_diffusion_transformer_hybrid_cogact_robot_7d_clean/checkpoints/
CHECKPOINT_PATH = "/home/kyji/storage_net/realworld_eval/diffusion_policy/data/outputs/2025.11.24/22.31.58_train_diffusion_transformer_hybrid_cogact_robot_7d/checkpoints/epoch=0160-train_loss=0.057.ckpt"
USE_EMA = True                     # 是否使用 EMA 模型

# 推理配置
DEVICE = "cuda:1"                  # 推理设备 (cuda:0, cuda:1, cuda:2, cuda:3 或 cpu)
                                   # GPU 0 被训练占用，使用 GPU 1
NUM_INFERENCE_STEPS = 100          # DDIM 推理步数（降低步数可加快推理速度）
INFERENCE_FREQ = 30.0              # 推理频率 (Hz)

# 图像配置
IMAGE_QUALITY = 85                 # JPEG 图像质量 (1-100)
IMAGE_RESIZE = True                # 是否调整图像大小
MAX_IMAGE_SIZE = (1920, 1080)      # 最大图像尺寸

# 通信配置
SOCKET_TIMEOUT = 5.0               # Socket 超时 (秒)
BUFFER_SIZE = 4096                 # 缓冲区大小
ENCODING = 'utf-8'                 # 编码格式
MAX_CLIENTS = 1                    # 最大客户端连接数

# 日志配置
VERBOSE = True                     # 是否打印详细日志
LOG_LEVEL = 'INFO'                 # 日志级别: DEBUG, INFO, WARNING, ERROR
