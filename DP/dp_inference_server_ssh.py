#!/usr/bin/env python3
"""
Diffusion Policy 推理服务器（SSH 隧道版本）
通过 SSH 隧道接收实时控制数据
- 接收观测 (图像 + 7D 状态)
- 运行 DP 推理
- 发送 7D 动作
"""

import socket
import json
import torch
import numpy as np
import cv2
import base64
import threading
from pathlib import Path
from typing import Optional, Dict
import hydra
import dill
import sys

# 添加 diffusion_policy 路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.real_world.real_inference_util import get_real_obs_dict

from server_config import (
    SERVER_IP, SERVER_PORT, CHECKPOINT_PATH, USE_EMA,
    DEVICE, NUM_INFERENCE_STEPS, INFERENCE_FREQ,
    SOCKET_TIMEOUT, BUFFER_SIZE, ENCODING, MAX_CLIENTS, VERBOSE
)


class DPInferenceServerSSH:
    """Diffusion Policy 推理服务器（SSH 隧道版本）"""
    
    def __init__(self,
                 checkpoint_path: str = CHECKPOINT_PATH,
                 use_ema: bool = USE_EMA,
                 device: str = DEVICE,
                 num_inference_steps: int = NUM_INFERENCE_STEPS,
                 inference_freq: float = INFERENCE_FREQ,
                 server_ip: str = SERVER_IP,
                 server_port: int = SERVER_PORT,
                 max_clients: int = MAX_CLIENTS,
                 verbose: bool = VERBOSE):
        """初始化推理服务器"""
        
        self.checkpoint_path = checkpoint_path
        self.use_ema = use_ema
        self.device = device
        self.num_inference_steps = num_inference_steps
        self.inference_freq = inference_freq
        self.server_ip = server_ip
        self.server_port = server_port
        self.max_clients = max_clients
        self.verbose = verbose
        
        self.policy = None
        self.cfg = None
        self.running = False
        self.expected_image_shape = None  # 模型期望的图像尺寸 (H, W)
        self.obs_keys = None  # 模型期望的观测键名 {'rgb': str, 'lowdim': [str, ...]}
        
        # 观测历史缓存 (用于 n_obs_steps)
        self.obs_history = {
            'images': [],   # RGB 图像历史
            'states': []    # 状态历史
        }
        
        # 轨迹记录
        self.inference_log = {
            'steps': []   # 每步包含 input 和 action
        }
        
        print("[推理服务器] 初始化...")
        self._load_model()
    
    def _load_model(self):
        """加载模型"""
        print(f"[推理服务器] 加载模型: {self.checkpoint_path}")
        
        try:
            payload = torch.load(open(self.checkpoint_path, 'rb'), pickle_module=dill)
            self.cfg = payload['cfg']
            
            cls = hydra.utils.get_class(self.cfg._target_)
            workspace = cls(self.cfg)
            workspace.load_payload(payload, exclude_keys=None, include_keys=None)
            
            self.policy = workspace.model
            if self.use_ema:
                self.policy = workspace.ema_model
            
            self.policy.eval().to(self.device)
            
            # 修改为 DDPM scheduler 并设置推理步数
            from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
            
            # 创建 DDPM scheduler（使用与训练时相同的配置）
            self.policy.noise_scheduler = DDPMScheduler(
                num_train_timesteps=100,
                beta_start=0.0001,
                beta_end=0.02,
                beta_schedule='squaredcos_cap_v2',
                clip_sample=True,
                prediction_type='epsilon',
                variance_type='fixed_small'
            )
            
            # 设置推理步数
            self.policy.num_inference_steps = self.num_inference_steps
            
            # 提取模型期望的观测键和图像尺寸
            shape_meta = self.cfg.task.shape_meta
            rgb_keys = [k for k, v in shape_meta['obs'].items() if v.get('type') == 'rgb']
            lowdim_keys = [k for k, v in shape_meta['obs'].items() if v.get('type') == 'low_dim']

            self.obs_keys = {
                'rgb': rgb_keys[0] if rgb_keys else None,
                'lowdim': lowdim_keys
            }

            if self.obs_keys['rgb']:
                image_shape = shape_meta['obs'][self.obs_keys['rgb']]['shape']  # [C, H, W]
                self.expected_image_shape = (image_shape[1], image_shape[2])  # (H, W)

            print(f"[推理服务器] ✓ 模型加载成功")
            print(f"[推理服务器] 期望图像尺寸: {self.expected_image_shape} (H, W)")
            print(f"[推理服务器] RGB 观测键: {self.obs_keys['rgb']}")
            print(f"[推理服务器] Low-dim 观测键: {self.obs_keys['lowdim']}")
            print(f"[推理服务器] 动作模式: 绝对位姿 (Absolute Pose)")
            print(f"[推理服务器] Scheduler: DDPM (50 steps)")
            
            # 预热模型
            self._warmup_model()
            
        except Exception as e:
            print(f"[推理服务器] ✗ 模型加载失败: {e}")
            raise
    
    def _warmup_model(self):
        """预热模型"""
        print("[推理服务器] 预热模型...")
        
        try:
            batch_size = 1
            n_obs_steps = self.policy.n_obs_steps
            
            # 获取观测规格
            shape_meta = self.cfg.task.shape_meta
            obs_keys = shape_meta['obs']
            
            # 找到 RGB 观测键
            rgb_key = None
            for key, spec in obs_keys.items():
                if spec.get('type') == 'rgb':
                    rgb_key = key
                    break
            
            if rgb_key is None:
                raise ValueError("未找到 RGB 类型的观测键")
            
            image_shape = shape_meta['obs'][rgb_key]['shape']
            
            # 创建虚拟观测
            dummy_image = torch.randn(
                batch_size, n_obs_steps, *image_shape,
                device=self.device, dtype=torch.float32
            )
            
            obs_dict = {rgb_key: dummy_image}
            
            # 添加 low_dim 观测
            for key, spec in obs_keys.items():
                if spec.get('type') == 'low_dim':
                    low_dim_shape = spec['shape']
                    dummy_low_dim = torch.randn(
                        batch_size, n_obs_steps, *low_dim_shape,
                        device=self.device, dtype=torch.float32
                    )
                    obs_dict[key] = dummy_low_dim
            
            # 推理
            with torch.no_grad():
                self.policy.reset()
                result = self.policy.predict_action(obs_dict)
                action = result['action'][0].detach().to('cpu').numpy()
            
            print(f"[推理服务器] ✓ 模型预热成功 (输出形状: {action.shape})")
            
        except Exception as e:
            print(f"[推理服务器] ✗ 模型预热失败: {e}")
            raise
    
    def start(self):
        """启动服务器"""
        print("\n" + "="*70)
        print("Diffusion Policy 推理服务器 (SSH 隧道版本)")
        print("="*70)
        print(f"\n配置:")
        print(f"  监听地址: {self.server_ip}:{self.server_port}")
        print(f"  推理设备: {self.device}")
        print(f"  推理步数: {self.num_inference_steps}")
        print(f"  推理频率: {self.inference_freq} Hz")
        print(f"\n按 Ctrl+C 停止\n")
        
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.server_ip, self.server_port))
            server_socket.listen(self.max_clients)
            
            print(f"[推理服务器] ✓ 已启动，监听 {self.server_ip}:{self.server_port}")
            print(f"[推理服务器] 等待客户端连接...\n")
            
            self.running = True
            
            while self.running:
                try:
                    client_socket, client_addr = server_socket.accept()
                    print(f"[推理服务器] ✓ 客户端已连接: {client_addr}")
                    
                    # 处理客户端
                    self._handle_client(client_socket, client_addr)
                    
                except KeyboardInterrupt:
                    print(f"\n[推理服务器] 检测到 Ctrl+C，正在停止...")
                    break
                except Exception as e:
                    print(f"[推理服务器] 错误: {e}")
            
            server_socket.close()
            print(f"[推理服务器] ✓ 已停止")
            
            # 保存推理日志
            self._save_inference_log()
            
        except Exception as e:
            print(f"[推理服务器] ✗ 启动失败: {e}")
    
    def _handle_client(self, client_socket: socket.socket, client_addr: tuple):
        """处理客户端连接"""
        try:
            client_socket.settimeout(SOCKET_TIMEOUT)
            buffer = b''
            
            while self.running:
                try:
                    data = client_socket.recv(BUFFER_SIZE)
                    if not data:
                        break
                    
                    buffer += data
                    
                    # 处理完整的 JSON 消息
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        msg = line.decode(ENCODING).strip()
                        
                        if msg:
                            self._process_message(client_socket, msg)
                
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[推理服务器] 接收错误: {e}")
                    break
            
            client_socket.close()
            print(f"[推理服务器] 客户端断开连接: {client_addr}")
            
        except Exception as e:
            print(f"[推理服务器] 处理客户端错误: {e}")
    
    def _process_message(self, client_socket: socket.socket, message: str):
        """处理客户端消息"""
        try:
            data = json.loads(message)
            
            if data.get('type') == 'reset':
                # 处理reset请求（episode开始）
                self.policy.reset()
                print("[推理服务器] ✓ 策略已reset")
                
                # 发送确认
                response = {'type': 'reset_ack'}
                msg = json.dumps(response) + '\n'
                client_socket.sendall(msg.encode(ENCODING))
                
            elif data.get('type') == 'observation':
                # 解码完整的时间对齐观测（poses和grippers分开）
                images_b64 = data.get('images', [])
                poses_list = data.get('poses', [])  # (n_obs_steps, 7)
                grippers_list = data.get('grippers', [])  # (n_obs_steps, 1)
                timestamps = np.array(data.get('timestamps', []), dtype=np.float32)

                # 解码所有图像
                images = []
                for img_b64 in images_b64:
                    img_data = base64.b64decode(img_b64)
                    img_array = np.frombuffer(img_data, dtype=np.uint8)
                    image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    # BGR → RGB 转换（与官方对齐）
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                    # 自动调整图像尺寸以匹配模型期望
                    if self.expected_image_shape is not None:
                        expected_h, expected_w = self.expected_image_shape
                        current_h, current_w = image.shape[:2]

                        if (current_h, current_w) != (expected_h, expected_w):
                            if self.verbose:
                                print(f"[推理服务器] 调整图像尺寸: {current_h}x{current_w} -> {expected_h}x{expected_w}")
                            image = cv2.resize(image, (expected_w, expected_h), interpolation=cv2.INTER_LINEAR)

                    images.append(image)

                # 转换为 numpy 数组
                poses = np.array(poses_list, dtype=np.float32)  # (n_obs_steps, 7)
                grippers = np.array(grippers_list, dtype=np.float32)  # (n_obs_steps, 1)

                # 构建 env_obs 格式，使用模型期望的键名
                env_obs = {}

                # 添加图像观测
                if self.obs_keys['rgb']:
                    env_obs[self.obs_keys['rgb']] = np.stack(images, axis=0).astype(np.uint8)  # (n_obs_steps, H, W, C)

                # 添加 low-dim 观测（根据配置文件的键名）
                # 假设 lowdim_keys 是 ['robot_eef_pose', 'robot_gripper_state']
                shape_meta = self.cfg.task.shape_meta
                for lowdim_key in self.obs_keys['lowdim']:
                    if 'pose' in lowdim_key.lower():
                        env_obs[lowdim_key] = poses  # (n_obs_steps, 7)
                    elif 'gripper' in lowdim_key.lower():
                        env_obs[lowdim_key] = grippers  # (n_obs_steps, 1)

                # 运行推理
                action = self._infer_action(env_obs, timestamps)
                
                # 发送动作（返回完整的7D动作序列，gripper由客户端从动作中提取）
                response = {
                    'type': 'action',
                    'action': action.tolist()  # (n_action_steps, 7) 或 (n_action_steps, 8)
                }
                
                msg = json.dumps(response) + '\n'
                client_socket.sendall(msg.encode(ENCODING))
                
                if self.verbose:
                    print(f"[推理服务器] 推理完成，发送动作")
        
        except Exception as e:
            print(f"[推理服务器] 处理消息错误: {e}")
            import traceback
            traceback.print_exc()
    
    def reset_obs_history(self):
        """重置观测历史（用于新 episode）"""
        self.obs_history['images'].clear()
        self.obs_history['states'].clear()
        print("[推理服务器] 观测历史已重置")
    
    def _infer_action(self, env_obs: dict, timestamps: np.ndarray) -> np.ndarray:
        """运行推理（使用官方 get_real_obs_dict 方法）

        Args:
            env_obs: 环境观测字典，包含时间对齐的完整观测
                - RGB key (e.g., 'image' or 'camera_0'): (n_obs_steps, H, W, C) RGB uint8
                - Low-dim keys (e.g., 'robot_eef_pose', 'robot_gripper_state'): (n_obs_steps, dim) float32
            timestamps: (n_obs_steps,) 对齐的时间戳
        """
        try:
            # 记录输入（拼接所有 lowdim 观测）
            lowdim_obs_list = []
            for lowdim_key in self.obs_keys['lowdim']:
                if lowdim_key in env_obs:
                    lowdim_obs_list.append(env_obs[lowdim_key][-1])

            last_state = np.concatenate(lowdim_obs_list) if lowdim_obs_list else np.array([])
            
            # 创建当前步的记录
            current_step = {
                'step': len(self.inference_log['steps']),
                'input': {
                    'state': last_state.astype(np.float32).tolist(),
                    'n_obs_steps': len(timestamps),
                    'timestamp': float(timestamps[-1]) if len(timestamps) > 0 else 0.0
                }
            }
            
            shape_meta = self.cfg.task.shape_meta

            # env_obs 已经使用正确的键名，直接添加时间戳
            env_obs['timestamp'] = timestamps

            # 使用官方的 get_real_obs_dict 处理观测
            obs_dict_np = get_real_obs_dict(
                env_obs=env_obs,
                shape_meta=shape_meta
            )
            
            # 转换为 PyTorch 张量
            obs_dict = dict_apply(obs_dict_np,
                lambda x: torch.from_numpy(x).unsqueeze(0).to(self.device))
            
            # 推理
            # 注意：不应该每次都reset！reset应该只在episode开始时调用
            with torch.no_grad():
                result = self.policy.predict_action(obs_dict)
                action = result['action'][0].detach().to('cpu').numpy()

            # 检查动作维度
            # 如果模型输出是7D (只有pose)，需要添加gripper维度（旧模型兼容）
            # 如果模型输出是8D (pose + gripper)，直接使用（新模型）
            if action.shape[-1] == 7:
                # 旧模型：只输出7D pose，gripper需要从观测中获取
                # 这里使用最后一个观测的gripper状态（保持不变）
                last_gripper = env_obs[self.obs_keys['lowdim'][-1]][-1]  # 获取最后一个gripper观测
                # 为每个动作步添加gripper
                n_action_steps = action.shape[0]
                gripper_actions = np.tile(last_gripper, (n_action_steps, 1))  # (n_action_steps, 1)
                action_with_gripper = np.concatenate([action, gripper_actions], axis=-1)  # (n_action_steps, 8)
                if self.verbose:
                    print(f"[推理服务器] 旧模型 (7D)，gripper 保持为: {last_gripper[0]}")
            else:
                # 新模型：已经输出了完整的8D动作 (pose + gripper)
                action_with_gripper = action
                if self.verbose:
                    print(f"[推理服务器] 新模型 (8D)，gripper 由模型预测")

            # CogAct 模型输出绝对位姿，无需累积处理
            if self.verbose:
                print(f"[推理服务器] 推理完成，动作shape: {action_with_gripper.shape}")
            
            # 添加动作到当前步记录
            current_step['action'] = {
                'values': action_with_gripper.astype(np.float32).tolist(),
                'shape': list(action_with_gripper.shape)
            }
            
            # 保存完整的步记录
            self.inference_log['steps'].append(current_step)
            
            # 返回完整动作序列（客户端决定使用哪些步）
            return action_with_gripper.astype(np.float32)
        
        except Exception as e:
            print(f"[推理服务器] 推理错误: {e}")
            import traceback
            traceback.print_exc()
            # 返回零动作序列 (8D: 7D pose + 1D gripper)
            return np.zeros((self.policy.n_action_steps, 8), dtype=np.float32)
    
    def _save_inference_log(self):
        """保存推理日志到文件"""
        try:
            import json
            from datetime import datetime
            
            # 创建日志目录
            log_dir = Path(__file__).parent / "log"
            log_dir.mkdir(exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = log_dir / f"inference_log_{timestamp}.json"
            
            with open(log_file, 'w') as f:
                json.dump(self.inference_log, f, indent=2)
            
            print(f"[推理服务器] 推理日志已保存: {log_file}")
            print(f"[推理服务器] - 推理步数: {len(self.inference_log['steps'])} 条")
        except Exception as e:
            print(f"[推理服务器] 保存推理日志失败: {e}")


if __name__ == "__main__":
    server = DPInferenceServerSSH()
    server.start()
