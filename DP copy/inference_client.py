#!/usr/bin/env python3
"""
Polymetis 推理客户端（SSH 隧道版本）
通过 SSH 隧道传递实时控制数据
"""

import socket
import json
import time
import numpy as np
import cv2
import base64
import threading
import math
import subprocess
import torch
from pathlib import Path
from typing import Optional, Tuple, Dict
from threading import Thread, Lock, Event
from collections import deque

try:
    from polymetis import RobotInterface, GripperInterface
    print("✓ Polymetis 库导入成功")
except ImportError as e:
    print(f"✗ 无法导入 Polymetis 库: {e}")
    import sys
    sys.exit(1)

from camera_manager import CameraManager
from inference_config import (
    SSH_HOST, SSH_USER, SSH_KEY, SSH_PORT,
    SERVER_PORT, LOCAL_PORT,
    ROBOT_IP, ROBOT_PORT, GRIPPER_PORT,
    CAMERA_INDEX, CAMERA_RESOLUTION, IMAGE_QUALITY,
    INFERENCE_FREQ, N_OBS_STEPS, CAMERA_FREQ,
    ACTION_SCALE, STEPS_PER_INFERENCE,
    GRIPPER_OPEN_WIDTH, GRIPPER_CLOSED_WIDTH, GRIPPER_SPEED, GRIPPER_FORCE
)


class ObservationBuffer:
    """观测缓冲区管理器（实现时间对齐）"""
    
    def __init__(self, n_obs_steps: int, control_freq: float, camera_freq: float = 30.0):
        self.n_obs_steps = n_obs_steps
        self.control_freq = control_freq
        self.camera_freq = camera_freq
        self.dt = 1.0 / control_freq
        self.k = math.ceil(n_obs_steps * (camera_freq / control_freq))
        
        self.image_buffer = deque(maxlen=self.k)
        self.image_timestamps = deque(maxlen=self.k)
        self.pose_buffer = deque(maxlen=self.k)  # 7D位姿
        self.gripper_buffer = deque(maxlen=self.k)  # 1D夹爪
        self.state_timestamps = deque(maxlen=self.k)
        self.lock = Lock()
    
    def add_image(self, image: np.ndarray, timestamp: float):
        with self.lock:
            self.image_buffer.append(image)
            self.image_timestamps.append(timestamp)
    
    def add_state(self, pose_7d: np.ndarray, gripper_1d: np.ndarray, timestamp: float):
        """添加状态（拆分为7D位姿 + 1D夹爪）"""
        with self.lock:
            self.pose_buffer.append(pose_7d)
            self.gripper_buffer.append(gripper_1d)
            self.state_timestamps.append(timestamp)
    
    def get_aligned_obs(self) -> Optional[Dict]:
        with self.lock:
            if len(self.image_buffer) < self.n_obs_steps or len(self.pose_buffer) < self.n_obs_steps:
                return None
            
            last_image_timestamp = self.image_timestamps[-1]
            last_state_timestamp = self.state_timestamps[-1]
            last_timestamp = max(last_image_timestamp, last_state_timestamp)
            obs_align_timestamps = last_timestamp - (np.arange(self.n_obs_steps)[::-1] * self.dt)
            
            image_timestamps_arr = np.array(list(self.image_timestamps))
            aligned_images = []
            for t in obs_align_timestamps:
                is_before_idxs = np.nonzero(image_timestamps_arr < t)[0]
                idx = is_before_idxs[-1] if len(is_before_idxs) > 0 else 0
                aligned_images.append(self.image_buffer[idx])
            
            state_timestamps_arr = np.array(list(self.state_timestamps))
            aligned_poses = []
            aligned_grippers = []
            for t in obs_align_timestamps:
                is_before_idxs = np.nonzero(state_timestamps_arr < t)[0]
                idx = is_before_idxs[-1] if len(is_before_idxs) > 0 else 0
                aligned_poses.append(self.pose_buffer[idx])
                aligned_grippers.append(self.gripper_buffer[idx])
            
            return {
                'images': np.stack(aligned_images, axis=0),
                'poses': np.stack(aligned_poses, axis=0),  # (n_obs_steps, 7)
                'grippers': np.stack(aligned_grippers, axis=0),  # (n_obs_steps, 1)
                'timestamps': obs_align_timestamps
            }


class DPFormatConverter:
    """DP 格式转换器
    
    配置文件格式：
    - robot_eef_pose: [7] = [x, y, z, qx, qy, qz, qw]
    - robot_gripper_state: [1] = [gripper]
    - action: [7] = [x, y, z, qx, qy, qz, qw] (gripper单独处理)
    """
    
    GRIPPER_OPEN = 1.0
    GRIPPER_CLOSED = 0.0
    GRIPPER_THRESHOLD = 0.5
    
    @staticmethod
    def polymetis_to_dp_state(ee_pos: np.ndarray, ee_quat: np.ndarray, gripper_open: bool) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (7D位姿, 1D夹爪) 以匹配配置文件"""
        pose_7d = np.concatenate([ee_pos, ee_quat]).astype(np.float32)  # [x,y,z,qx,qy,qz,qw]
        gripper_value = DPFormatConverter.GRIPPER_OPEN if gripper_open else DPFormatConverter.GRIPPER_CLOSED
        gripper_1d = np.array([gripper_value], dtype=np.float32)
        return pose_7d, gripper_1d
    
    @staticmethod
    def dp_to_polymetis_action(pose_7d: np.ndarray, gripper_1d: np.ndarray) -> Tuple[np.ndarray, np.ndarray, bool]:
        """从 (7D位姿, 1D夹爪) 转换为 Polymetis 格式"""
        pose_7d = np.array(pose_7d).flatten()
        ee_pos = pose_7d[:3]  # [x, y, z]
        ee_quat = pose_7d[3:7]  # [qx, qy, qz, qw]
        gripper_value = float(gripper_1d[0]) if isinstance(gripper_1d, np.ndarray) else float(gripper_1d)
        gripper_open = gripper_value > DPFormatConverter.GRIPPER_THRESHOLD
        return ee_pos.astype(np.float32), ee_quat.astype(np.float32), gripper_open


class SSHTunnelClient:
    """SSH 隧道客户端"""
    
    def __init__(self, ssh_host: str, ssh_user: str, ssh_key: str, ssh_port: int, 
                 remote_port: int, local_port: int = 8006):
        self.ssh_host = ssh_host
        self.ssh_user = ssh_user
        self.ssh_key = ssh_key
        self.ssh_port = ssh_port
        self.remote_port = remote_port
        self.local_port = local_port
        self.tunnel_process = None
        self.socket = None
        
    def start_tunnel(self):
        print(f"[SSH隧道] 启动隧道: {self.ssh_host}:{self.remote_port} -> localhost:{self.local_port}")
        cmd = ['ssh', '-i', self.ssh_key, '-p', str(self.ssh_port),
               '-L', f'{self.local_port}:127.0.0.1:{self.remote_port}',
               '-N', f'{self.ssh_user}@{self.ssh_host}']
        
        try:
            self.tunnel_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(2)
            print(f"[SSH隧道] ✓ 隧道已建立")
            return True
        except Exception as e:
            print(f"[SSH隧道] ✗ 启动失败: {e}")
            return False
    
    def stop_tunnel(self):
        if self.tunnel_process:
            self.tunnel_process.terminate()
            self.tunnel_process.wait()
            print(f"[SSH隧道] ✓ 隧道已关闭")
    
    def connect(self, timeout: float = 5.0) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect(('localhost', self.local_port))
            print(f"[SSH隧道] ✓ 已连接")
            return True
        except Exception as e:
            print(f"[SSH隧道] ✗ 连接失败: {e}")
            return False
    
    def send_data(self, data: Dict) -> bool:
        try:
            msg = json.dumps(data) + '\n'
            self.socket.sendall(msg.encode('utf-8'))
            return True
        except Exception as e:
            print(f"[SSH隧道] 发送错误: {e}")
            return False
    
    def recv_data(self, timeout: float = 5.0) -> Optional[Dict]:
        try:
            self.socket.settimeout(timeout)
            data = b''
            while True:
                chunk = self.socket.recv(4096)
                if not chunk:
                    return None
                data += chunk
                if b'\n' in data:
                    break
            msg = data.decode('utf-8').strip()
            return json.loads(msg)
        except socket.timeout:
            return None
        except Exception as e:
            print(f"[SSH隧道] 接收错误: {e}")
            return None
    
    def close(self):
        if self.socket:
            self.socket.close()


class PolymetisInferenceClientSSH:
    """Polymetis 推理客户端"""
    
    def __init__(self, ssh_host: str = SSH_HOST, ssh_user: str = SSH_USER,
                 ssh_key: str = SSH_KEY, ssh_port: int = SSH_PORT,
                 server_port: int = SERVER_PORT, robot_ip: str = ROBOT_IP,
                 robot_port: int = ROBOT_PORT, gripper_port: int = GRIPPER_PORT,
                 camera_index: int = CAMERA_INDEX, camera_resolution: Tuple[int, int] = CAMERA_RESOLUTION,
                 inference_freq: float = INFERENCE_FREQ, n_obs_steps: int = N_OBS_STEPS,
                 camera_freq: float = CAMERA_FREQ, image_quality: int = IMAGE_QUALITY):
        
        self.ssh_tunnel = SSHTunnelClient(ssh_host, ssh_user, ssh_key, ssh_port, server_port, 8006)
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.gripper_port = gripper_port
        self.robot = None
        self.gripper = None
        
        self.camera_manager = CameraManager(camera_index, camera_resolution)
        self.inference_freq = inference_freq
        self.inference_interval = 1.0 / inference_freq
        self.n_obs_steps = n_obs_steps
        self.camera_freq = camera_freq
        self.image_quality = image_quality
        
        self.obs_buffer = ObservationBuffer(n_obs_steps, inference_freq, camera_freq)
        self.running = False
        self.data_lock = Lock()
        self.latest_action = None
        self.action_received = Event()
        
        self.actions_received = 0
        self.observations_sent = 0
        self.gripper_open = True
        self.last_gripper_state = True  # 记录上一次夹爪状态，避免重复发送命令
        
        self.control_thread = None
        self.current_target_pos = None
        self.current_target_quat = None
        self.target_lock = Lock()
        
        # 时间戳管理（用于动作过滤）
        self.eval_t_start = None  # 评估开始时间
        self.iter_idx = 0  # 推理迭代索引
        
        self.trajectory_log = {
            'observations': [],
            'actions': [],
            'executed': []
        }

    def run(self):
        print("\n" + "="*70)
        print("Polymetis 推理客户端 (SSH 隧道版本)")
        print("="*70)
        
        try:
            if not self.ssh_tunnel.start_tunnel():
                print("SSH 隧道启动失败")
                return
            
            time.sleep(2)
            
            if not self.ssh_tunnel.connect():
                print("连接服务器失败")
                self.ssh_tunnel.stop_tunnel()
                return
            
            print("\n[客户端] 初始化机械臂...")
            self._initialize_robot()
            
            print("[客户端] 初始化摄像头...")
            self.camera_manager.start()
            
            self.running = True
            
            # 启动观测收集线程
            obs_thread = Thread(target=self._collect_observations, daemon=True)
            obs_thread.start()
            
            recv_thread = Thread(target=self._receive_loop, daemon=True)
            recv_thread.start()
            
            self.control_thread = Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
            
            self._inference_loop()
            
        except KeyboardInterrupt:
            print("\n[客户端] 检测到 Ctrl+C，正在停止...")
        except Exception as e:
            print(f"程序出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()

    def _initialize_robot(self):
        try:
            self.robot = RobotInterface(ip_address=self.robot_ip, port=self.robot_port)
            print(f"已连接到机器人")
            
            try:
                self.gripper = GripperInterface(ip_address=self.robot_ip, port=self.gripper_port)
                print(f"已连接到夹爪")
            except:
                self.gripper = None
                print(f"夹爪服务器未启动")
            
            print("启动笛卡尔阻抗控制...")
            self.robot.start_cartesian_impedance()
            
            ee_pos, ee_quat = self.robot.get_ee_pose()
            with self.target_lock:
                self.current_target_pos = ee_pos.cpu().numpy()
                self.current_target_quat = ee_quat.cpu().numpy()
            
            print("笛卡尔阻抗控制已启动")
        except Exception as e:
            print(f"初始化机器人失败: {e}")
            raise

    def _control_loop(self):
        print("[控制线程] 已启动，频率: 30 Hz")
        rate = 1.0 / 30.0
        
        while self.running:
            try:
                loop_start = time.time()
                
                with self.target_lock:
                    if self.current_target_pos is not None and self.current_target_quat is not None:
                        target_pos = torch.from_numpy(self.current_target_pos).float()
                        target_quat = torch.from_numpy(self.current_target_quat).float()
                        self.robot.update_desired_ee_pose(position=target_pos, orientation=target_quat)
                
                elapsed = time.time() - loop_start
                sleep_time = rate - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                if not self.running:
                    break
                print(f"[控制线程] 错误: {e}")
                time.sleep(0.1)

    def _collect_observations(self):
        print("[客户端] 观测收集线程已启动 (30Hz)")
        
        while self.running:
            try:
                frame = self.camera_manager.read_frame()
                if frame is not None:
                    self.obs_buffer.add_image(frame, time.time())
                
                ee_pos, ee_quat = self.robot.get_ee_pose()
                if ee_pos is not None and ee_quat is not None:
                    ee_pos_np = ee_pos.cpu().numpy()
                    ee_quat_np = ee_quat.cpu().numpy()
                    pose_7d, gripper_1d = DPFormatConverter.polymetis_to_dp_state(ee_pos_np, ee_quat_np, self.gripper_open)
                    self.obs_buffer.add_state(pose_7d, gripper_1d, time.time())
                
                time.sleep(1.0 / 30.0)  # 33ms = 30Hz
            except Exception as e:
                if self.running:
                    print(f"[客户端] 观测收集错误: {e}")

    def _receive_loop(self):
        while self.running:
            try:
                data = self.ssh_tunnel.recv_data(timeout=1.0)
                if data and data.get('type') == 'action':
                    action = np.array(data.get('action'), dtype=np.float32)
                    self.trajectory_log['actions'].append({'step': self.actions_received, 'action': action.tolist()})
                    
                    with self.data_lock:
                        self.latest_action = action
                        self.actions_received += 1
                    self.action_received.set()
            except Exception as e:
                if self.running:
                    print(f"[客户端] 接收错误: {e}")

    def _inference_loop(self):
        dt = self.inference_interval  # 基础时间步长
        actual_inference_interval = dt * STEPS_PER_INFERENCE  # 实际推理间隔
        print(f"[客户端] 推理循环已启动")
        print(f"  基础频率: {1/dt:.1f}Hz (dt={dt:.3f}s)")
        print(f"  实际推理频率: {1/actual_inference_interval:.1f}Hz (每次执行{STEPS_PER_INFERENCE}步)")
        
        # 等待观测缓冲区填充
        print("[客户端] 等待观测缓冲区填充...")
        start_delay = 0.5
        self.eval_t_start = time.time() + start_delay
        t_start = time.monotonic() + start_delay
        time.sleep(start_delay)
        print("[客户端] ✓ 开始推理控制")
        
        frame_latency = 1/30  # 摄像头延迟补偿
        
        while self.running:
            try:
                aligned_obs = self.obs_buffer.get_aligned_obs()
                if aligned_obs is None:
                    time.sleep(0.01)
                    continue
                
                # 记录观测（拼接pose+gripper用于日志）
                last_pose = aligned_obs['poses'][-1]
                last_gripper = aligned_obs['grippers'][-1]
                self.trajectory_log['observations'].append({
                    'step': self.observations_sent,
                    'pose': last_pose.tolist(),
                    'gripper': last_gripper.tolist(),
                    'n_obs_steps': self.n_obs_steps
                })
                
                images_b64 = []
                for img in aligned_obs['images']:
                    _, img_encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.image_quality])
                    images_b64.append(base64.b64encode(img_encoded).decode('utf-8'))
                
                # 发送观测：分别发送poses和grippers
                obs_msg = {
                    'type': 'observation',
                    'images': images_b64,
                    'poses': aligned_obs['poses'].astype(np.float32).tolist(),  # (n_obs_steps, 7)
                    'grippers': aligned_obs['grippers'].astype(np.float32).tolist(),  # (n_obs_steps, 1)
                    'timestamps': aligned_obs['timestamps'].tolist()
                }
                
                if self.ssh_tunnel.send_data(obs_msg):
                    self.observations_sent += 1
                    print(f"[客户端] 发送观测 #{self.observations_sent} (iter={self.iter_idx})")
                
                # 计算本次推理周期结束时间
                t_cycle_end = t_start + (self.iter_idx + STEPS_PER_INFERENCE) * dt
                
                if self.action_received.wait(timeout=actual_inference_interval):
                    self.action_received.clear()
                    
                    with self.data_lock:
                        action = self.latest_action.copy() if self.latest_action is not None else None
                    
                    if action is not None:
                        if action.ndim == 1:
                            action_sequence = [action]
                        else:
                            action_sequence = action
                        
                        # ========== 时间戳过滤机制（参考 Diffusion Policy 官方代码）==========
                        # 获取观测时间戳
                        obs_timestamps = aligned_obs['timestamps']
                        
                        # 为每个动作分配时间戳
                        # action_timestamps[i] = obs_time + i * dt
                        dt = self.inference_interval
                        action_offset = 0  # 动作偏移量
                        action_timestamps = (np.arange(len(action_sequence), dtype=np.float64) + action_offset
                            ) * dt + obs_timestamps[-1]
                        
                        # 过滤过期的动作
                        action_exec_latency = 0.01  # 10ms 执行延迟
                        curr_time = time.time()
                        is_new = action_timestamps > (curr_time + action_exec_latency)
                        
                        if np.sum(is_new) == 0:
                            # 所有动作都过期了，执行最后一个动作
                            print("[客户端] 警告: 所有动作已过期，执行最后一个")
                            action_sequence_filtered = [action_sequence[-1]]
                            # 调度到下一个可用步骤
                            next_step_idx = int(np.ceil((curr_time - self.eval_t_start) / dt))
                            action_timestamp = self.eval_t_start + next_step_idx * dt
                            action_timestamps_filtered = np.array([action_timestamp])
                            print(f"[客户端] 超出时间预算 {action_timestamp - curr_time:.3f}s")
                        else:
                            # 只执行未过期的动作
                            action_sequence_filtered = [action_sequence[i] for i in range(len(action_sequence)) if is_new[i]]
                            action_timestamps_filtered = action_timestamps[is_new]
                        
                        # 关键：只执行前 STEPS_PER_INFERENCE 个动作（防止重叠执行）
                        if len(action_sequence_filtered) > STEPS_PER_INFERENCE:
                            action_sequence_filtered = action_sequence_filtered[:STEPS_PER_INFERENCE]
                            action_timestamps_filtered = action_timestamps_filtered[:STEPS_PER_INFERENCE]
                        
                        print(f"[客户端] 收到 {len(action_sequence)} 个动作，过滤后 {len([a for a in is_new if a])} 个未过期，执行 {len(action_sequence_filtered)} 个 (steps_per_inference={STEPS_PER_INFERENCE})")
                        
                        # 执行过滤后的动作
                        for step_idx, single_action in enumerate(action_sequence_filtered):
                            # single_action 应该是 {'pose': [7], 'gripper': [1]} 或者是两个数组
                            if isinstance(single_action, dict):
                                pose_7d = np.array(single_action['pose'])
                                gripper_1d = np.array(single_action['gripper'])
                            else:
                                # 如果是数组，假设前7个pose，后1个gripper
                                single_action = np.array(single_action).flatten()
                                if len(single_action) == 8:
                                    pose_7d = single_action[:7]
                                    gripper_1d = single_action[7:8]
                                elif len(single_action) == 7:
                                    pose_7d = single_action
                                    gripper_1d = np.array([1.0])  # 默认打开
                                else:
                                    print(f"[客户端] 警告: 动作维度不匹配: {len(single_action)}")
                                    continue
                            
                            target_pos, target_quat, target_gripper_open = DPFormatConverter.dp_to_polymetis_action(pose_7d, gripper_1d)
                            
                            current_pos, current_quat = self.robot.get_ee_pose()
                            if current_pos is None or current_quat is None:
                                continue
                            
                            current_pos_np = current_pos.cpu().numpy()
                            delta_pos = target_pos - current_pos_np
                            scaled_target_pos = current_pos_np + delta_pos * ACTION_SCALE
                            scaled_target_quat = target_quat
                            
                            with self.target_lock:
                                self.current_target_pos = scaled_target_pos
                                self.current_target_quat = scaled_target_quat
                            
                            # 夹爪控制：只在状态变化时执行
                            if self.gripper is not None and target_gripper_open != self.last_gripper_state:
                                action_name = "打开" if target_gripper_open else "关闭"
                                print(f"[客户端] 夹爪{action_name}...")
                                try:
                                    if target_gripper_open:
                                        # 打开：使用 goto
                                        self.gripper.goto(
                                            width=0.09,
                                            speed=0.3,
                                            force=1.0,
                                            blocking=True
                                        )
                                    else:
                                        # 关闭/抓取：使用 grasp
                                        self.gripper.grasp(
                                            speed=0.2,
                                            force=1.0,
                                            grasp_width=0.0,
                                            epsilon_inner=0.1,
                                            epsilon_outer=0.1,
                                            blocking=True
                                        )
                                    # 验证夹爪状态
                                    actual_width = self.gripper.get_state().width
                                    print(f"✓ 夹爪已{action_name} (实际宽度: {actual_width:.4f}m)")
                                    self.last_gripper_state = target_gripper_open
                                except Exception as e:
                                    print(f"✗ 夹爪控制失败: {e}")
                            
                            self.gripper_open = target_gripper_open
                            
                            self.trajectory_log['executed'].append({
                                'step': self.actions_received,
                                'action_step': step_idx,
                                'pos': scaled_target_pos.tolist(),
                                'quat': scaled_target_quat.tolist(),
                                'gripper_open': target_gripper_open,
                                'timestamp': action_timestamps_filtered[step_idx]
                            })
                            
                            # 等待到动作的目标时间戳
                            target_time = action_timestamps_filtered[step_idx]
                            wait_time = target_time - time.time()
                            if wait_time > 0:
                                time.sleep(wait_time)
                            
                            print(f"[客户端] 执行动作 #{self.actions_received} 步 {step_idx+1}/{len(action_sequence_filtered)}")
                        
                        # 更新迭代索引
                        self.iter_idx += STEPS_PER_INFERENCE
                        
                        # 等待到下一个推理周期（关键：控制推理频率）
                        wait_until = t_cycle_end - frame_latency
                        wait_time = wait_until - time.monotonic()
                        if wait_time > 0:
                            time.sleep(wait_time)
                        else:
                            print(f"[客户端] 警告: 推理+执行超时 {-wait_time:.3f}s")
                else:
                    # 没有收到动作，等待一小段时间
                    time.sleep(0.01)
                
            except Exception as e:
                print(f"[客户端] 推理循环错误: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _save_trajectory_log(self):
        try:
            from datetime import datetime
            
            def convert_to_serializable(obj):
                if isinstance(obj, dict):
                    return {k: convert_to_serializable(v) for k, v in obj.items()}
                elif isinstance(obj, list):
                    return [convert_to_serializable(item) for item in obj]
                elif isinstance(obj, (np.bool_, np.integer, np.floating)):
                    return obj.item()
                else:
                    return obj
            
            serializable_log = convert_to_serializable(self.trajectory_log)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = f"trajectory_log_{timestamp}.json"
            
            with open(log_file, 'w') as f:
                json.dump(serializable_log, f, indent=2)
            
            print(f"[客户端] 轨迹日志已保存: {log_file}")
        except Exception as e:
            print(f"[客户端] 保存轨迹日志失败: {e}")
    
    def stop(self):
        self.running = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        if self.camera_manager:
            self.camera_manager.stop()
        
        self.ssh_tunnel.close()
        self.ssh_tunnel.stop_tunnel()
        
        print(f"\n[客户端] ✓ 已停止")
        print(f"[客户端] 总共发送 {self.observations_sent} 个观测")
        print(f"[客户端] 总共接收 {self.actions_received} 个动作")
        
        self._save_trajectory_log()


if __name__ == "__main__":
    client = PolymetisInferenceClientSSH()
    try:
        client.run()
    except KeyboardInterrupt:
        print("\n[客户端] 检测到 Ctrl+C，正在停止...")
        client.stop()
