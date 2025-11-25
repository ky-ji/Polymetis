"""
USB摄像头管理器
用于数据采集时的图像采集
"""
import cv2
import numpy as np
from typing import Optional, Tuple
import time


class CameraManager:
    """USB摄像头管理器"""
    
    def __init__(self, 
                 camera_index: int = 0,
                 resolution: Tuple[int, int] = (1920, 1080),
                 fps: int = 30):
        """
        初始化摄像头管理器
        
        Args:
            camera_index: 摄像头索引（默认0）
            resolution: 图像分辨率 (width, height)
            fps: 摄像头帧率
        """
        self.camera_index = camera_index
        self.resolution = resolution
        self.fps = fps
        self.cap = None
        self.is_opened = False
        
        # 统计信息
        self.frames_captured = 0
        self.failed_reads = 0
        self.last_frame_time = 0
        
    def start(self) -> bool:
        """
        启动摄像头
        
        Returns:
            bool: 是否成功启动
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                print(f"[摄像头] ✗ 无法打开摄像头 {self.camera_index}")
                return False
            
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            # 设置帧率
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # 设置缓冲区大小为1，减少旧帧堆积
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # 获取实际设置的参数
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            print(f"[摄像头] ✓ 摄像头已启动")
            print(f"  索引: {self.camera_index}")
            print(f"  分辨率: {actual_width}x{actual_height}")
            print(f"  帧率: {actual_fps} fps")
            
            # 预热摄像头（清空初始缓冲区）
            self.clear_buffer()
            
            self.is_opened = True
            return True
            
        except Exception as e:
            print(f"[摄像头] ✗ 启动失败: {e}")
            return False
    
    def clear_buffer(self, n_frames: int = 10):
        """
        清空摄像头缓冲区
        使用grab()方法快速丢弃缓冲区中的旧帧
        
        Args:
            n_frames: 要清空的帧数（默认10帧）
        """
        if self.cap is None:
            return
        
        for _ in range(n_frames):
            self.cap.grab()  # grab()比read()快，因为不解码
    
    def read_frame(self) -> Optional[np.ndarray]:
        """
        读取一帧图像
        
        Returns:
            np.ndarray or None: 图像数据 (BGR格式)，失败返回None
        """
        if not self.is_opened or self.cap is None:
            return None
        
        try:
            ret, frame = self.cap.read()
            
            if ret:
                self.frames_captured += 1
                self.last_frame_time = time.time()
                return frame
            else:
                self.failed_reads += 1
                return None
                
        except Exception as e:
            print(f"[摄像头] 读取帧失败: {e}")
            self.failed_reads += 1
            return None
    
    def stop(self):
        """停止摄像头"""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        
        self.is_opened = False
        
        print(f"\n[摄像头] 统计信息:")
        print(f"  总帧数: {self.frames_captured}")
        print(f"  失败次数: {self.failed_reads}")
        if self.frames_captured > 0:
            success_rate = (self.frames_captured / (self.frames_captured + self.failed_reads)) * 100
            print(f"  成功率: {success_rate:.1f}%")
    
    def __enter__(self):
        """上下文管理器入口"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.stop()
