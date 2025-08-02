import os
import cv2
import datetime

class VideoRecorder:
    def __init__(self, output_path="recordings", fps=30, codec='XVID', resolution=None):
        """
        初始化视频录制器
        
        参数:
            output_path (str): 视频保存路径
            fps (int): 输出视频的帧率
            codec (str): 四字符编码代码
            resolution (tuple): 可选的调整帧大小的(宽度, 高度)
        """
        self.output_path = output_path
        self.fps = fps
        self.codec = codec
        self.resolution = resolution
        self.writer = None
        self.is_recording = False
        self.current_filename = None
        
    def start(self, frame=None):
        """开始录制视频"""
        if self.is_recording:
            return
            
        if frame is not None and self.resolution is None:
            h, w = frame.shape[:2]
            self.resolution = (w, h)
        
        # 如果目录不存在则创建
        os.makedirs(self.output_path, exist_ok=True)
        
        # 生成带时间戳的文件名
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_filename = f"{self.output_path}/video_{timestamp}.avi"
        
        # 初始化视频写入器
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self.writer = cv2.VideoWriter(
            self.current_filename, 
            fourcc, 
            self.fps,
            self.resolution
        )
        
        self.is_recording = True
        print(f"开始录制: {self.current_filename}")
        
        # 如果提供了第一帧，立即录制
        if frame is not None:
            self.write(frame)
    
    def write(self, frame):
        """将一帧写入视频文件"""
        if not self.is_recording or self.writer is None:
            return
            
        # 如果需要，调整帧大小
        if self.resolution:
            h, w = frame.shape[:2]
            if (w, h) != self.resolution:
                frame = cv2.resize(frame, self.resolution)
            
        self.writer.write(frame)
    
    def stop(self):
        """停止录制并释放资源"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        if self.writer:
            self.writer.release()
            self.writer = None
        print(f"录制停止: {self.current_filename}")
        
    def __del__(self):
        """确保对象销毁时释放资源"""
        self.stop()