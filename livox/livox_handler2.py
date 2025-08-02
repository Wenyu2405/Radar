import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import cv2
import struct 
import collections 

class DepthQueue:
    def __init__(self, capacity, size, K, D, E):
        """
        用队列关系储存点云
        
        :param capacity: 深度队列的最大长度
        :param size: 图像大小 [W,H]
        :param K: 相机内参
        :param D: 畸变系数
        :param E: 雷达到相机外参
        """
        self.capacity = capacity
        self.size = size
        self.depth = np.ones((size[1], size[0]), dtype=np.float32) * np.inf
        self.queue = collections.deque(maxlen=capacity)
        self.K = K
        self.D = D
        self.E = E
        
        # 从E矩阵获取旋转向量和平移向量
        self.rvec = cv2.Rodrigues(E[:3, :3])[0]
        self.tvec = E[:3, 3]
        
        # 初始化标志，用于判断队列是否已经开始接收数据
        self.init_flag = False
        
    def push_back(self, pc):
        """
        添加点云数据并更新深度图
        
        :param pc: 点云数据，形状为 (N, 3)
        """
        # 当队列为空时，说明该类正在被初始化
        if len(self.queue) == 0:
            self.init_flag = True
            
        # 坐标转换：从雷达坐标系转换到相机坐标系
        homogeneous_points = np.hstack((pc, np.ones((pc.shape[0], 1))))
        transformed_points = (self.E @ homogeneous_points.T).T
        depths = transformed_points[:, 2]  # Z坐标表示深度
        
        # 投影到图像平面
        projected_points = cv2.projectPoints(pc, self.rvec, self.tvec, self.K, self.D)[0].reshape(-1, 2).astype(np.int32)
        
        # 筛选在图像范围内的点
        inside_mask = (projected_points[:, 0] >= 0) & (projected_points[:, 0] < self.size[0]) & \
                      (projected_points[:, 1] >= 0) & (projected_points[:, 1] < self.size[1])
        valid_points = projected_points[inside_mask]
        valid_depths = depths[inside_mask]
        
        # 将当前帧投影点加入队列
        self.queue.append(valid_points)
        
        # 如果队列已满，移除最旧的点并清除其深度信息
        if len(self.queue) == self.capacity:
            old_points = self.queue.popleft()
            for point in old_points:
                self.depth[point[1], point[0]] = np.inf
        
        # 更新深度图，采用取最小值的方式处理遮挡关系
        for point, depth in zip(valid_points, valid_depths):
            x, y = point
            if depth < self.depth[y, x]:
                self.depth[y, x] = depth
    
    def depth_detect_refine(self, r):
        """
        优化的装甲板深度检测
        
        :param r: 装甲板边界框 [x, y, w, h]
        :return: 归一化相机坐标系中的坐标加深度 [x0, y0, z]
        """
        center = np.float32([r[0]+r[2]/2, r[1]+r[3]/2])
        
        # 采用以中心点为基准点扩大一倍的装甲板框作为ROI
        area = self.depth[int(max(0, center[1]-r[3])):int(min(center[1]+r[3], self.size[1]-1)),
                         int(max(center[0]-r[2], 0)):int(min(center[0]+r[2], self.size[0]-1))]
        
        # 计算ROI区域的平均深度，若全为inf则返回nan
        z = np.nanmean(area[area < np.inf]) if np.any(area < np.inf) else np.nan
        
        # 获取归一化相机坐标
        normalized_coords = cv2.undistortPoints(center.reshape(1, 1, 2), self.K, self.D).reshape(-1)
        
        return np.concatenate([normalized_coords, np.array([z])], axis=0)
    
    def detect_depth(self, rects):
        """
        批量检测多个装甲板的深度
        
        :param rects: 装甲板边界框列表，每个元素格式为 [x, y, w, h]
        :return: 装甲板位置数组，每行为 [x0, y0, z]
        """
        if len(rects) == 0:
            return []
        
        results = []
        for rect in rects:
            results.append(self.depth_detect_refine(rect))
            
        return np.stack(results, axis=0)


class LivoxHandler(Node):
    def __init__(self):
        super().__init__('livox_handler')
        
        # 图像和点云参数
        self.image_width = 640
        self.image_height = 480
        self.queue_size = 200
        
        # 相机参数
        self.camera_matrix = np.array([
            [1532.053784058548, 0, 540.799185574042],
            [22.292672791962, 1532.884847669569, 431.563854499829],
            [0, 0, 1.0]
        ])
        
        self.distortion_coeffs = np.array([-0.7602051118827, 19.0400047729123, 
                                       -250.0791631461161, 0.001557386972865, 0.007094663902134])
        
        # 雷达到相机的外参矩阵（需要通过标定获得更准确的值）
        self.lidar_to_camera_matrix = np.array([
            [0.0, -1.0, 0.0, 0.0],    # 雷达Y轴(-Y方向)映射到相机X轴
            [0.0, 0.0, -1.0, 0.10],   # 雷达Z轴(-Z方向)映射到相机Y轴，加上10cm偏移
            [1.0, 0.0, 0.0, 0.0],     # 雷达X轴映射到相机Z轴
            [0.0, 0.0, 0.0, 1.0]      # 齐次坐标
        ])
        
        # 初始化深度队列
        self.depth_queue = DepthQueue(
            capacity=self.queue_size,
            size=[self.image_width, self.image_height],
            K=self.camera_matrix,
            D=self.distortion_coeffs,
            E=self.lidar_to_camera_matrix
        )
        
        # 深度图和可视化
        self.depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
        self.integrated_depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
        self.visualization_image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # 点云数据
        self.latest_point_cloud = None
        self.point_count = 0
        
        # 相机位姿（从相机到世界坐标系）
        self.camera_pose_set = False
        self.rvec = None
        self.tvec = None
        self.camera_to_world_matrix = np.eye(4)
        self.camera_position = np.zeros(3)
        
        # 创建ROS订阅器
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',  
            self.point_cloud_callback,
            10)
        
        self.get_logger().info('Livox雷达处理器已启动，等待点云数据...')
    
    def point_cloud_callback(self, msg):
        """接收到点云数据的回调函数"""
        self.point_count = msg.width * msg.height
        self.latest_point_cloud = msg
        
        # 提取点云数据
        points = self.extract_points(msg)
        
        if len(points) > 0:
            # 使用深度队列处理点云
            self.depth_queue.push_back(points)
            
            # 更新深度图
            self.depth_map = self.depth_queue.depth.copy()
            
            # 更新积分深度图（可选，看具体需求）
            self.integrated_depth_map = self.update_integrated_depth_map(self.depth_map)
            
            # 更新可视化
            self.visualization_image = self.create_depth_visualization(self.integrated_depth_map)
            
            # self.get_logger().info(f'处理了点云数据: {self.point_count}个点')
    
    def extract_points(self, msg):
        """从PointCloud2消息中提取点云数据"""
        # 这部分保持原样，已经实现得很好
        points = []
        try:
            point_step = msg.point_step
            row_step = msg.row_step
            data = msg.data
            
            for i in range(0, len(data), point_step):
                if i + 12 <= len(data):
                    x = struct.unpack_from('f', data, i)[0]
                    y = struct.unpack_from('f', data, i + 4)[0]
                    z = struct.unpack_from('f', data, i + 8)[0]
                    
                    # 可以添加一些基本的过滤
                    if np.sqrt(x*x + y*y + z*z) > 0.4:  # 过滤近距离点
                        points.append([x, y, z])
            
            return np.array(points)
        except Exception as e:
            self.get_logger().error(f'提取点云数据时出错: {str(e)}')
            return np.array([])
    
    def update_integrated_depth_map(self, depth_map):
        """
        更新积分深度图，可用于平滑或累积多帧深度信息
        """
        # 简单实现，直接返回当前深度图
        # 如果需要真正的积分，可以维护一个深度图历史队列
        return depth_map
    
    def set_camera_pose(self, rvec, tvec):
        """
        设置相机位姿（从相机到世界坐标系）
        
        :param rvec: 旋转向量
        :param tvec: 平移向量
        """
        self.rvec = rvec
        self.tvec = tvec
        self.camera_pose_set = True
        
        # 计算变换矩阵
        R, _ = cv2.Rodrigues(rvec)
        self.camera_to_world_matrix = np.eye(4)
        self.camera_to_world_matrix[:3, :3] = R
        self.camera_to_world_matrix[:3, 3] = tvec.reshape(-1)
        
        # 计算相机位置
        self.camera_position = -R.T @ tvec.reshape(-1)
        
        self.get_logger().info(f'已设置相机位姿，相机位置: {self.camera_position}')
    
    def camera_to_world(self, camera_point):
        """
        将相机坐标系中的点转换到世界坐标系
        
        :param camera_point: 相机坐标系中的点 [x, y, z]
        :return: 世界坐标系中的点 [x, y, z]
        """
        if not self.camera_pose_set:
            return camera_point
        
        homogeneous_point = np.append(camera_point, 1)
        world_point = self.camera_to_world_matrix @ homogeneous_point
        return world_point[:3]
    
    def adjust_z_axis(self, current_position, previous_position, threshold=0.2):
        """
        处理Z轴突变，类似上交的实现
        
        :param current_position: 当前位置 [x, y, z]
        :param previous_position: 上一帧位置 [x, y, z]
        :param threshold: Z轴突变阈值
        :return: 调整后的位置 [x, y, z]
        """
        if previous_position is None:
            return current_position
        
        # 检测Z轴突变
        z_diff = current_position[2] - previous_position[2]
        
        if abs(z_diff) > threshold:
            # 仅当前一帧为地面高度且发生向上突变时调整
            if previous_position[2] < 100:  # 假设地面阈值为100mm
                # 计算从相机到当前位置的方向向量
                direction = current_position - self.camera_position
                
                # 计算射线参数t
                t = (previous_position[2] - self.camera_position[2]) / direction[2]
                
                # 计算调整后的位置
                adjusted_position = self.camera_position + t * direction
                
                return adjusted_position
        
        return current_position
    
    def estimate_armor_depth(self, armor_bbox):
        """
        估计装甲板深度，调用depth_queue的方法
        
        :param armor_bbox: 装甲板边界框 [x, y, w, h]
        :return: 归一化相机坐标和深度 [x0, y0, z]
        """
        return self.depth_queue.depth_detect_refine(armor_bbox)
    
    def detect_armors_depth(self, armors_bbox):
        """
        批量检测多个装甲板的深度
        
        :param armors_bbox: 装甲板边界框列表
        :return: 装甲板位置数组
        """
        return self.depth_queue.detect_depth(armors_bbox)
    
    def create_depth_visualization(self, depth_map):
        """创建深度图可视化"""
        # 与原始方法类似，但可以增加更多信息
        depth_vis = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        valid_mask = depth_map < np.inf
        valid_count = np.sum(valid_mask)
        
        if np.any(valid_mask):
            # 归一化深度值
            min_depth = np.min(depth_map[valid_mask])
            max_depth = np.max(depth_map[valid_mask])
            
            depth_range = max_depth - min_depth
            if depth_range > 0:
                normalized_depth = np.zeros_like(depth_map)
                normalized_depth[valid_mask] = (depth_map[valid_mask] - min_depth) / depth_range
                
                depth_uint8 = (normalized_depth * 255).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
                
                # 将无效区域设为黑色
                depth_vis[~valid_mask] = [0, 0, 0]
        
        # 添加信息文本
        cv2.putText(depth_vis, f"Points: {self.point_count}", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.camera_pose_set:
            cv2.putText(depth_vis, "Camera Pose: Set", (20, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(depth_vis, "Camera Pose: Not Set", (20, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return depth_vis
    
    def get_visualization(self):
        """获取可视化图像"""
        return self.visualization_image.copy()
    
    def is_receiving_data(self):
        """检查是否正在接收数据"""
        return self.latest_point_cloud is not None and self.depth_queue.init_flag