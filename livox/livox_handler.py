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



class LivoxHandler(Node):
    def __init__(self):
        super().__init__('livox_handler')
        
            # 添加深度图相关参数
        self.image_width = 640
        self.image_height = 480
        self.queue_size = 200  # 点云队列大小
        self.depth_queue = None
    
        # 假设一个简单的外参矩阵
        # self.lidar_to_camera_matrix = np.eye(4)  # 初始设为单位矩阵
        self.lidar_to_camera_matrix = np.array([
            [0.0, -1.0, 0.0, 0.0],    # 雷达Y轴(-Y方向)映射到相机X轴
            [0.0, 0.0, -1.0, 0.10],   # 雷达Z轴(-Z方向)映射到相机Y轴，加上10cm偏移
            [1.0, 0.0, 0.0, 0.0],     # 雷达X轴映射到相机Z轴
            [0.0, 0.0, 0.0, 1.0]      # 齐次坐标
        ])
        # 相机参数
        self.camera_matrix = np.array([
        [1532.053784058548, 0, 540.799185574042],
        [22.292672791962, 1532.884847669569, 431.563854499829],
        [0, 0, 1.0]
        ])
    
        self.distortion_coeffs = np.array([-0.7602051118827, 19.0400047729123, 
                                       -250.0791631461161, 0.001557386972865, 0.007094663902134])
    
        # 添加深度图存储
        self.depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
        self.integrated_depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
    

        # 创建一个订阅器，订阅雷达驱动自带的发布的点云数据
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',  
            self.point_cloud_callback,
            10)
        
        # 点云数据存储
        self.latest_point_cloud = None
        self.point_count = 0
        self.visualization_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        self.get_logger().info('Livox雷达处理器已启动，等待点云数据...')
    

    def point_cloud_callback(self, msg):
        """接收到点云数据的回调函数"""
        self.point_count = msg.width * msg.height
        self.latest_point_cloud = msg
        # self.get_logger().info(f'接收到点云数据: {self.point_count}个点')

        points=self.extract_points(msg)
        
        if len(points) > 0:
            
            # self.get_logger().info(f'提取了{len(points)}个有效点')
            # self.get_logger().info(f'点云范围: X({np.min(points[:,0]):.2f} to {np.max(points[:,0]):.2f}), ' +
            #                         f'Y({np.min(points[:,1]):.2f} to {np.max(points[:,1]):.2f}), ' +
            #                         f'Z({np.min(points[:,2]):.2f} to {np.max(points[:,2]):.2f})')
        

        # 生成深度图
            R = self.lidar_to_camera_matrix[:3, :3]  # 旋转矩阵
            T = self.lidar_to_camera_matrix[:3, 3]   # 平移向量
        
        # 生成当前帧深度图
            self.depth_map = self.generate_depth_map(points, self.camera_matrix, 
                                               self.distortion_coeffs, R, T)
        
        # 更新积分深度图
            self.integrated_depth_map = self.update_depth_queue(self.depth_map)
        
        # 可视化
            self.visualization_image = self.create_depth_visualization(self.integrated_depth_map)
    
        # self.get_logger().info(f'处理了点云数据: {self.point_count}个点')

       #更新可视化图像
        # self.visualization_image = self.create_point_cloud_visualization(msg)
        # 简单地更新可视化图像（可以自定义成复杂的，是例子例子）
        # self.visualization_image = self.create_simple_visualization()
        
    # def create_simple_visualization(self):
    #     """一个简单的点云数据可视化hhh测试用"""
    #     img = np.zeros((480, 640, 3), dtype=np.uint8)
        
    #     # 只是一个占位符，实际项目中应该处理点云数据并生成有意义的可视化
    #     # 在这里显示点云数量和接收状态
    #     cv2.putText(img, f"Points: {self.point_count}", (50, 50), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #     cv2.putText(img, "Livox data received", (50, 100), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
    #     return img

    def create_point_cloud_visualization(self, msg):
        """创建点云数据的深度图可视化"""
        # 创建深度图
        depth_img = np.ones((480, 640), dtype=np.float32) * np.inf
        
        # 从PointCloud2消息中提取点云数据
        points = self.extract_points(msg)
        
        if len(points) == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 简单的投影处理，仅用于示例
        for point in points:
            x, y, z = point[:3]  # 假设点云格式为(x,y,z,...)
            
            # 只处理前方的点，Livox点云坐标系通常为：X右，Y上，Z前
            if z > 0:
                # 简单的投影变换
                u = int((x / z * 10) + 150)  # 将点云x坐标投影到图像u坐标
                v = int((-y / z * 10) + 150)  # 将点云y坐标投影到图像v坐标
                
                # 确保坐标在图像范围内
                if 0 <= u < 640 and 0 <= v < 480:
                    # 记录最近的点的深度
                    depth_img[v, u] = min(depth_img[v, u], z)
        
        # 将无穷大的值设为0（没有点的地方）
        depth_img[depth_img == np.inf] = 0
        
        # 归一化并转换为8位整数以便可视化
        if np.max(depth_img) > 0:
            norm_depth = (depth_img / np.max(depth_img) * 255).astype(np.uint8)
        else:
            norm_depth = np.zeros_like(depth_img, dtype=np.uint8)
        
        # 使用伪彩色显示
        depth_img_color = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
        
        # 添加信息文本
        cv2.putText(depth_img_color, f"Points: {self.point_count}", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return depth_img_color
    
    def extract_points(self, msg):
        """从PointCloud2消息中提取点云数据"""
        points = []
        try:
            # 获取点云格式信息
            point_step = msg.point_step
            row_step = msg.row_step
            
            # 点云字节数组
            data = msg.data
            
            # 遍历每个点
            for i in range(0, len(data), point_step):
                if i + 12 <= len(data):  # 确保有足够的字节读取xyz (3 * 4 = 12字节)
                    # 解析x, y, z (假设是float32格式)
                    x = struct.unpack_from('f', data, i)[0]
                    y = struct.unpack_from('f', data, i + 4)[0]
                    z = struct.unpack_from('f', data, i + 8)[0]
                    points.append([x, y, z])
            
            return np.array(points)
        except Exception as e:
            self.get_logger().error(f'提取点云数据时出错: {str(e)}')
            return np.array([])

    def generate_depth_map_optimized(self, points, K, D, R, T):
    # 创建深度图
        depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
    
    # 坐标转换
        homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))
        RT = np.eye(4)
        RT[:3, :3] = R
        RT[:3, 3] = T
        transformed_points = (RT @ homogeneous_points.T).T[:, :3]
    
    # 筛选前方点
        front_mask = transformed_points[:, 2] > 0
        front_points = transformed_points[front_mask]
    
        if len(front_points) == 0:
            return depth_map
    
    # 一次性投影
        projected_points = cv2.projectPoints(front_points, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).astype(int)
    
    # 筛选有效点
        valid_mask = (projected_points[:, 0] >= 0) & (projected_points[:, 0] < self.image_width) & \
                    (projected_points[:, 1] >= 0) & (projected_points[:, 1] < self.image_height)
    
        valid_points = projected_points[valid_mask]
        valid_depths = front_points[valid_mask, 2]
    
    # 更新深度图
        for (v, u), z in zip(valid_points, valid_depths):
            depth_map[v, u] = min(depth_map[v, u], z)
    
        return depth_map           

    def generate_depth_map(self, points, K, D, R, T):
        """
        生成深度图
    
       参数:
      points: 点云数据
        K: 相机内参矩阵
        D: 畸变系数
        R: 雷达到相机的旋转矩阵
        T: 雷达到相机的平移向量
    
        返回:
        depth_map: 生成的深度图
        """
    # 创建深度图，初始化为无穷大
        depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
    
    # 将雷达坐标转换到相机坐标系
    # [R|T]变换矩阵
        RT = np.eye(4)
        RT[:3, :3] = R
        RT[:3, 3] = T.reshape(-1)
    
    # 转换点云坐标
        homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))
        transformed_points = (RT @ homogeneous_points.T).T[:, :3]
    
    # 添加调试信息
        points_in_front = np.sum(transformed_points[:, 2] > 0)
        # self.get_logger().info(f'相机前方的点数: {points_in_front}')
    
        # if points_in_front > 0:
        #     self.get_logger().info(f'前方点云范围: X({np.min(transformed_points[transformed_points[:, 2] > 0, 0]):.2f} to {np.max(transformed_points[transformed_points[:, 2] > 0, 0]):.2f}), ' +
        #                         f'Y({np.min(transformed_points[transformed_points[:, 2] > 0, 1]):.2f} to {np.max(transformed_points[transformed_points[:, 2] > 0, 1]):.2f}), ' +
        #                         f'Z({np.min(transformed_points[transformed_points[:, 2] > 0, 2]):.2f} to {np.max(transformed_points[transformed_points[:, 2] > 0, 2]):.2f})')
    
    # 投影计数
        projected_count = 0

    # 投影到图像平面
        for point in transformed_points:
            if point[2] <= 0:  # 背后的点不处理
                continue
            
        # 相机投影
            uv = K @ point.reshape(3, 1)
            u, v = int(uv[0] / uv[2]), int(uv[1] / uv[2])
        
        # 确保在图像范围内
            if 0 <= u < self.image_width and 0 <= v < self.image_height:
            # 取最小深度值（最近的点）
                depth_map[v, u] = min(depth_map[v, u], point[2])
                projected_count += 1

        # self.get_logger().info(f'成功投影到图像的点数: {projected_count}')

        return depth_map

    # def generate_depth_map(self, points, K, D, R, T):
    #     """生成深度图 - 优化版本"""
    #     # 创建深度图
    #     depth_map = np.ones((self.image_height, self.image_width), dtype=np.float32) * np.inf
    
    # # 坐标转换 - 一次性处理所有点
    #     homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))
    #     RT = np.eye(4)
    #     RT[:3, :3] = R
    #     RT[:3, 3] = T
    #     transformed_points = (RT @ homogeneous_points.T).T[:, :3]
    
    # # 筛选前方点
    #     front_mask = transformed_points[:, 2] > 0
    #     front_points = transformed_points[front_mask]
    
    #     if len(front_points) == 0:
    #         return depth_map
    
    # # 添加调试信息
    #     points_in_front = len(front_points)
    #     self.get_logger().info(f'相机前方的点数: {points_in_front}')
    
    #     if points_in_front > 0:
    #         self.get_logger().info(f'前方点云范围: X({np.min(front_points[:, 0]):.2f} to {np.max(front_points[:, 0]):.2f}), ' +
    #                            f'Y({np.min(front_points[:, 1]):.2f} to {np.max(front_points[:, 1]):.2f}), ' +
    #                            f'Z({np.min(front_points[:, 2]):.2f} to {np.max(front_points[:, 2]):.2f})')
    
    # # 一次性投影
    # # 由于OpenCV的projectPoints函数需要输入的点为nx3的形状，且需要旋转向量和平移向量
    # # 这里我们使用零旋转和零平移，因为前面已经进行了转换
    #     projected_points = cv2.projectPoints(front_points, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).astype(int)
    
    # # 筛选有效点
    #     valid_mask = (projected_points[:, 0] >= 0) & (projected_points[:, 0] < self.image_width) & \
    #              (projected_points[:, 1] >= 0) & (projected_points[:, 1] < self.image_height)
    
    #     valid_points = projected_points[valid_mask]
    #     valid_depths = front_points[valid_mask, 2]
    
    # # 更新深度图
    #     for (u, v), z in zip(valid_points, valid_depths):
    #         depth_map[v, u] = min(depth_map[v, u], z)
    
    #     self.get_logger().info(f'成功投影到图像的点数: {len(valid_points)}')
    
    #     return depth_map

    def estimate_armor_depth_roi(self, armor_bbox, depth_map):
        """
    估计装甲板的深度值，使用扩展的ROI区域
    
    参数:
    armor_bbox: 装甲板边界框 [x, y, w, h]
    depth_map: 深度图
    
    返回:
    平均深度值
        """
        x, y, w, h = armor_bbox
    # 扩大ROI区域
        x_min = max(0, x - w//2)
        y_min = max(0, y - h//2)
        x_max = min(depth_map.shape[1], x + w + w//2)
        y_max = min(depth_map.shape[0], y + h + h//2)
    
        roi = depth_map[y_min:y_max, x_min:x_max]
        valid_depths = roi[roi < np.inf]
    
        if len(valid_depths) > 0:
            return np.mean(valid_depths)
        else:
            return None

    def update_depth_queue(self, new_depth_map):
        """
        使用队列积分更新深度图
        """
        if self.depth_queue is None:
            self.depth_queue = collections.deque(maxlen=self.queue_size)
    
    # 将新深度图加入队列
        self.depth_queue.append(new_depth_map)
    
    # 合并深度图（取最小值）
        integrated_depth = np.ones_like(new_depth_map) * np.inf
        for depth in self.depth_queue:
            integrated_depth = np.minimum(integrated_depth, depth)
    
        return integrated_depth
   

    def create_depth_visualization(self, depth_map):
        """创建深度图可视化"""
        # 创建一个彩色深度图
        depth_vis = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # 找出有效深度值
        valid_mask = depth_map < np.inf
        valid_count = np.sum(valid_mask)



        if np.any(valid_mask):
            # 归一化深度值
            min_depth = np.min(depth_map[valid_mask])
            max_depth = np.max(depth_map[valid_mask])
            
            # 防止除以零
            depth_range = max_depth - min_depth
            if depth_range > 0:
                normalized_depth = np.zeros_like(depth_map)
                normalized_depth[valid_mask] = (depth_map[valid_mask] - min_depth) / depth_range
                
                # 转换为uint8并应用颜色映射
                depth_uint8 = (normalized_depth * 255).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
                
                # 将无效区域设为黑色
                depth_vis[~valid_mask] = [0, 0, 0]
        
        # 添加一些文本信息
        cv2.putText(depth_vis, f"Points: {self.point_count}", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return depth_vis

    # def create_depth_visualization(self, depth_map):
    #     """创建深度图可视化"""
    # # 创建一个彩色深度图
    #     depth_vis = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
    
    # # 找出有效深度值
    #     valid_mask = depth_map < np.inf
    #     valid_count = np.sum(valid_mask)
    
    # # 添加调试信息到图像
    #     cv2.putText(depth_vis, f"Points: {self.point_count}", (20, 30), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    #     cv2.putText(depth_vis, f"Valid depth pixels: {valid_count}", (20, 60), 
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
    
    #     if valid_count > 0:
    #     # 归一化深度值
    #         min_depth = np.min(depth_map[valid_mask])
    #         max_depth = np.max(depth_map[valid_mask])
        
    #     # 添加深度范围信息
    #         cv2.putText(depth_vis, f"Depth range: {min_depth:.2f}m - {max_depth:.2f}m", (20, 90), 
    #                     cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
    #     # 防止除以零
    #         depth_range = max_depth - min_depth
    #         if depth_range > 0:
    #             normalized_depth = np.zeros_like(depth_map)
    #             normalized_depth[valid_mask] = (depth_map[valid_mask] - min_depth) / depth_range
            
    #         # 转换为uint8并应用颜色映射
    #             depth_uint8 = (normalized_depth * 255).astype(np.uint8)
            
    #         # 使用高斯模糊填充空隙
    #             depth_uint8 = cv2.GaussianBlur(depth_uint8, (5, 5), 0)
            
    #             depth_vis = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
            
    #     return depth_vis

    def get_visualization(self):
        """获取可视化图像"""
        return self.visualization_image.copy()
    
    def is_receiving_data(self):
        """检查是否正在接收数据"""
        return self.latest_point_cloud is not None

