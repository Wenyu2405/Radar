#!/usr/bin/env python
# coding: utf-8

import threading
import numpy as np
import cv2
from dahua.ImageConvert import *
from dahua.MVSDK import *
import struct
import time
import datetime

g_cameraStatusUserInfo = b"statusInfo"

class BITMAPFILEHEADER(Structure):
    _fields_ = [
                ('bfType', c_ushort),
                ('bfSize', c_uint),
                ('bfReserved1', c_ushort),
                ('bfReserved2', c_ushort),
                ('bfOffBits', c_uint),
                ]
 
class BITMAPINFOHEADER(Structure):
    _fields_ = [
                ('biSize', c_uint),
                ('biWidth', c_int),
                ('biHeight', c_int),
                ('biPlanes', c_ushort),
                ('biBitCount', c_ushort),
                ('biCompression', c_uint),
                ('biSizeImage', c_uint),
                ('biXPelsPerMeter', c_int),
                ('biYPelsPerMeter', c_int),
                ('biClrUsed', c_uint),
                ('biClrImportant', c_uint),
                ] 

# 调色板，只有8bit及以下才需要
class RGBQUAD(Structure):
    _fields_ = [
                ('rgbBlue', c_ubyte),
                ('rgbGreen', c_ubyte),
                ('rgbRed', c_ubyte),
                ('rgbReserved', c_ubyte),
                ]

class DahuaCamera:
    def __init__(self):
        self.camera = None
        self.streamSource = None
        self.isConnected = False
        self.camera_index = 0  # Default to first camera
    
    def enumCameras(self):
        """枚举相机"""
        # 获取系统单例
        system = pointer(GENICAM_System())
        nRet = GENICAM_getSystemInstance(byref(system))
        if (nRet != 0):
            print("getSystemInstance fail!")
            return 0, None

        # 发现相机 
        cameraList = pointer(GENICAM_Camera()) 
        cameraCnt = c_uint()
        nRet = system.contents.discovery(system, byref(cameraList), byref(cameraCnt), c_int(GENICAM_EProtocolType.typeAll))
        if (nRet != 0):
            print("discovery fail!")
            return 0, None
        elif cameraCnt.value < 1:
            print("discovery no camera!")
            return 0, None
        else:
            print(f"Discovered {cameraCnt.value} Dahua cameras")
            return cameraCnt.value, cameraList
    
    def connect(self, camera_index=0):
        """连接指定索引的相机"""
        self.camera_index = camera_index
        
        # 枚举相机
        cameraCnt, cameraList = self.enumCameras()
        if cameraCnt == 0 or cameraList is None:
            print("No Dahua cameras found")
            return False
            
        if camera_index >= cameraCnt:
            print(f"Camera index {camera_index} is out of range (found {cameraCnt} cameras)")
            return False
            
        self.camera = cameraList[camera_index]
        
        # 连接相机
        nRet = self.camera.connect(self.camera, c_int(GENICAM_ECameraAccessPermission.accessPermissionControl))
        if (nRet != 0):
            print(f"Failed to connect to Dahua camera {camera_index}")
            return False
        else:
            print(f"Successfully connected to Dahua camera {camera_index}")
            self.isConnected = True
            
        # 注册相机连接状态回调
        self._subscribeCameraStatus()
        
        # 创建流对象
        streamSourceInfo = GENICAM_StreamSourceInfo()
        streamSourceInfo.channelId = 0
        streamSourceInfo.pCamera = pointer(self.camera)
        
        self.streamSource = pointer(GENICAM_StreamSource())
        nRet = GENICAM_createStreamSource(pointer(streamSourceInfo), byref(self.streamSource))
        if (nRet != 0):
            print("Failed to create StreamSource")
            return False
            
        # 设置连续采集模式
        self._setTriggerMode("Off")
        
        return True
    
    def _subscribeCameraStatus(self):
        """注册相机连接状态回调"""
        # 定义回调函数
        @connectCallBackEx
        def deviceLinkNotify(connectArg, linkInfo):
            if (EVType.offLine == connectArg.contents.m_event):
                print(f"Camera {self.camera_index} went offline")
                self.isConnected = False
            elif (EVType.onLine == connectArg.contents.m_event):
                print(f"Camera {self.camera_index} came online")
                self.isConnected = True
        
        # 保存回调函数引用，防止被垃圾回收
        self._deviceLinkNotify = deviceLinkNotify
        
        # 注册上下线通知
        eventSubscribe = pointer(GENICAM_EventSubscribe())
        eventSubscribeInfo = GENICAM_EventSubscribeInfo()
        eventSubscribeInfo.pCamera = pointer(self.camera)
        nRet = GENICAM_createEventSubscribe(byref(eventSubscribeInfo), byref(eventSubscribe))
        if (nRet != 0):
            print("Failed to create eventSubscribe")
            return False
        
        userInfo = g_cameraStatusUserInfo
        nRet = eventSubscribe.contents.subscribeConnectArgsEx(eventSubscribe, self._deviceLinkNotify, userInfo)
        if (nRet != 0):
            print("Failed to subscribeConnectArgsEx")
            eventSubscribe.contents.release(eventSubscribe)
            return False
        
        # 不再使用时，需释放相关资源
        eventSubscribe.contents.release(eventSubscribe)
        return True
    
    def _setTriggerMode(self, mode="Off"):
        """设置触发模式，默认为连续采集(Off)，可设置为软触发(On)"""
        # 如果相机未连接，则直接返回
        if not self.isConnected or self.camera is None:
            return False
            
        # 构造触发模式节点
        trigModeEnumNode = pointer(GENICAM_EnumNode())
        trigModeEnumNodeInfo = GENICAM_EnumNodeInfo()
        trigModeEnumNodeInfo.pCamera = pointer(self.camera)
        trigModeEnumNodeInfo.attrName = b"TriggerMode"
        nRet = GENICAM_createEnumNode(byref(trigModeEnumNodeInfo), byref(trigModeEnumNode))
        if (nRet != 0):
            print("Failed to create TriggerMode node")
            return False
            
        # 设置触发模式
        nRet = trigModeEnumNode.contents.setValueBySymbol(trigModeEnumNode, mode.encode())
        if (nRet != 0):
            print(f"Failed to set TriggerMode to {mode}")
            trigModeEnumNode.contents.release(trigModeEnumNode)
            return False
            
        # 释放资源
        trigModeEnumNode.contents.release(trigModeEnumNode)
        
        # 如果设置为软触发模式，还需要设置触发源
        if mode == "On":
            return self._setSoftTriggerSource()
            
        return True
    
    def _setSoftTriggerSource(self):
        """设置软触发源"""
        # 创建control节点
        acqCtrlInfo = GENICAM_AcquisitionControlInfo()
        acqCtrlInfo.pCamera = pointer(self.camera)
        acqCtrl = pointer(GENICAM_AcquisitionControl())
        nRet = GENICAM_createAcquisitionControl(pointer(acqCtrlInfo), byref(acqCtrl))
        if (nRet != 0):
            print("Failed to create AcquisitionControl")
            return False
        
        # 设置触发源为软触发
        trigSourceEnumNode = acqCtrl.contents.triggerSource(acqCtrl)
        nRet = trigSourceEnumNode.setValueBySymbol(byref(trigSourceEnumNode), b"Software")
        if (nRet != 0):
            print("Failed to set TriggerSource to Software")
            trigSourceEnumNode.release(byref(trigSourceEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return False
        
        # 释放资源
        trigSourceEnumNode.release(byref(trigSourceEnumNode))
        
        # 设置触发方式
        trigSelectorEnumNode = acqCtrl.contents.triggerSelector(acqCtrl)
        nRet = trigSelectorEnumNode.setValueBySymbol(byref(trigSelectorEnumNode), b"FrameStart")
        if (nRet != 0):
            print("Failed to set TriggerSelector to FrameStart")
            trigSelectorEnumNode.release(byref(trigSelectorEnumNode))
            acqCtrl.contents.release(acqCtrl)
            return False
         
        # 释放资源    
        trigSelectorEnumNode.release(byref(trigSelectorEnumNode))
        acqCtrl.contents.release(acqCtrl)
        
        return True
    
    def start_grabbing(self):
        """开始采集图像"""
        if not self.isConnected or self.streamSource is None:
            print("Camera not connected or stream source not created")
            return False
            
        # 开始采集
        nRet = self.streamSource.contents.startGrabbing(self.streamSource, c_ulonglong(0), 
                                               c_int(GENICAM_EGrabStrategy.grabStrartegySequential))
        if (nRet != 0):
            print("Failed to start grabbing")
            return False
            
        print(f"Camera {self.camera_index} started grabbing")
        return True
    
    def stop_grabbing(self):
        """停止采集图像"""
        if self.streamSource is not None:
            self.streamSource.contents.stopGrabbing(self.streamSource)
            print(f"Camera {self.camera_index} stopped grabbing")
    
    def get_frame(self, timeout_ms=1000):
        """获取一帧图像"""
        if not self.isConnected or self.streamSource is None:
            return None
            
        # 主动获取一帧图像
        frame = pointer(GENICAM_Frame())
        nRet = self.streamSource.contents.getFrame(self.streamSource, byref(frame), c_uint(timeout_ms))
        if (nRet != 0):
            print("Failed to get frame")
            return None
            
        try:
            # 获取图像大小
            imageSize = frame.contents.getImageSize(frame)
            buffAddr = frame.contents.getImage(frame)
            
            # 将裸数据拷贝出来
            frameBuff = create_string_buffer(imageSize)
            memmove(frameBuff, c_char_p(buffAddr), imageSize)
            
            # 设置转码参数
            convertParams = IMGCNV_SOpenParam()
            convertParams.dataSize = imageSize
            convertParams.height = frame.contents.getImageHeight(frame)
            convertParams.width = frame.contents.getImageWidth(frame)
            convertParams.paddingX = frame.contents.getImagePaddingX(frame)
            convertParams.paddingY = frame.contents.getImagePaddingY(frame)
            convertParams.pixelForamt = frame.contents.getImagePixelFormat(frame)
            
            # 创建RGB缓冲区
            rgbBuff = create_string_buffer(convertParams.height * convertParams.width * 3)
            rgbSize = c_int()
            
            # 转换为BGR24格式
            nRet = IMGCNV_ConvertToBGR24(cast(frameBuff, c_void_p), byref(convertParams), 
                                   cast(rgbBuff, c_void_p), byref(rgbSize))
            if (nRet != 0):
                print("Failed to convert to BGR24")
                return None
                
            # 转换为numpy数组
            np_array_bgr_flat = np.frombuffer(rgbBuff, dtype=np.uint8)
            cv_image = np_array_bgr_flat.reshape((convertParams.height, convertParams.width, 3))
            
            return cv_image
        finally:
            # 释放驱动图像缓存
            frame.contents.release(frame)
    
    def set_exposure_time(self, exposure_time):
        """设置曝光时间 (单位: 微秒)"""
        if not self.isConnected:
            return False
            
        # 创建浮点型节点
        exposureTimeNode = pointer(GENICAM_DoubleNode())
        exposureTimeNodeInfo = GENICAM_DoubleNodeInfo()
        exposureTimeNodeInfo.pCamera = pointer(self.camera)
        exposureTimeNodeInfo.attrName = b"ExposureTime"
        nRet = GENICAM_createDoubleNode(byref(exposureTimeNodeInfo), byref(exposureTimeNode))
        if (nRet != 0):
            print("Failed to create ExposureTime node")
            return False
            
        # 设置曝光值
        nRet = exposureTimeNode.contents.setValue(exposureTimeNode, c_double(exposure_time))
        if (nRet != 0):
            print(f"Failed to set ExposureTime to {exposure_time}")
            exposureTimeNode.contents.release(exposureTimeNode)
            return False
            
        # 释放资源
        exposureTimeNode.contents.release(exposureTimeNode)
        return True
    
    def set_gain(self, gain):
        """设置增益值"""
        if not self.isConnected:
            return False
            
        # 创建浮点型节点
        gainNode = pointer(GENICAM_DoubleNode())
        gainNodeInfo = GENICAM_DoubleNodeInfo()
        gainNodeInfo.pCamera = pointer(self.camera)
        gainNodeInfo.attrName = b"Gain"
        nRet = GENICAM_createDoubleNode(byref(gainNodeInfo), byref(gainNode))
        if (nRet != 0):
            print("Failed to create Gain node")
            return False
            
        # 设置增益值
        nRet = gainNode.contents.setValue(gainNode, c_double(gain))
        if (nRet != 0):
            print(f"Failed to set Gain to {gain}")
            gainNode.contents.release(gainNode)
            return False
            
        # 释放资源
        gainNode.contents.release(gainNode)
        return True
    
    def get_exposure_time(self):
        """获取当前曝光时间"""
        if not self.isConnected:
            return None
            
        exposureTimeNode = pointer(GENICAM_DoubleNode())
        exposureTimeNodeInfo = GENICAM_DoubleNodeInfo()
        exposureTimeNodeInfo.pCamera = pointer(self.camera)
        exposureTimeNodeInfo.attrName = b"ExposureTime"
        nRet = GENICAM_createDoubleNode(byref(exposureTimeNodeInfo), byref(exposureTimeNode))
        if (nRet != 0):
            print("Failed to create ExposureTime node")
            return None
            
        value = c_double()
        nRet = exposureTimeNode.contents.getValue(exposureTimeNode, byref(value))
        if (nRet != 0):
            print("Failed to get ExposureTime value")
            exposureTimeNode.contents.release(exposureTimeNode)
            return None
            
        exposureTimeNode.contents.release(exposureTimeNode)
        return value.value
    
    def get_gain(self):
        """获取当前增益值"""
        if not self.isConnected:
            return None
            
        gainNode = pointer(GENICAM_DoubleNode())
        gainNodeInfo = GENICAM_DoubleNodeInfo()
        gainNodeInfo.pCamera = pointer(self.camera)
        gainNodeInfo.attrName = b"Gain"
        nRet = GENICAM_createDoubleNode(byref(gainNodeInfo), byref(gainNode))
        if (nRet != 0):
            print("Failed to create Gain node")
            return None
            
        value = c_double()
        nRet = gainNode.contents.getValue(gainNode, byref(value))
        if (nRet != 0):
            print("Failed to get Gain value")
            gainNode.contents.release(gainNode)
            return None
            
        gainNode.contents.release(gainNode)
        return value.value
    
    def set_gamma(self, gamma_value):
        """设置伽马值"""
        if not self.isConnected:
            return False
        
    # 创建浮点型节点
        gammaNode = pointer(GENICAM_DoubleNode())
        gammaNodeInfo = GENICAM_DoubleNodeInfo()
        gammaNodeInfo.pCamera = pointer(self.camera)
        gammaNodeInfo.attrName = b"Gamma"  # 大华相机的伽马节点名称
        nRet = GENICAM_createDoubleNode(byref(gammaNodeInfo), byref(gammaNode))
        if (nRet != 0):
            print("Failed to create Gamma node")
            return False
        
    # 设置伽马值
        nRet = gammaNode.contents.setValue(gammaNode, c_double(gamma_value))
        if (nRet != 0):
            print(f"Failed to set Gamma to {gamma_value}")
            gammaNode.contents.release(gammaNode)
            return False
        
    # 释放资源
        gammaNode.contents.release(gammaNode)
        return True

    def get_gamma(self):
        """获取当前伽马值"""
        if not self.isConnected:
            return None
        
        gammaNode = pointer(GENICAM_DoubleNode())
        gammaNodeInfo = GENICAM_DoubleNodeInfo()
        gammaNodeInfo.pCamera = pointer(self.camera)
        gammaNodeInfo.attrName = b"Gamma"
        nRet = GENICAM_createDoubleNode(byref(gammaNodeInfo), byref(gammaNode))
        if (nRet != 0):
            print("Failed to create Gamma node")
            return None
        
        value = c_double()
        nRet = gammaNode.contents.getValue(gammaNode, byref(value))
        if (nRet != 0):
            print("Failed to get Gamma value")
            gammaNode.contents.release(gammaNode)
            return None
        
        gammaNode.contents.release(gammaNode)
        return value.value

    def set_frame_rate(self, frame_rate):
        """设置帧率"""
        if not self.isConnected:
            return False
            
        # 创建浮点型节点
        frameRateNode = pointer(GENICAM_DoubleNode())
        frameRateNodeInfo = GENICAM_DoubleNodeInfo()
        frameRateNodeInfo.pCamera = pointer(self.camera)
        frameRateNodeInfo.attrName = b"AcquisitionFrameRate"
        nRet = GENICAM_createDoubleNode(byref(frameRateNodeInfo), byref(frameRateNode))
        if (nRet != 0):
            print("Failed to create AcquisitionFrameRate node")
            return False
            
        # 设置帧率
        nRet = frameRateNode.contents.setValue(frameRateNode, c_double(frame_rate))
        if (nRet != 0):
            print(f"Failed to set AcquisitionFrameRate to {frame_rate}")
            frameRateNode.contents.release(frameRateNode)
            return False
            
        # 释放资源
        frameRateNode.contents.release(frameRateNode)
        return True
    
    def get_frame_rate(self):
        """获取当前帧率"""
        if not self.isConnected:
            return None
            
        frameRateNode = pointer(GENICAM_DoubleNode())
        frameRateNodeInfo = GENICAM_DoubleNodeInfo()
        frameRateNodeInfo.pCamera = pointer(self.camera)
        frameRateNodeInfo.attrName = b"AcquisitionFrameRate"
        nRet = GENICAM_createDoubleNode(byref(frameRateNodeInfo), byref(frameRateNode))
        if (nRet != 0):
            print("Failed to create AcquisitionFrameRate node")
            return None
            
        value = c_double()
        nRet = frameRateNode.contents.getValue(frameRateNode, byref(value))
        if (nRet != 0):
            print("Failed to get AcquisitionFrameRate value")
            frameRateNode.contents.release(frameRateNode)
            return None
            
        frameRateNode.contents.release(frameRateNode)
        return value.value
    
    def close(self):
        """关闭相机"""
        if self.streamSource is not None:
            self.stop_grabbing()
            self.streamSource.contents.release(self.streamSource)
            self.streamSource = None
            
        if self.isConnected and self.camera is not None:
            self.camera.disConnect(byref(self.camera))
            self.isConnected = False
            print(f"Camera {self.camera_index} disconnected")


