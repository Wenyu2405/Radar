import time
import cv2
import numpy as np

index_table = {
    1: "R1",
    2: "R2", 
    3: "R3",
    4: "R4",
    5: "R7",
    101: "B1",
    102: "B2",
    103: "B3", 
    104: "B4",
    105: "B7"
}

def draw_information_ui(bar_list, camp, image, system_status=None):

    image.fill(0)
    
    img_height, img_width = image.shape[:2]
    
    cv2.line(image, (img_width//2, 0), (img_width//2, img_height), (100, 100, 100), 2)
    
    # 左侧
    draw_progress_section(image, bar_list, camp, 0, 0, img_width//2, img_height)
    
    # 右侧
    if system_status:
        draw_status_section(image, system_status, img_width//2, 0, img_width//2, img_height)
    
    return [0, 0, 0, 0, 0, 0] 

def draw_progress_section(image, bar_list, camp, x_start, y_start, width, height):
    cv2.putText(image, "Marking Progress", (x_start + 10, y_start + 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    progress_y_start = y_start + 50
    available_height = height - 60
    
    if camp == 'R':
        robot_ids = [1, 2, 3, 4, 5]  # R1, R2, R3, R4, R7
        robot_names = ["R1", "R2", "R3", "R4", "R7"]
    else:
        robot_ids = [101, 102, 103, 104, 105]  # B1, B2, B3, B4, B7  
        robot_names = ["B1", "B2", "B3", "B4", "B7"]
    
    num_robots = len(robot_ids)
    if num_robots == 0:
        return
        
    item_height = available_height // num_robots
    max_value = 120
    threshold = 100
    max_bar_width = width - 120 
    
    for i in range(min(len(bar_list), num_robots)):
        y_pos = progress_y_start + i * item_height + item_height // 2
        
        value = bar_list[i] if i < len(bar_list) else 0
        
        bar_width = int((value / max_value) * max_bar_width) if max_value > 0 else 0
        bar_width = max(0, min(bar_width, max_bar_width))
        
        if value >= threshold:
            if camp == 'R':
                color = (0, 255, 0)  #达到阈值
            else:
                color = (0, 255, 0)
            bar_height = 8
        else:
            if camp == 'R':
                color = (0, 100, 255)
            else:
                color = (255, 100, 0) 
            bar_height = 6
        
        cv2.putText(image, robot_names[i], (x_start + 10, y_pos + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.rectangle(image, (x_start + 50, y_pos - bar_height//2), 
                      (x_start + 50 + max_bar_width, y_pos + bar_height//2), 
                      (50, 50, 50), -1)
        
        if bar_width > 0:
            cv2.rectangle(image, (x_start + 50, y_pos - bar_height//2),
                          (x_start + 50 + bar_width, y_pos + bar_height//2),
                          color, -1)

        cv2.putText(image, f"{value}", (x_start + max_bar_width + 60, y_pos + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        
        threshold_x = x_start + 50 + int((threshold / max_value) * max_bar_width)
        cv2.line(image, (threshold_x, y_pos - bar_height//2 - 2),
                 (threshold_x, y_pos + bar_height//2 + 2), (255, 255, 255), 1)

def draw_status_section(image, system_status, x_start, y_start, width, height):
    cv2.putText(image, "System Status", (x_start + 10, y_start + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    status_y = y_start + 60
    line_height = 35
    
    status_items = [
        ("Team Color", system_status.get('team_color', 'Unknown'), get_team_color),
        ("Vulner Chances", f"{system_status.get('vulnerability_chances', -1)}", get_vulnerability_color),
        ("Vulner State", system_status.get('vulnerability_active', 'No'), get_vulnerability_active_color),
        ("Recording", system_status.get('recording_status', 'Unknown'), get_recording_color),
        ("Camera", system_status.get('camera_status', 'Unknown'), get_camera_color),
        ("Radar", system_status.get('radar_status', 'Unknown'), get_radar_color),
        ("FPS", f"{system_status.get('fps', 0):.1f}", get_fps_color),
    ]
    
    for i, (label, value, color_func) in enumerate(status_items):
        y_pos = status_y + i * line_height
        
        cv2.putText(image, f"{label}:", (x_start + 10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        
        color = color_func(value, system_status)
        
        if label in ["Team Color", "Vulner Chances", "Vulner State"]:
            x_offset = 200  
        else :
            x_offset = 140 
            
        cv2.putText(image, str(value), (x_start + x_offset, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

def get_team_color(value, status):
    if value == 'R':
        return (0, 0, 255)
    elif value == 'B':
        return (255, 0, 0)
    return (128, 128, 128) 

def get_vulnerability_color(value, status):
    try:
        chances = int(value)
        if chances > 0:
            return (0, 255, 0)
        elif chances == 0:
            return (0, 255, 255)
        else:
            return (128, 128, 128
    except:
        return (128, 128, 128)

def get_vulnerability_active_color(value, status):
    if value == "Active":
        return (0, 255, 0)  
    elif value == "Inactive":
        return (0, 255, 255) 
    return (128, 128, 128) 

def get_recording_color(value, status):
    if value == "Recording":
        return (0, 255, 0) 
    elif value == "Stopped":
        return (0, 255, 255) 
    return (128, 128, 128) 

def get_camera_color(value, status):
    if value == "Connected":
        return (0, 255, 0) 
    elif value == "Disconnected":
        return (0, 0, 255) 
    return (128, 128, 128) 

def get_radar_color(value, status):
    if value == "Connected":
        return (0, 255, 0) 
    elif value == "Disconnected":
        return (0, 0, 255)  
    return (128, 128, 128)  

def get_fps_color(value, status):
    try:
        fps = float(value)
        if fps >= 25:
            return (0, 255, 0) 
        elif fps >= 15:
            return (0, 255, 255)
        else:
            return (0, 0, 255) 
    except:
        return (128, 128, 128)
