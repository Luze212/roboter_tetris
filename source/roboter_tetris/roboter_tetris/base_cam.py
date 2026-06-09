import cv2
import numpy as np
from cv_bridge import CvBridge
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo
import time

class BaseCam(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        self._bridge = CvBridge()
        
        # Parameters
        self.add_parameter(sr.Parameter("conveyor_z_dist", 865.0, sr.ParameterType.DOUBLE), "Z distance to conveyor in mm")
        self.add_parameter(sr.Parameter("min_obj_height", 15.0, sr.ParameterType.DOUBLE), "Min height of object in mm")
        self.add_parameter(sr.Parameter("max_obj_height_mm", 150.0, sr.ParameterType.DOUBLE), "Max height of object in mm")
        self.add_parameter(sr.Parameter("min_contour_area", 500.0, sr.ParameterType.DOUBLE), "Min contour area in px")
        
        # Inputs
        self._color_msg = Image()
        self.add_input("color_image", "_color_msg", Image)
        self._info_msg = CameraInfo()
        self.add_input("color_camera_info", "_info_msg", CameraInfo)
        self._depth_msg = Image()
        self.add_input("depth_image", "_depth_msg", Image)
        
        # Outputs
        self._objects_msg = Float64MultiArray()
        self.add_output("objects", "_objects_msg", Float64MultiArray)
        
        self._debug_msg = Image()
        self.add_output("debug_image", "_debug_msg", Image)
        
        # Tracker state
        self._tracks = []
        self._next_id = 1
        
    def on_step_callback(self):
        # We process only if we have recent messages
        if self._color_msg.width == 0 or self._depth_msg.width == 0 or len(self._info_msg.k) < 9:
            return
            
        try:
            color_img = self._bridge.imgmsg_to_cv2(self._color_msg, "bgr8")
            
            # Handle both 16UC1 (mm) and 32FC1 (m)
            depth_img = self._bridge.imgmsg_to_cv2(self._depth_msg, desired_encoding="passthrough")
            if depth_img.dtype == np.float32:
                depth_mm = depth_img * 1000.0
            else:
                depth_mm = depth_img.astype(np.float32)
                
            fx = self._info_msg.k[0]
            cx = self._info_msg.k[2]
            fy = self._info_msg.k[4]
            cy = self._info_msg.k[5]
            
            conveyor_z = self.get_parameter("conveyor_z_dist").get_value()
            min_h = self.get_parameter("min_obj_height").get_value()
            max_h = self.get_parameter("max_obj_height_mm").get_value()
            min_area = self.get_parameter("min_contour_area").get_value()
            
            # Mask generation (numpy vectorized)
            # Find points where depth is within the object height range on the conveyor
            valid_depth = (depth_mm > 0)
            z_condition_1 = depth_mm < (conveyor_z - min_h)
            z_condition_2 = depth_mm > (conveyor_z - max_h)
            mask = (valid_depth & z_condition_1 & z_condition_2).astype(np.uint8) * 255
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            debug_img = color_img.copy()
            hsv_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
            
            current_objects = []
            
            for cnt in contours:
                if cv2.contourArea(cnt) < min_area:
                    continue
                    
                # Reject border touching (margin = 2px)
                x, y, w, h = cv2.boundingRect(cnt)
                if x <= 2 or y <= 2 or (x+w) >= (mask.shape[1]-2) or (y+h) >= (mask.shape[0]-2):
                    continue
                    
                rrect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rrect)
                box = np.intp(box)
                
                # Estimate color using center pixel
                center_x, center_y = int(rrect[0][0]), int(rrect[0][1])
                color_id = 6 # Unknown
                if 0 <= center_y < hsv_img.shape[0] and 0 <= center_x < hsv_img.shape[1]:
                    h_val, s_val, v_val = hsv_img[center_y, center_x]
                    if v_val < 45: color_id = 5 # Black
                    elif s_val < 35 and v_val > 170: color_id = 4 # White
                    elif h_val <= 10 or h_val >= 170: color_id = 0 # Red
                    elif h_val <= 35: color_id = 1 # Yellow
                    elif h_val <= 85: color_id = 2 # Green
                    elif h_val <= 135: color_id = 3 # Blue
                
                # Height calculation (average z over the contour)
                c_mask = np.zeros_like(mask)
                cv2.drawContours(c_mask, [cnt], -1, 255, cv2.FILLED)
                points_depth = depth_mm[c_mask == 255]
                valid_points = points_depth[points_depth > 0]
                if len(valid_points) == 0:
                    continue
                    
                avg_z = np.mean(valid_points)
                obj_height = conveyor_z - avg_z + 20.0
                
                # 3D position of center
                center_z = avg_z
                center_x_3d = (center_x - cx) * center_z / fx
                center_y_3d = (center_y - cy) * center_z / fy
                
                # Width / Length from minAreaRect
                width_mm = rrect[1][0] * center_z / fx
                length_mm = rrect[1][1] * center_z / fy
                if length_mm < width_mm:
                    length_mm, width_mm = width_mm, length_mm
                    
                # Orientation
                p0 = box[0]
                p1 = box[1]
                dx = p1[0] - p0[0]
                dy = p1[1] - p0[1]
                orientation = np.arctan2(dy, dx)
                
                obj_dict = {
                    'x': center_x_3d,
                    'y': -center_y_3d, # Match C++ axis flip
                    'z': center_z + obj_height/2.0,
                    'width': width_mm,
                    'length': length_mm,
                    'height': obj_height,
                    'orientation': orientation,
                    'color': float(color_id),
                    'id': 0.0,
                    'vy': 0.0
                }
                current_objects.append(obj_dict)
                
                # Draw debug
                cv2.drawContours(debug_img, [box], 0, (0, 255, 0), 2)
                color_names = ["Red", "Yellow", "Green", "Blue", "White", "Black", "Unknown"]
                label_text = f"ID: {obj_dict['id']} {color_names[color_id]}"
                cv2.putText(debug_img, label_text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            # Nearest neighbor matching for ID persistence
            matched_objs = []
            for obj in current_objects:
                best_dist = 50.0 # max match distance mm
                best_track = None
                for track in self._tracks:
                    dist = np.sqrt((obj['x'] - track['x'])**2 + (obj['y'] - track['y'])**2 + (obj['z'] - track['z'])**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_track = track
                if best_track is not None:
                    obj['id'] = best_track['id']
                    obj['vy'] = obj['y'] - best_track['y']
                    best_track.update(obj)
                    matched_objs.append(best_track)
                else:
                    obj['id'] = float(self._next_id)
                    self._next_id += 1
                    matched_objs.append(obj)
                    
            self._tracks = matched_objs
            
            # Serialize to Float64MultiArray
            out_array = []
            for obj in self._tracks:
                out_array.extend([
                    obj['id'],
                    obj['color'],
                    obj['x'],
                    obj['y'],
                    obj['z'],
                    obj['orientation'],
                    obj['vy'],
                    obj['length'],
                    obj['width'],
                    obj['height']
                ])
            self._objects_msg.data = out_array
            
            self._debug_msg = self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
            
        except Exception as e:
            self.get_logger().error(f"Error in base_cam on_step_callback: {e}")
