#!/usr/bin/env python3
import rospy
import os
import math
import speech_recognition as sr
from gtts import gTTS
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class HospitalRobot:
    def __init__(self):
        rospy.init_node('hospital_voice_control')
        
        # --- CẤU HÌNH ---
        self.target_x = 0.0
        self.target_y = 0.0
        self.is_moving = False
        
        # Vị trí hiện tại
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Cảm biến
        self.min_dist_front = 10.0

        # --- DỮ LIỆU PHÒNG (Sẽ điền ở Bước 2) ---
        # Lưu ý: Tọa độ này phải lấy từ bản đồ map.yaml của bạn
        self.locations = {
            "cấp cứu":   {"x": -8.38, "y": 11.4,  "msg": "Đang đến phòng cấp cứu"},
            "bác sĩ":    {"x": 5.0, "y": 2.5,  "msg": "Đang dẫn bạn gặp bác sĩ"},
            "dược":      {"x": 1.0, "y": -3.0, "msg": "Đang đi lấy thuốc"},
            "về trạm":   {"x": 0.0, "y": 0.0,  "msg": "Đang quay về vị trí sạc"},
            "vệ sinh":   {"x": 2.86, "y": 9.54, "msg": "Đang di chuyển đến phòng vệ sinh"}
        }

        # --- ROS ---
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.recognizer = sr.Recognizer()

    def speak(self, text):
        rospy.loginfo(f"Robot: {text}")
        try:
            tts = gTTS(text=text, lang='vi')
            tts.save("voice.mp3")
            os.system("mpg321 voice.mp3 --quiet")
        except: pass

    def listen(self):
        with sr.Microphone() as source:
            rospy.loginfo("Đang nghe...")
            try:
                audio = self.recognizer.listen(source, timeout=3, phrase_time_limit=3)
                text = self.recognizer.recognize_google(audio, language="vi-VN")
                return text.lower()
            except: return None

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        rot = msg.pose.pose.orientation
        (_, _, self.current_yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

    def scan_cb(self, msg):
        # 1. Chuyển đổi dữ liệu thô sang list để dễ xử lý
        ranges = list(msg.ranges)
        
        # 2. Xử lý giá trị inf (vô cực) -> Gán bằng số lớn (10m)
        # Để tránh việc tính toán min bị lỗi
        clean_ranges = [r if (r > 0.1 and r < 10.0) else 10.0 for r in ranges]
        
        # 3. Lấy góc nhìn phía trước
        # Giả sử LiDAR 360 độ: 
        # - Phía trước thường là: [đầu mảng] + [cuối mảng] (nếu 0 độ ở trước)
        # - HOẶC là khúc giữa (nếu 0 độ ở sau)
        # --> CÁCH AN TOÀN NHẤT: Quét một vùng rộng phía trước robot.
        
        # Cách hack: Lấy 1/6 số lượng tia ở giữa mảng (Front)
        mid = len(clean_ranges) // 2
        window = len(clean_ranges) // 6  # Quét góc rộng khoảng 60 độ trước mặt
        
        front_ranges = clean_ranges[mid - window : mid + window]
        
        if front_ranges:
            self.min_dist_front = min(front_ranges)
        else:
            self.min_dist_front = 10.0
            
        # DEBUG: In ra để biết robot đang thấy gì
        # Nếu để robot trước tường mà số này vẫn > 1.0 thì là Sai Index
        # print(f"Khoảng cách: {self.min_dist_front:.2f}")

    def run(self):
        rate = rospy.Rate(10)
        self.speak("Xin chào, bạn cần đi đâu?")
        
        while not rospy.is_shutdown():
            cmd_vel = Twist()

            if not self.is_moving:
                # Đứng yên đợi lệnh
                text = self.listen()
                if text:
                    rospy.loginfo(f"Nghe được: {text}")
                    for key, data in self.locations.items():
                        if key in text:
                            self.speak(data["msg"])
                            self.target_x = data["x"]
                            self.target_y = data["y"]
                            self.is_moving = True
                            break
            else:
                # Logic di chuyển & né vật cản
                dist = math.sqrt((self.target_x - self.current_x)**2 + 
                                 (self.target_y - self.current_y)**2)
                
                if dist < 0.3: # Đã đến đích
                    self.speak("Đã đến nơi.")
                    self.is_moving = False
                    cmd_vel.linear.x = 0.0
                elif self.min_dist_front < 0.6: # Gặp vật cản
                    rospy.logwarn("Né vật cản...")
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.5 # Quay trái
                else: # Đường thoáng -> Tiến về đích
                    angle = math.atan2(self.target_y - self.current_y, 
                                       self.target_x - self.current_x)
                    err = angle - self.current_yaw
                    # Chuẩn hóa góc
                    if err > 3.14: err -= 6.28
                    elif err < -3.14: err += 6.28
                    
                    cmd_vel.angular.z = 1.0 * err
                    if abs(err) < 0.3: cmd_vel.linear.x = 0.3
                
                self.pub_vel.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try: HospitalRobot().run()
    except: pass