import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
from rclpy.qos import qos_profile_sensor_data
import os
from enum import Enum

class State(Enum):
    searching = 1
    tracking = 2
    inspecting = 3
    idle = 4


class MachineFace(Node):
    def __init__(self):
        super().__init__('machine_face')
        self.start_time=time.time()
        self.last_speech_time=0.0
        self.speak("system initializing")
        time.sleep(0.5)
        # 1. SUBSCRIBE (Video)
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.process_image,
            qos_profile_sensor_data)
            
        # 2. PUBLISH (Movement)
        self.publisher_ = self.create_publisher(Twist, '/X3/cmd_vel', 10)
        
        # 3. CONVERT (Ros to OpenCV)
        self.br = CvBridge()
        self.speak("checking Points")
        time.sleep(0.5)
        # 4. LOAD THE BRAIN (Haar Cascade)
        # We load the XML file we just downloaded
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.last_speech_time=0.0
        self.last_seen_time = time.time()
        self.last_known_direction=1
        print("👤 MACHINE FACE ONLINE. LOOKING FOR HUMAN...")
        self.speak("callibrating motous")
        time.sleep(0.5)
        self.current_state =State.searching
        self.last_state_change=time.time()
        self.last_state=State.idle
        self.search_pattern_timer = 0
        self.last_snapshot_time = 0
        self.evidence_path=os.path.join(os.getcwd(),"evidence_locker")
        if not os.path.exists(self.evidence_path):
            os.makedirs(self.evidence_path)
            print("saved : {self.evidence_path} ")
        self.log_file=os.path.join(self.evidence_path,"mission_log.txt")
        with open(self.log_file,"w") as f :
            f.write(f"__SENTINEL__FIGHT__LOG__: {time.strftime('%Y-%m-%d %H:%M:%S')}--/n")

    def speak(self,text):
        if time.time()-self.last_speech_time>5.0:
            command=f'espeak -v en+f3 "{text}"&'
            os.system(command)
            self.last_speech_time=time.time()

    def log_event(self,message):
        timestamp=time.strftime("[%H:%M:%S}")
        entry= f"{timestamp} {message}/n"
        with open(self.log_file, "a") as f :
            f.write(entry)
        print(f"logged: {message}")


    def process_image(self, msg):
        # A. Get the image
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        # B. Convert to GRAYSCALE
        # Face detection works on light/shadow (Black & White), not color.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # C. DETECT FACES
        # scaleFactor=1.1: How much image size is reduced at each image scale
        # minNeighbors=5: How many neighbors each candidate rectangle should have to retain it
        faces = self.face_cascade.detectMultiScale(gray, 1.05, 2)
        
        cmd = Twist()
        current_time=time.time()
        # D. LOGIC LOOP
        if len(faces) > 0:
            
            self.current_state = State.tracking
            self.last_seen_time = time.time()
            self.speak("Target acquired")
            # We only care about the first face found (faces[0])
            (x, y, w, h) = faces[0]
            
            # Calculate Center
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            
            # --- MOVEMENT LOGIC (The P-Controller) ---
            
            # 1. YAW (Turn to Face)
            # Screen Center X = 320
            error_yaw = 320 - center_x
            if error_yaw>0:
                self.last_known_direction=1
            else:
                self.last_known_direction=-1
            cmd.angular.z = error_yaw * 0.002
            
            # 2. ALTITUDE (Fly to Eye Level)
            # Screen Center Y = 240
            error_alt = 240 - center_y
            cmd.linear.z = error_alt * 0.002
            
            # 3. DISTANCE (Personal Space)
            # If face width > 100, we are close.
            desired_width = 100
            error_dist = desired_width - w
            speed_x = error_dist * 0.005
            
            # Speed Cap
            if speed_x > 0.5: speed_x = 0.5
            if speed_x < -0.5: speed_x = -0.5
            cmd.linear.x = speed_x

            if abs(error_yaw)<20 and abs(error_dist)<20:
                self.current_state=State.inspecting
                if self.last_state != self.current_state:
                    self.log_event("TARGET ACQURIED: OPERATOR")
            if self.current_state==State.inspecting:
                cmd.linear.x=0.9
                self.speak("inspecting")
                if time.time()-self.last_snapshot_time>5:
                    filename=f"Intruder_{int(time.time())}.jpg"
                    full_path=os.path.join(self.evidence_path,filename)
                    cv2.imwrite(full_path, frame)
                    print(f"snapshot takena: {filename} ")
                    self.last_snapshot_time=time.time()
                    cv2.putText(frame,"saveing evidence",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                    if self.last_state != self.current_state:
                        self.log_event(f"EVIDENCE SAVED: {filename} ")
                if w>130:
                    self.current_state=State.tracking
            
            # --- VISUALS ---
            # Draw a Blue Box around the face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame,"Target:Operator",(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,0),2)
            # Draw line to center
            cv2.line(frame, (320, 240), (center_x, center_y), (0, 255, 255), 2)
            cv2.line(frame, (300,240), (340,240), (0,0,255), 1)
            cv2.line(frame, (320,220), (320,260), (0,0,255), 1)
            cv2.circle(frame,(320,240),20,(0,0,255),1)
            print(f"👁️  SEEN: Face Width {w} | Turn: {cmd.angular.z:.2f}")

        else:
            # NO FACE SEEN
            time_lost = time.time() - self.last_seen_time
            
            if time_lost > 2.0:
                if self.last_state != self.current_state:
                    self.log_event("PATROL_STARTED.")
                print("❓ LOST HUMAN... HOVERING." )
                self.speak("Target lost")
                #cv2.putText(frame,"status:scanning",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
                self.current_state = State.searching
                timer =current_time%42
                if time.time()-self.start_time<9.0:
                    print("hey")
                    cmd.linear.z=0.9
                elif timer<2:
                    #self.log_event("PATROL_STARTED.")
                    self.speak("sector clear ")
                    cmd.linear.x= 0.2
                    cmd.angular.z=0.0
                    cmd.linear.z=0.00
                    #if self.last_state != self.current_state:
                    #    self.log_event("PATROL_STARTED.")
                elif timer<9:
                    cmd.linear.x=0.0
                    cmd.angular.z=0.4*self.last_known_direction
                    cmd.linear.z=0.0
                elif timer<14:
                    cmd.linear.x=0.3
                    cmd.linear.z=0.0
                    cmd.angular.z=0.0
                elif timer<23:
                    cmd.angular.z=0.4*self.last_known_direction
                    cmd.linear.z=0.0
                elif timer<28:
                    cmd.linear.x=0.2
                    cmd.linear.z=0.0
                elif timer<37:
                    cmd.angular.z=0.4*self.last_known_direction
                    cmd.linear.z=0.0
                elif timer<42:
                    cmd.linear.x=0.2
                    cmd.linear.z=0.0
                else:
                    print("❓ LOST HUMAN... HOVERING." )
                   # self.speak("Target lost")
                    #cv2.putText(frame,"status:scanning",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
                # Stop moving if lost
                    cmd.linear.x = 0.0
                    cmd.linear.z = 0.0
                    cmd.angular.z = 0.0*self.last_known_direction
            else:
                 # Keep last command for 2 seconds (Momentum)
                 pass

        # E. EXECUTE & SHOW
        self.last_state = self.current_state
        cv2.rectangle(frame, (22,402), (100,419), (34,139,34), -1)
        cv2.rectangle(frame, (20,400), (120,420), (0,0,0), 2)
        #cv2.rectangle(frame, (22,402), (100,419), (34,139,34), -1)
        cv2.putText(frame, "PWR: 80%", (20,390), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,100,0), 2)
        timestamp=time.strftime("%H:%M:%S UTC")
        #cv2.putText(frame, timestamp, (480,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (75,0,00), 2)
        if time.time()%1.0>0.5:
            cv2.rectangle(frame,(0,0),(640,60),(30,30,30),-1)
            if self.current_state==State.searching:
                cv2.putText(frame,"WARNING: SEARCHING....",(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(60,20,220),3)
            elif self.current_state==State.tracking:
                cv2.putText(frame,"[LOCKED]",(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(34,139,34),3)
        cv2.putText(frame, timestamp, (480,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        self.publisher_.publish(cmd)
        cv2.imshow("MACHINE FACE (Identity Scan)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MachineFace()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

