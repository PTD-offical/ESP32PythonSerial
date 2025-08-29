import mediapipe as mp
import cv2
import numpy as np
import serial
import time

class MediaPipe:
    def __init__(self, static_mode=False, max_hands=1, detection_conf=0.5, tracking_conf=0.5, serial_port='COM3', baud_rate=115200):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=static_mode,
            max_num_hands=max_hands,
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.Distance = 0
        
        # Initialize serial connection with error handling
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for connection to establish
            print(f"Serial connection established on {serial_port}")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
        
    def process_frame(self, frame):
        frame.flags.writeable = False
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        frame.flags.writeable = True
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        if results.multi_hand_landmarks:
            for hand_world_landmarks in results.multi_hand_world_landmarks:
                Thumb_Tip = 4
                Index_Finger_Tip = 8
                
                Thumb_Tip_Coords = hand_world_landmarks.landmark[Thumb_Tip]
                Index_Tip_Coords = hand_world_landmarks.landmark[Index_Finger_Tip]
                
                Index_Tip_Coords_Value = np.array([Index_Tip_Coords.x, Index_Tip_Coords.y, Index_Tip_Coords.z])
                Thumb_Tip_Coords_Value = np.array([Thumb_Tip_Coords.x, Thumb_Tip_Coords.y, Thumb_Tip_Coords.z])
                
                self.Distance = np.linalg.norm(Index_Tip_Coords_Value - Thumb_Tip_Coords_Value)
                self.Distance = self.Distance * 100  # Convert to cm
                
                # Send distance data via serial
                self.send_distance_data()
                
                print(f"Distance between index and thumb tip: {self.Distance:.2f} cm")
                
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
        return frame, results, self.Distance
    
    def send_distance_data(self):
        """Send distance data to ESP32 via serial"""
        if self.ser is None:
            return
        
        try:
            # Option 1: Send simple commands
            if self.Distance <= 8.0:
                self.ser.write(b"LED_ON\n")
            else:
                self.ser.write(b"LED_OFF\n")
            
            # Option 2: Send the actual distance value (uncomment if needed)
            # distance_str = f"DISTANCE:{self.Distance:.2f}\n"
            # self.ser.write(distance_str.encode('utf-8'))
            
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
    
    def close_serial(self):
        """Close serial connection"""
        if self.ser:
            self.ser.close()
            print("Serial connection closed")