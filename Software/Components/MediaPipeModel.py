import mediapipe as mp
import cv2
from .VirtualButtons import VirtualButtons

import serial

serial = serial.Serial(
    port='COM3',  # Replace with your ESP32's serial port
    baudrate=115200,
    timeout=1
)

class MediaPipe:
    def __init__(self, static_mode=False, max_hands=1, detection_conf=0.5, tracking_conf=0.5):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=static_mode,
            max_num_hands=max_hands,
            min_detection_confidence=detection_conf,
            min_tracking_confidence=tracking_conf
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # Initialize button configurations
        self.button_configs = [
            {'pos': (100, 100), 'label': 'LED ON', 'action': lambda: serial.write(b"LED_ON\n")},
            {'pos': (300, 100), 'label': 'LED OFF', 'action': lambda: serial.write(b"LED_OFF\n")}
        ]
        self.virtual_buttons = None

    def process_frame(self, frame):
        frame.flags.writeable = False
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        
        frame.flags.writeable = True
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # Always initialize virtual buttons with empty landmarks if none detected
        landmarks = []
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
                
                # Update landmarks if hands are detected
                landmarks = [(int(landmark.x * frame.shape[1]), int(landmark.y * frame.shape[0])) 
                           for landmark in hand_landmarks.landmark]
        
        # Create/update virtual buttons (always shown)
        self.virtual_buttons = VirtualButtons(frame, landmarks, self.button_configs)
        frame = self.virtual_buttons.draw_buttons()
        
        # Check for button presses if landmarks exist
        if landmarks:
            self.virtual_buttons.check_button_presses()
                
        return frame, results