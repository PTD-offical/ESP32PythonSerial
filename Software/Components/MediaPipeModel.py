import mediapipe as mp
import cv2
import numpy as np

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
                
                distance = np.linalg.norm(Index_Tip_Coords_Value - Thumb_Tip_Coords_Value)
                
                #?Note :.2f is a format specifier used within f-strings to limit the number of decimal places to two
                print(f"Distance between index and thumb tip: {distance:.2f} mm")
                
                # Used to generate the connections on your hands
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )
        return frame, results