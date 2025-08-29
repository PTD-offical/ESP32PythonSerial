import cv2
from .MediaPipeModel import MediaPipe
from .FPSCounter import FPSCalculator

class CameraCapture:
    def __init__(self, camera_id, window_name):
        self.camera_id = camera_id
        self.window_name = window_name
        self.hand_tracker = MediaPipe()
        self.fps_calculator = FPSCalculator()
        
    def main_video_capture(self):
        cap = cv2.VideoCapture(self.camera_id)
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                print("Ignoring empty frame")
                continue
            
            frame = cv2.flip(frame, 1)
            
            processed_frame, results, distance = self.hand_tracker.process_frame(frame)
            processed_frame = self.fps_calculator.calculate(processed_frame)
            
            cv2.imshow(self.window_name, processed_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
                
        cap.release()
        cv2.destroyAllWindows()