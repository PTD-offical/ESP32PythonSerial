import time
import cv2

class FPSCalculator:
    def __init__(self):
        self.previous_time = 0
        
    def calculate(self, frame):
        current_time = time.time()
        if self.previous_time != 0:
            fps = int(1 / (current_time - self.previous_time))
            cv2.putText(frame, f"FPS: {fps}", (0, 50), 
                       cv2.FONT_HERSHEY_COMPLEX, 2, (255, 255, 0), 2)
        self.previous_time = current_time
        return frame 