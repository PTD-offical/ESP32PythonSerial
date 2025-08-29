from Components.VideoCapture import CameraCapture

if __name__ == "__main__":
    camera_id = int(input("Please enter your Camera-WebCam InputID (0 for default): "))
    window_name = input("Please enter your window Title Name: ")
    app = CameraCapture(camera_id, window_name)
    app.main_video_capture()