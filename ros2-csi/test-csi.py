import cv2
import numpy as np
import threading

class CSI_Camera:
    def __init__(self, capture_width=1280, capture_height=720, display_width=1280,
                 display_height=720, framerate=30, flip_method=0):
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.display_width = display_width
        self.display_height = display_height
        self.framerate = framerate
        self.flip_method = flip_method
        self.pipeline = self.build_gstreamer_pipeline()
        self.cap = None
        self.frame = None
        self.running = False

    def build_gstreamer_pipeline(self):
        """
        Creates a GStreamer pipeline string to capture video from the CSI camera.
        You can adjust various parameters such as resolution, framerate, and flip method.
        """
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int){}, height=(int){}, "
            "format=(string)NV12, framerate=(fraction){}/1 ! "
            "nvvidconv flip-method={} ! "
            "video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink".format(
                self.capture_width,
                self.capture_height,
                self.framerate,
                self.flip_method,
                self.display_width,
                self.display_height,
            )
        )

    def start_camera(self):
        """
        Initializes the video capture and starts the camera in a separate thread for continuous frame capture.
        """
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera. Please check if your camera is connected and the GStreamer pipeline is correct.")
        self.running = True
        self.capture_thread = threading.Thread(target=self.update_frame)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def update_frame(self):
        """
        Continuously captures frames from the camera in a separate thread to avoid blocking the main loop.
        """
        while self.running:
            ret_val, self.frame = self.cap.read()
            if not ret_val:
                print("Failed to capture frame.")
                self.stop_camera()
                break

    def stop_camera(self):
        """
        Stops the camera capture and releases resources.
        """
        self.running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

    def apply_filters(self):
        """
        Apply multiple filters (perspectives) to the camera stream.
        Returns a combined image with multiple perspectives in a grid.
        """
        if self.frame is None:
            return None

        # Original image
        original = self.frame

        # Grayscale filter
        grayscale = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        grayscale = cv2.cvtColor(grayscale, cv2.COLOR_GRAY2BGR)  # Convert back to BGR for stacking

        # Edge detection
        edges = cv2.Canny(self.frame, 100, 200)
        edges = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert to BGR for stacking

        # Sepia filter
        sepia_filter = np.array([[0.272, 0.534, 0.131],
                                 [0.349, 0.686, 0.168],
                                 [0.393, 0.769, 0.189]])
        sepia = cv2.transform(self.frame, sepia_filter)
        sepia = np.clip(sepia, 0, 255).astype(np.uint8)

        # Stack all filters together horizontally
        top_row = np.hstack((original, grayscale))
        bottom_row = np.hstack((edges, sepia))

        # Stack vertically to create a 2x2 grid
        combined = np.vstack((top_row, bottom_row))
        return combined

    def show_stream(self):
        """
        Displays the captured frames with different filters in a 2x2 grid layout.
        """
        if not self.cap.isOpened():
            raise RuntimeError("Cannot display stream, camera is not initialized.")
        
        while True:
            if self.frame is not None:
                # Apply filters and create the combined view
                combined_frame = self.apply_filters()

                # Display the combined frame
                cv2.imshow("CSI Camera Stream - Multiple Perspectives", combined_frame)

                # Exit the stream when 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop_camera()
                    break

if __name__ == "__main__":
    # Customize these settings as per your camera and resolution requirements
    capture_width = 1280
    capture_height = 720
    display_width = 640  # Reduced to make space for multiple views
    display_height = 360
    framerate = 30
    flip_method = 0  # 0 = no flip, 1 = flip horizontally, 2 = flip vertically, etc.

    try:
        # Create an instance of the CSI camera
        camera = CSI_Camera(
            capture_width=capture_width,
            capture_height=capture_height,
            display_width=display_width,
            display_height=display_height,
            framerate=framerate,
            flip_method=flip_method,
        )

        # Start camera stream
        camera.start_camera()

        # Show the camera stream with multiple perspectives
        camera.show_stream()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        camera.stop_camera()
