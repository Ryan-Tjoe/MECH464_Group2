import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCamera:
    """
    Class to interface with the RealSense camera using the Intel RealSense SDK.
    """
    
    def __init__(self, width:int = 640, height:int = 480, fps:int = 30):
        """
        Initialize the RealSense camera pipeline.
        
        :param width: Width of the color and depth frames
        :type width: int
        :param height: Height of the color and depth frames
        :type height: int
        :param fps: Frame rate of the camera
        :type fps: int
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable both color and depth streams
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # Start the pipeline
        self.pipeline.start(self.config)
        
        # Align depth frame to color frame
        self.align = rs.align(rs.stream.color)
        
    def get_color_image(self) -> np.ndarray:
        """
        Capture a color frame as a NumPy array.

        :return: Color image as a NumPy array
        :rtype: numpy.ndarray
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            return None
        
        color_image = np.asanyarray(color_frame.get_data())
        
        return color_image
    
    def get_depth_image(self) -> np.ndarray:
        """
        Capture a depth frame as a NumPy array.

        :return: Depth image as a NumPy array
        :rtype: numpy.ndarray
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame:
            return None
        
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image
    
    def get_depth_at(self, x:int, y:int) -> float:
        """
        Get depth measurement at a specific (x, y) coordinate in millimeters.

        :param x: X-coordinate of the pixel
        :type x: int
        :param y: Y-coordinate of the pixel
        :type y: int
        :return: Depth measurement in millimeters
        :rtype: float
        
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        
        if not depth_frame:
            return None
        
        depth_value = depth_frame.get_distance(x, y)  # Distance in meters
        return depth_value * 1000  # Convert to millimeters
    
    def show_stream(self) -> None:
        """
        Display the real-time color and depth streams. Press 'q' to exit.
        """
        try:
            while True:
                color_image = self.get_color_image()
                depth_image = self.get_depth_image()
                if color_image is None or depth_image is None:
                    continue
                
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                combined = np.hstack((color_image, depth_colormap))
                
                cv2.imshow('RealSense Camera', combined)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.release()
    
    def release(self) -> None:
        """Stop the camera pipeline and close OpenCV windows."""
        self.pipeline.stop()
        cv2.destroyAllWindows()

    def __del__(self):
        """Destructor to ensure camera pipeline is stopped."""
        self.release()

# Example:
if __name__ == "__main__":
    camera = RealSenseCamera()
    camera.show_stream()
