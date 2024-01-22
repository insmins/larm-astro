#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import pyrealsense2 as rs
import signal, time, numpy as np, math
import sys, cv2
from cv_bridge import CvBridge


class CameraDriver(Node):

    def __init__(self, name="cam_driver", timerFreq = 1/60.0, isRos = True):
        if isRos:
            super().__init__(name) # Create the node

            # Initialize publishers:
            self.init_publishers(timerFreq)

        # Initialize camera
        self._init_camera()


    def init_publishers(self, timerFreq : float):
        """
        Initialize Publishers
        """
        self._rgb_publisher = self.create_publisher(Image, 'cam_rgb', 10)
        self._depth_publisher = self.create_publisher(Image, 'cam_depth', 10)
        self._infra_publisher_1 = self.create_publisher(Image, 'cam_infra_1',10)
        self._infra_publisher_2 = self.create_publisher(Image, 'cam_infra_2',10)

        # Initialize a clock for the publisher
        self.create_timer(timerFreq, self.publish_imgs)

    def _init_camera(self):
        """
        Initialize camera
        """
        ## Configure depth and color streams
        self._pipeline = rs.pipeline()
        self._config = rs.config()
        self._colorizer = rs.colorizer()

        self._align_to = rs.stream.color
        self._align = rs.align(self._align_to)
        self._color_info=(0, 0, 255)

        self._cam_width = 848
        self._cam_height = 480

        ## Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self._pipeline)
        pipeline_profile = self._config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        print( f"Connect: {device_product_line}" )
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera required !!!")
            exit(0)

        ## Configure stream width, height, format and frequency
        self._config.enable_stream(rs.stream.color, width=int(self._cam_width), height=int(self._cam_height), format=rs.format.bgr8, framerate=60)
        self._config.enable_stream(rs.stream.depth, width=int(self._cam_width), height=int(self._cam_height), format=rs.format.z16, framerate=60)
        self._config.enable_stream(rs.stream.infrared, 1, width=int(self._cam_width), height=int(self._cam_height), format=rs.format.y8, framerate=60)
        self._config.enable_stream(rs.stream.infrared, 2, width=int(self._cam_width), height=int(self._cam_height), format=rs.format.y8, framerate=60)

        ## Start the acquisition    
        self._pipeline.start(self._config)

    def _update_imgs(self):
        """
        Update self images parameters
        self._color_frame, self._depth_dist_frame, self._infra_frame_1, self._infra_frame_2
        """

        # Get frames
        frames = self._pipeline.wait_for_frames()
        
        # Split frames
        aligned_frames =  self._align.process(frames)

        #self._color_frame = frames.first(rs.stream.color)
        self._color_frame = aligned_frames.get_color_frame()
        self._depth_dist_frame = aligned_frames.get_depth_frame()
        self._infra_frame_1 = frames.get_infrared_frame(1)
        self._infra_frame_2 = frames.get_infrared_frame(2)

        return None if (self._color_frame and self._depth_dist_frame and self._infra_frame_1 and self._infra_frame_2) is None else 1


    def read_imgs(self):
        """
        Read and convert images to ROS - only to publish ros images
        """
        self._update_imgs()

        if not (self._depth_dist_frame and self._color_frame and self._infra_frame_1 and self._infra_frame_2):
            return
        

        # Convert images to numpy arrays
        color_image = np.asanyarray(self._color_frame.get_data())
        infra_image_1 = np.asanyarray(self._infra_frame_1.get_data())
        infra_image_2 = np.asanyarray(self._infra_frame_2.get_data())
        

        # Apply colormap on depth and IR images (image must be converted to 8-bit per pixel first)
        depth_colormap   = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,   alpha=0.03), cv2.COLORMAP_JET)
        infra_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_1, alpha=1), cv2.COLORMAP_JET)
        infra_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(infra_image_2, alpha=1), cv2.COLORMAP_JET)
        
        bridge=CvBridge()
        # Convert to ROS2 Image type
        # - RGB Image
        self._rgb_image = bridge.cv2_to_imgmsg(color_image,"bgr8")
        self._rgb_image.header.stamp = self.get_clock().now().to_msg()
        self._rgb_image.header.frame_id = "image"
        # - Depth Image Colormap
        self._depth_image = bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
        self._depth_image.header.stamp = self._rgb_image.header.stamp
        self._depth_image.header.frame_id = "depth"
        # - Depth Image Values
        val = Float32MultiArray()
        val.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        val.layout.dim[0].label = 'width'
        val.layout.dim[1].label = 'height'
        val.layout.dim[0].size = self._cam_width
        val.layout.dim[1].size = self._cam_height
        
        
        val.data = depth_dist.reshape(self._cam_height*self._cam_width)
        #print(depth_dist)
        self._depth_dist = val
        
        # - IR1 Image
        self._infra1_image = bridge.cv2_to_imgmsg(infra_colormap_1,"bgr8")
        self._infra1_image.header.stamp = self._rgb_image.header.stamp
        self._infra1_image.header.frame_id = "infrared_1"
        # - IR2 Image
        self._infra2_image = bridge.cv2_to_imgmsg(infra_colormap_2,"bgr8")
        self._infra2_image.header.stamp = self._rgb_image.header.stamp
        self._infra2_image.header.frame_id = "infrared_2"

        return self._rgb_image, self._depth_image, self._infra1_image, self._infra2_image

    def publish_imgs(self):
        """
        publish ros images
        """
        self.publish_rgb()
        self.publish_depth()
        self.publish_IR()


    def publish_rgb(self):
        """ 
        Publish RGB image
        """

        self._rgb_publisher.publish(self._rgb_image)

        return 0
    
    def publish_depth(self):
        """ 
        Publish depth colormap image
        """

        self._depth_publisher.publish(self._depth_image)
        self._depth_dist_publisher.publish(self._depth_dist)

        return 0

    def publish_IR(self):
        """ 
        Publish infrared colormap image
        """

        self._infra_publisher_1.publish(self._infra1_image)
        self._infra_publisher_2.publish(self._infra2_image)

        return 0

# Capture ctrl-c event
isOk=True
def signalInterruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

def main():
    """
    Main loop
    """

    signal.signal(signal.SIGINT, signalInterruption)

    rclpy.init()
    rosNode = CameraDriver()
    isOk = True
    while isOk:
        rosNode.read_imgs()
        rclpy.spin_once(rosNode, timeout_sec=0.001)


    rosNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()