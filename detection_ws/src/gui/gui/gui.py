import rclpy
from gui.webgui import WebGUI
from gui.ros2backend import ROS2Backend

def main():
    rclpy.init()
    ros_backend = ROS2Backend()
    gui = WebGUI(ros_backend)
    gui.run()
