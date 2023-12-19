#!python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2
from std_msgs.msg import Header

rosNode= None

def scan_callback(scanMsg):
    global rosNode
    obstacles = ranges_to_positions(scanMsg)
    sample = [ [ round(p[0], 2), round(p[1], 2), round(p[2], 2) ] for p in  obstacles[10:20] ]


    # Publish a msg
    pointCloud = sensor_msgs_py.point_cloud2.create_cloud_xyz32(Header(frame_id='frame'), obstacles)
    rosNode.get_logger().info( f"\nheader:\n{scanMsg.header}\nnumber of ranges: {len(scanMsg.ranges)}\nSample: {pointCloud}\n" )
    point_publisher.publish(pointCloud)    


def ranges_to_positions(scan_message: LaserScan) -> list:
    obstacles= []
    angle= scan_message.angle_min
    for aDistance in scan_message.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance,
                0
            ]
            obstacles.append(aPoint)
        angle+= scan_message.angle_increment
    return obstacles


rclpy.init()
rosNode= Node('scan_interpreter')
# Initialize a subscriber:
rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)
# Initialize a publisher:
point_publisher = rosNode.create_publisher(PointCloud2, '/scan_points', 10)

while True :
    rclpy.spin_once( rosNode )




scanInterpret.destroy_node()
rclpy.shutdown()