import rclpy                  # core ROS2 client python librairie
from rclpy.node import Node   # To manipulate ROS Nodes

from geometry_msgs.msg import Twist # Message to publish:

print("tuto_move :: START...")

def main():
    rclpy.init()     # Initialize ROS2 client
    myNode= Node('move_node') # Create a Node, with a name     

    # Initialize a publisher:
    velocity_publisher = myNode.create_publisher(Twist, '/cmd_vel', 10)


    # Start the ros infinit loop with myNode.
    while True :
        rclpy.spin_once( myNode, timeout_sec=0.1 )
        print("Running...")

        # Publish a msg
        velo = Twist()
        velo.linear.x= 0.2   # meter per second
        velo.angular.z = 0.5 # radians per second
        velocity_publisher.publish(velo)    

    # # At the end, destroy the node explicitly.
    # myNode.destroy_node()

    # # and shut the light down.
    # rclpy.shutdown()

    # print("tuto_move :: STOP.")











# activate main() function,
# if the file is executed as a script (ie. not imported).
if __name__ == '__main__':
    # call main() function
    main()