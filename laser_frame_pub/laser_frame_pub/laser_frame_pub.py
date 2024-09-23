import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import math
from scipy.spatial.transform import Rotation as R
import socket
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class LaserFramePub(Node):
    """
    This handles rotation, I don't think we need to also publish the translation from
    the chassis frame. I thin these transorms are atomic operations, get a message, apply the
    transorm, there may be a separate translation message
    """
    def __init__(self):
        
        super().__init__("LaserFramePub")
        self.pub = self.create_publisher(JointState, "/laser_frame", 10)
        self.broadcaster = TransformBroadcaster(self, qos=10)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("192.168.0.100",19999))

        self.lidar_trans = TransformStamped()
        self.lidar_trans.header.frame_id = 'base_link'
        self.lidar_trans.child_frame_id = 'laser_frame'
        self.joint_state = JointState()

        self.create_timer(0.1, self.listen)

    def listen(self):
        data = None
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.error as ex:
                if ex.args[0] == socket.errno.EWOULDBLOCK:
                    break
        if not data:
            return
        rads = np.frombuffer(data,np.float64)[0]
        # Data is radians
        now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = now
        self.joint_state.position = data
        self.pub.publish(self.joint_state)

        self.lidar_trans.header.stamp = now
        # lidar_trans.transform.translation
        Q = quaternion_from_euler(0,rads,0)
        self.lidar_trans.transform.rotation.x = Q[0]
        self.lidar_trans.transform.rotation.y = Q[1]
        self.lidar_trans.transform.rotation.z = Q[2]
        self.lidar_trans.transform.rotation.w = Q[3]
        self.broadcaster.sendTransform(self.lidar_trans)
        


def main(args=None):
    rclpy.init(args=args)
    node = LaserFramePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
        
if __name__=="__main__":
    main()