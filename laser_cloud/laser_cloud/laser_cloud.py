import rclpy
import asyncio
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import array
import time

class LaserProjection:
    """
    A class to Project Laser Scan

    This class will project laser scans into point clouds. It caches
    unit vectors between runs (provided the angular resolution of
    your scanner is not changing) to avoid excess computation.

    By default all range values less than the scanner min_range or
    greater than the scanner max_range are removed from the generated
    point cloud, as these are assumed to be invalid.

    If it is important to preserve a mapping between the index of
    range values and points in the cloud, the recommended approach is to
    pre-filter your laser scan message to meet the requirement that all
    ranges are between min and max_range.

    The generate PointClouds have a number of channels which can be enabled
    through the use of ChannelOption.
    - ChannelOption.INTENSITY - Create a channel named "intensities" with the
    intensity of the return for each point.
    - ChannelOption.INDEX     - Create a channel named "index" containing the
    index from the original array for each point.
    - ChannelOption.DISTANCE  - Create a channel named "distance" containing
    the distance from the laser to each point.
    - ChannelOption.TIMESTAMP - Create a channel named "stamps" containing the
    specific timestamp at which each point was measured.
    """

    LASER_SCAN_INVALID   = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE      = 0x00 # Enable no channels
        INTENSITY = 0x01 # Enable "intensities" channel
        INDEX     = 0x02 # Enable "index"       channel
        DISTANCE  = 0x04 # Enable "distances"   channel
        TIMESTAMP = 0x08 # Enable "stamps"      channel
        VIEWPOINT = 0x10 # Enable "viewpoint"   channel
        DEFAULT   = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0

        self.__cos_sin_map = np.array([[]])

    def projectLaser(self, scan_in,
            range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        """
        Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2.

        Project a single laser scan from a linear array into a 3D
        point cloud. The generated cloud will be in the same frame
        as the original laser scan.

        Keyword arguments:
        scan_in -- The input laser scan.
        range_cutoff -- An additional range cutoff which can be
            applied which is more limiting than max_range in the scan
            (default -1.0).
        channel_options -- An OR'd set of channels to include.
        """
        return self.__projectLaser(scan_in, range_cutoff, channel_options)

    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)

        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != scan_in.angle_min or
            self.__angle_max != scan_in.angle_max):
            rclpy.logging.get_logger("project_laser").debug(
                    "No precomputed map given. Computing one.")

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max
            
            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        # Set the output cloud accordingly
        cloud_out = PointCloud2()

        fields = [pc2.PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = pc2.PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = pc2.PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = pc2.PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance =  idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
            len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = pc2.PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(pc2.PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([pc2.PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = pc2.PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        for i in range(N):
            ri = scan_in.ranges[i]
            if ri < range_cutoff and ri >= scan_in.range_min:
                point = output[:, i].tolist()
                point.append(0)

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

        cloud_out = pc2.create_cloud(scan_in.header, fields, points)

        return cloud_out

    def transformLaserScan2PointClout(self, cloud_out):
        """
        Fields are Default [x,y,z,intensity,index] all 4 byte floats

        https://github.com/ros-perception/laser_geometry/blob/kinetic-devel/src/laser_geometry.cpp#L573
        """
        pass



class LaserSubscriber(Node):
    """
    TODO: 
        - publish point cloud every scan. Adding the new scan to our point cloud,
        - parameterize time in which to delete old scans.
        - Send to SLAM/LOAM mapping node someday
        - Profile the callbacks and optimize with more numpy/c-extensions/pyO3 rust extensions
    """

    def __init__(self):
        super().__init__("LaserSub")
        self.lock = asyncio.Lock()
        self.target_frame = "base_link"
        self.lp = LaserProjection()  
        self.pub = self.create_publisher(PointCloud2, "/converted_scan", 10)
        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.clear_time = time.time()
        self.cloud_out = PointCloud2()

    async def timer_callback(self):
        """Publish accumulated point cloud"""
        async with self.lock:
            self.pub.publish(self.cloud_out)
            if time.time() - self.clear_time > 14:
                self.cloud_out = PointCloud2()
                self.clear_time = time.time()

    async def scan_cb(self, msg):
        """
        Fields are Default [x,y,z,intensity,index] all 4 bytes for 20 bytes a point
        """
        try:

            # convert the message of type LaserScan to a PointCloud2
            cloud_row = self.lp.projectLaser(msg)

            # Get the latest transform to rotate the laserscan into and add to the accumulating point cloud
            trans = await self._tf_buffer.lookup_transform_async("laser_frame","base_link", rclpy.time.Time(seconds=0,nanoseconds=0))
            
            rot = R.from_quat([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
            
            # slerp from the last rotation to this one
            # slerp = Slerp([x*time_increment for x in range(cloud_row.width)])
            
            # Translation for moving robots
            # trans.transform.translation

            # xyz_arr has 5 fields for [x,y,z,intesity,index]
            xyz_arr = np.empty((cloud_row.width,3), dtype=np.float32)
            bits = cloud_row.data.tobytes()
            xyz_arr[:,0] = np.frombuffer(bits, np.float32)[0::5]
            xyz_arr[:,1] = np.frombuffer(bits, np.float32)[1::5]
            xyz_arr[:,2] = np.frombuffer(bits, np.float32)[2::5]
            
            rotated = rot.apply(xyz_arr)

            out_arr = np.zeros((cloud_row.width,), dtype=np.dtype(
                    [
                        ("x",np.float32),
                        ("y",np.float32),
                        ("z",np.float32),
                        ("intensity",np.float32),
                        ("index",np.int32),
                    ]
                )
            )
            out_arr['x'][:] = rotated.astype(np.float32)[:,0]
            out_arr['y'][:] = rotated.astype(np.float32)[:,1]
            out_arr['z'][:] = rotated.astype(np.float32)[:,2]
            out_arr['intensity'][:] = np.frombuffer(bits, np.float32)[3::5]
            out_arr['index'][:] = np.frombuffer(bits, np.int32)[4::5]

            rotated_cloud = array.array('B', out_arr.tobytes())

            
            
            async with self.lock:
                if self.cloud_out.height == 0:
                    self.cloud_out = cloud_row
                    self.cloud_out.header.frame_id = self.target_frame                
                self.cloud_out.height += 1

                self.cloud_out.width = max(cloud_row.width,self.cloud_out.width)
                self.cloud_out.data.extend(rotated_cloud)

                if len(self.cloud_out.data) < self.cloud_out.height*self.cloud_out.width*self.cloud_out.point_step:
                    diff = abs( len(self.cloud_out.data) - self.cloud_out.height*self.cloud_out.width*self.cloud_out.point_step)
                    self.cloud_out.data.extend([0]*diff)
            
        except Exception as ex:
            self._logger.error(ex)

def main(args=None):

    loop = asyncio.get_event_loop()

    async def mainloop():
        rclpy.init(args=args)
        node = LaserSubscriber()
        while rclpy.ok():
            rclpy.spin_once(node)
            await asyncio.sleep(0)
        node.destroy_node()
        rclpy.shutdown()
    loop.create_task(mainloop())
    loop.run_forever()
        
if __name__=="__main__":
    main()