import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

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
        pass
        for i in range((cloud_out.points)):
            pass
            # get the index for this point
            pt_index = cloud_out.channels[index_channel_idx].values[i];

    #   // Instead, assume constant motion during the laser-scan, and use slerp to compute intermediate transforms
    #   tfScalar ratio = pt_index / ( (double) scan_in.ranges.size() - 1.0) ;

    #   //! \todo Make a function that performs both the slerp and linear interpolation needed to interpolate a Full Transform (Quaternion + Vector)

    #   //Interpolate translation
    #   tf::Vector3 v (0, 0, 0);
    #   v.setInterpolate3(start_transform.getOrigin(), end_transform.getOrigin(), ratio) ;
    #   cur_transform.setOrigin(v) ;

    #   //Interpolate rotation
    #   tf::Quaternion q1, q2 ;
    #   start_transform.getBasis().getRotation(q1) ;
    #   end_transform.getBasis().getRotation(q2) ;

    #   // Compute the slerp-ed rotation
    #   cur_transform.setRotation( slerp( q1, q2 , ratio) ) ;

    #   // Apply the transform to the current point
    #   tf::Vector3 pointIn(cloud_out.points[i].x, cloud_out.points[i].y, cloud_out.points[i].z) ;
    #   tf::Vector3 pointOut = cur_transform * pointIn ;

    #   // Copy transformed point into cloud
    #   cloud_out.points[i].x  = pointOut.x();
    #   cloud_out.points[i].y  = pointOut.y();
    #   cloud_out.points[i].z  = pointOut.z();


class LaserSubscriber(Node):

    def __init__(self):
        super().__init__("LaserSub")
        self.target_frame = "base_frame"
        self.lp = LaserProjection()  
        self.pub = self.create_publisher(PointCloud2, "/converted_scan", 10)
        self.sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.timer = self.create_timer(5, self.timer_callback)
        self.cloud_out = PointCloud2()

    def timer_callback(self):
        """Publish accumulated point cloud"""
        self.pub.publish(self.cloud_out)
        self.cloud_out = PointCloud2()

    def scan_cb(self, msg):
        # convert the message of type LaserScan to a PointCloud2
        cloud_out = self.lp.projectLaser(msg)
        # Set the output cloud accordingly
        # cloud_out = PointCloud2()
        cloud_out.header.frame_id = self.target_frame

        # Get the latest transform to rotate the laserscan into and add to the accumulating point cloud
        trans = self._tf_buffer.lookup_transform("laser_frame","base_frame", 0)
        # now we can do something with the PointCloud2 for example:
        # publish it
        # self.pub.publish(pc2_msg)
        
        # convert it to a generator of the individual points
        # point_generator = pc2.read_points(pc2_msg)
        # we can access a generator in a loop
        # sum = 0.0
        # num = 0
        # for point in point_generator:
        #     if not math.isnan(point[2]):
        #         sum += point[2]
        #         num += 1
        # we can calculate the average z value for example
        # print(str(sum/num))

        # or a list of the individual points which is less efficient
        # point_list = pc2.read_points_list(pc2_msg)

        # we can access the point list with an index, each element is a namedtuple
        # we can access the elements by name, the generator does not yield namedtuples!
        # if we convert it to a list and back this possibility is lost
        # print(point_list[len(point_list)/2].x)

def main(args=None):
    
    rclpy.init(args=args)
    node = LaserSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__=="__main__":
    main()