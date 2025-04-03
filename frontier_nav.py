import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf2_ros
import tf2_geometry_msgs

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from enum import Enum

import numpy as np

import math

OCC_THRESHOLD = 10
MIN_FRONTIER_LIST = 2

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            print(f'wx: {wx}, wy : {wy}')
            print(f'origin: {self.map.info.origin.position.x}, {self.map.info.origin.position.y}')
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)
    
    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx
    
class FrontierCache():
    cache = {}

    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)

        if idx in self.cache:
            return self.cache[idx]

        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}

class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length

def findFree(mx, my, costmap):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(pose, costmap, logger):
    print("Getting frontiers")
    fCache = FrontierCache()

    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    newX, newY = (pose.position.x, pose.position.y)
    print(f"pose: {int(newX)}, {int(newY)}")

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    print(f"map point: {mapPointQueue[0].mapX}, {mapPointQueue[0].mapY}")
    print(f"map point: {mx}, {my}")
    print(f"map point: {costmap.getCost(mapPointQueue[0].mapX, mapPointQueue[0].mapY)}")
    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue
        
        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value

            
            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            print(f"newFrontier: {len(newFrontier)}")
            if len(newFrontier) > MIN_FRONTIER_LIST:
                frontiers.append(centroid(newFrontierCords))

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers

def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point, costmap, fCache):
    print(costmap.getCost(point.mapX, point.mapY))
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False
    print(costmap.getCost(point.mapX, point.mapY))
    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)
        print(f"x: {n.mapX}, y: {n.mapY}")
        print(cost)
        '''if cost > OCC_THRESHOLD:
            print("exiting")
            return False'''

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class FrontierNav(Node):
    def __init__(self):
        super().__init__('frontier_nav')
        self.currentPose = None
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.UNKNOWN,
          depth=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.model_pose_sub = self.create_subscription(Odometry, 'odom', self.poseCallback, 10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.costmapSub = self.create_subscription(OccupancyGrid, 'map', self.occupancyGridCallback, qos_profile_sensor_data)
        self.costmap = None
        
        self.get_logger().info('Starting Navigation')

    def occupancyGridCallback(self, msg):
        print(msg.data)
        self.costmap = OccupancyGrid2d(msg)

    def rotateBot(self, angle):
        twist = Twist()
        curr_yaw = self.yaw
        c_yaw = complex(math.cos(curr_yaw), math.sin(curr_yaw))
        new_yaw = curr_yaw + angle
        c_new_yaw = complex(math.cos(new_yaw), math.sin(new_yaw))

        c_change = c_new_yaw / c_yaw
        dir = 1 if c_change.imag > 0 else -1

        twist.linear.x = 0.0
        twist.angular.z = 0.5 * dir
        self.publisher_.publish(twist)

        og_dir = dir
        while(og_dir*dir>0):
            rclpy.spin_once(self)
            curr_yaw = self.yaw
            c_yaw = complex(math.cos(curr_yaw), math.sin(curr_yaw))
            c_change = c_new_yaw / c_yaw
            dir = 1 if c_change.imag > 0 else -1
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def pick_direction(self):
        frontiers = getFrontier(self.currentPose, self.costmap, self.get_logger())
        if len(frontiers) == 0:
            raise Exception("No frontiers found")
        
        location = None
        largest_dist = 0
        for f in frontiers:
            dist = math.sqrt((f[0] - self.currentPose.position.x)**2 + (f[1] - self.currentPose.position.y)**2)
            if dist > largest_dist:
                location = f
                largest_dist = dist

        delta_x = self.currentPose.position.x - location[0]
        delta_y = self.currentPose.position.y - location[1]
        angle = math.atan2(delta_y, delta_x)
        self.get_logger().info(f'Angle: {angle}')
        self.get_logger().info(f'Location: {location}')

        self.rotateBot(angle)

        self.get_logger().info('Rotated')

        while (delta_x > 0.1 or delta_y > 0.1):
            rclpy.spin_once(self)
            delta_x = self.currentPose.position.x - location[0]
            #delta_x = location[0] - self.currentPose.position.x
            delta_y = self.currentPose.position.y - location[1]

            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            time.sleep(1)
            self.publisher_.publish(twist)
            self.get_logger().info('Moving')
        

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
    
    def move(self):
        try:
            self.pick_direction()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.stopbot()

    def transform_pose(self, input_pose, from_frame, to_frame):
        # Set the header frame id of the pose to be sure
        input_pose.header.frame_id = from_frame
        try:
            # Use the current time (or a time from the header) for lookup
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose.pose, transform)
            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None
    
    def poseCallback(self, msg):
        self.info_msg('Received pose')
        self.currentPose = msg.pose.pose
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = self.currentPose
        transformed = self.transform_pose(pose_stamped, from_frame="odom", to_frame="map")
        if transformed is not None:
            print("Transformed pose successfully")
            self.currentPose = transformed
            orientation = self.currentPose.orientation
            self.roll, self.pitch, self.yaw = euler_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w)
        else:
            self.get_logger().error("Failed to transform pose")
            orientation = self.currentPose.orientation
            self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
    
    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)

def main(args=None):
    rclpy.init(args=args)
    auto_nav = FrontierNav()
    while auto_nav.costmap is None and rclpy.ok():
        auto_nav.get_logger().info("Waiting for occupancy grid...")
        rclpy.spin_once(auto_nav, timeout_sec=0.5)
    while rclpy.ok():
        rclpy.spin_once(auto_nav, timeout_sec=0.1)
        try:
            auto_nav.move()
        except Exception as e:
            if "No frontiers found" in str(e):
                auto_nav.get_logger().info("No more frontiers available. Shutting down.")
                break
            else:
                auto_nav.get_logger().error(f'Error: {e}')
    
    time.sleep(4)
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
