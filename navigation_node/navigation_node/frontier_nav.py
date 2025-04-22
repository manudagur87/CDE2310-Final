import time

from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Int8

import tf2_ros
import tf2_geometry_msgs

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from enum import Enum

import numpy as np

import math

OCC_THRESHOLD = 10
MIN_FRONTIER_LIST = 40
MERGE_RADIUS = 0.5
THRESHOLD = 24
FIELD_OF_VIEW = 60
ROTATE_CHANGE = 0.1
SPEED_CHANGE = 0.1
FRONT_ANGLE = 30
STOP_DISTANCE = 0.25
SAFETY_DIST = 0.5

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
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) // self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) // self.map.info.resolution)
        
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
    median_x = np.median(arr[:, 0])
    median_y = np.median(arr[:, 1])
    return median_x, median_y

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
    print(f"frontiers: {frontiers}")
    return frontiers

def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False
    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)
        if cost > OCC_THRESHOLD:
            return False

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
        self.launch_ = self.create_publisher(Int8, 'launch_topic', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.model_pose_sub = self.create_subscription(Odometry, 'odom', self.poseCallback, 10)
        self.ogPose = None
        time.sleep(1)

        self.costmapSub = self.create_subscription(OccupancyGrid, 'map', self.occupancyGridCallback, qos_profile_sensor_data)
        self.costmap = None
        
        self.get_logger().info('Starting Navigation')

        self.thermalSub = self.create_subscription(Float64MultiArray, 'thermal_image', self.thermal_image_callback, 10)
        self.thermalpoint = None
        self.visitedThermal = []
        self.nearHeat = False
        self.thermalRan = False

        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)   
        self.laser_range = np.array([])   
        self.allFrontier = False  
        self.angle_min = None

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)

    def goal_pose(self,navigator):
        self.thermalgoal = False
        rclpy.spin_once(self)
        time.sleep(0.1)
        visited = False
        if self.thermalpoint is not None:
            for (vx,vy) in self.visitedThermal:
                if math.hypot(self.thermalpoint.point.x - vx, self.thermalpoint.point.y - vy) < MERGE_RADIUS:
                    self.get_logger().info("Already visited thermal point")
                    self.thermalpoint = None
                    visited = True
                    break
        if(self.allFrontier and self.thermalpoint is None):
            location = (self.randompoint.point.x, self.randompoint.point.y)
            self.get_logger().info("Sending randompoint")
        
        elif((self.thermalpoint is None or visited) and not self.allFrontier):
            frontiers = getFrontier(self.currentPose, self.costmap, self.get_logger())
            if len(frontiers) == 0:
                self.allFrontier = True
            
            location = None
            largest_dist = 0
            for f in frontiers:
                dist = math.sqrt((f[0] - self.currentPose.position.x)**2 + (f[1] - self.currentPose.position.y)**2)
                if dist > largest_dist:
                    location = f
                    largest_dist = dist
            self.get_logger().info("Sending Frontier")
        else:
            location = (self.thermalpoint.point.x, self.thermalpoint.point.y)
            self.get_logger().info("Sending Thermalpoint")
            self.thermalgoal = True



        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = location[0]
        goal_pose.pose.position.y = location[1]
        goal_pose.pose.orientation.w = self.currentPose.orientation.w
        goal_pose.pose.orientation.z = self.currentPose.orientation.z

        return goal_pose
    
    def move(self,navigator):
        init = PoseStamped()
        init.header.frame_id = "map"
        init.header.stamp = self.get_clock().now().to_msg()
        init.pose = self.currentPose
        navigator.setInitialPose(init)
        #print("Initial pose set")
        #print(init.pose.position.x,init.pose.position.y)
        #navigator.waitUntilNav2Active(localizer='slam_toolbox')
        #WRITE CODE FOR TURNING 360 DEGREES TO LOOK FOR HEAT SIGNATURES
        
        goal_pose = self.goal_pose(navigator)
        navigator.goToPose(goal_pose)
        i = 0
        while not navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.01)
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=45.0):
                    if self.thermalgoal==False:
                        navigator.cancelTask()
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if self.thermalgoal==True:
                self.launch_.publish(Int8(1))
                time.sleep(15)
                self.nearHeat = False
                self.visitedThermal.append((self.thermalpoint.point.x, self.thermalpoint.point.y))
                self.thermalpoint = None
                self.get_logger().info("Visited thermal point")
                goal_pose = self.goal_pose(navigator)
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = navigator.getTaskError()
            print('Goal failed!{error_code}:{error_msg}')
        else:
            print('Goal has an invalid return status!')   
        

    def transform_pose(self, input_pose, from_frame, to_frame, pose=True):
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
            if pose:
                transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose.pose, transform)
            else:
                transformed_pose = tf2_geometry_msgs.do_transform_point(input_pose, transform)
            return transformed_pose
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None
    
    def poseCallback(self, msg):
        self.info_msg('Received pose')
        self.currentPose = msg.pose.pose
        self.ogPose = self.currentPose
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
        ogorientation = self.ogPose.orientation
        self.ogroll, self.ogpitch, self.ogyaw = euler_from_quaternion(ogorientation.x, ogorientation.y, ogorientation.z, ogorientation.w)
    
    def thermal_image_callback(self, msg):
        if len(self.laser_range)==0:
            return
        self.thermalRan = True
        self.image_data = msg.data
        if len(self.image_data) != 64:
            self.get_logger().warn("Received data does not contain 64 elements.") # for error checking
            return
        
        max_val = np.max(self.image_data)
        if max_val < THRESHOLD or self.ogPose is None:
            self.get_logger().info('No target detected.')
            return
        
        max_index = self.image_data.index(max_val)
        col = max_index % 8

        avg_val = 0
        for i in range(8):
            avg_val += self.image_data[col+(8*i)]
        avg_val = avg_val/8

        if avg_val > 35.5:
            self.nearHeat = True
            return
        self.bearing = ((col-3.5)/7)*FIELD_OF_VIEW

        global_angle = self.ogyaw + np.deg2rad(self.bearing)

        self.thermalpoint = PointStamped()
        self.thermalpoint.header.stamp = self.get_clock().now().to_msg()
        self.thermalpoint.header.frame_id = 'odom'
        cleaned = self.laser_range[~np.isnan(self.laser_range)]
        final =[]
        if(self.bearing<0):
            final.append(cleaned[math.floor((self.bearing*11/18))])
            for i in range(1,4):
                if (math.floor((self.bearing*11/18))+i)<len(cleaned):
                    final.append(cleaned[math.floor((self.bearing*11/18))+i])
                if (math.floor((self.bearing*11/18))-i)>0: 
                    final.append(cleaned[math.floor((self.bearing*11/18))-i])
            distance = np.min(final)
        else:
            final.append(cleaned[len(cleaned)-math.floor((self.bearing*11/18))])
            for i in range(1,4):
                if (len(cleaned)-math.floor((self.bearing*11/18))+i)<len(cleaned):
                    final.append(cleaned[len(cleaned)-math.floor((self.bearing*11/18))+i])
                if (len(cleaned)-math.floor((self.bearing*11/18))-i)>0:
                    final.append(cleaned[len(cleaned)-math.floor((self.bearing*11/18))-i])
            distance = np.min(final)
        self.thermalpoint.point.x = float(self.ogPose.position.x+(distance*math.cos(global_angle)))
        self.thermalpoint.point.y = float(self.ogPose.position.y+(distance*math.sin(global_angle)))
        transformed = self.transform_pose(self.thermalpoint, from_frame="odom", to_frame="map", pose=False)
        if transformed is not None:
            print("Transformed thermalpoint successfully")
            self.thermalpoint = transformed
        else:
            self.get_logger().error("Failed to transform point")

    def scan_callback(self, msg):
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        if not self.ogPose:
            return
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan  
        self.laser_range[np.isinf(self.laser_range)] = np.nan
        largest_distance = np.nanmax(self.laser_range)
        largest_angle = np.deg2rad(np.nanargmax(self.laser_range) *(18/11))

        self.randompoint = PointStamped()
        self.randompoint.header.stamp = self.get_clock().now().to_msg()
        self.randompoint.header.frame_id = 'odom'
        self.randompoint.point.x = float(self.ogPose.position.x + (largest_distance * math.sin((math.pi/2)-self.ogyaw-largest_angle)))
        self.randompoint.point.y = float(self.ogPose.position.y + (largest_distance * math.cos((math.pi/2)-self.ogyaw-largest_angle)))
        transformed = self.transform_pose(self.randompoint, from_frame="odom", to_frame="map", pose=False)
        if transformed is not None:
            print("Transformed randompoint successfully")
            self.randompoint = transformed
        else:
            self.get_logger().error("Failed to transform point")

    
    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)

def main(args=None):
    rclpy.init(args=args)
    auto_nav = FrontierNav()
    navigator = BasicNavigator()
    while auto_nav.costmap is None and rclpy.ok():
        auto_nav.get_logger().info("Waiting for occupancy grid...")
        rclpy.spin_once(auto_nav, timeout_sec=0.5)
    while rclpy.ok():
        try:
            auto_nav.move(navigator)
        except Exception as e:
            auto_nav.get_logger().error(f'Error: {e}')
    
    time.sleep(4)
    navigator.lifecycleShutdown()
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
