import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image

import tf2_ros
import tf2_geometry_msgs

import rclpy
import rclpy.time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from enum import Enum

import numpy as np

import math, cmath

OCC_THRESHOLD = 10
MIN_FRONTIER_LIST = 50
MERGE_RADIUS = 0.5
THRESHOLD = 20
FIELD_OF_VIEW = 60
ROTATE_CHANGE = 0.1
SPEED_CHANGE = 0.1
FRONT_ANGLE = 30
STOP_DISTANCE = 0.25

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

    #print(f"map point: {mapPointQueue[0].mapX}, {mapPointQueue[0].mapY}")
    #print(f"map point: {mx}, {my}")
    #print(f"map point: {costmap.getCost(mapPointQueue[0].mapX, mapPointQueue[0].mapY)}")
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
    #print(costmap.getCost(point.mapX, point.mapY))
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False
    #print(costmap.getCost(point.mapX, point.mapY))
    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)
        #print(f"x: {n.mapX}, y: {n.mapY}")
        #print(cost)
        if cost > OCC_THRESHOLD:
            #print("exiting")
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
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)

        '''pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.UNKNOWN,
          depth=1)'''

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.model_pose_sub = self.create_subscription(Odometry, 'odom', self.poseCallback, 10)
        '''self.roll = 0
        self.pitch = 0
        self.yaw = 0'''

        self.costmapSub = self.create_subscription(OccupancyGrid, 'map', self.occupancyGridCallback, qos_profile_sensor_data)
        self.costmap = None
        
        self.get_logger().info('Starting Navigation')

        self.thermalSub = self.create_subscription(Image, '/thermal_image', self.thermal_image_callback, 10)
        self.thermalpoint = None
        self.visitedThermal = []
        self.nearHeat = False

        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)   
        self.laser_range = np.array([])   
        self.allFrontier = False  

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)

    def goal_pose(self,navigator):
        rclpy.spin_once(self)

        if(self.allFrontier and self.thermalpoint is None):
            return None
        
        visited = False
        if self.thermalpoint is not None:
            for (vx,vy) in self.visitedThermal:
                if math.hypot(self.thermalpoint.point.x - vx, self.thermalpoint.point.y - vy) < MERGE_RADIUS:
                    self.get_logger().info("Already visited thermal point")
                    self.thermalpoint = None
                    visited = True
                    break
        
        if (self.thermalpoint is None or visited) and not self.allFrontier:
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
        else:

            location = (self.thermalpoint.point.x, self.thermalpoint.point.y)



        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = location[0]
        goal_pose.pose.position.y = location[1]
        goal_pose.pose.orientation.w = self.currentPose.orientation.w
        goal_pose.pose.orientation.z = self.currentPose.orientation.z

        return goal_pose

    '''def rotateBot(self, angle):
        twist = Twist()
        curr_yaw = self.yaw
        c_yaw = complex(math.cos(curr_yaw), math.sin(curr_yaw))
        new_yaw = curr_yaw + angle
        c_new_yaw = complex(math.cos(new_yaw), math.sin(new_yaw))

        c_change = c_new_yaw / c_yaw
        dir = 1 if c_change.imag > 0 else -1

        twist.linear.x = 0.0
        twist.angular.z = 0.3 * dir
        self.publisher_.publish(twist)
        self.get_logger().info('Rotating')

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
        self.publisher_.publish(twist)'''
    
    def move(self,navigator):
        '''try:
            self.pick_direction()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.stopbot()'''
        init = PoseStamped()
        init.header.frame_id = "map"
        init.header.stamp = self.get_clock().now().to_msg()
        init.pose = self.currentPose
        navigator.setInitialPose(init)
        #navigator.waitUntilNav2Active(localizer='slam_toolbox')
        #WRITE CODE FOR TURNING 360 DEGREES TO LOOK FOR HEAT SIGNATURES
        
        goal_pose = self.goal_pose(navigator)
        if self.nearHeat:
            #publish to solenoid node
            time.sleep(15)
            self.visitedThermal.append((self.thermalpoint.point.x, self.thermalpoint.point.y))
            self.thermalpoint = None
            self.get_logger().info("Visited thermal point")
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
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=45.0):
                    goal_pose.pose.position.x = 0.0
                    goal_pose.pose.position.y = 0.0
                    navigator.goToPose(goal_pose)
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            (error_code, error_msg) = navigator.getTaskError()
            print('Goal failed!{error_code}:{error_msg}')
        else:
            print('Goal has an invalid return status!')

    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * ROTATE_CHANGE
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = SPEED_CHANGE
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def randomWalk(self,navigator):
        goal_pose = self.goal_pose(navigator)
        if goal_pose is not None:
            if self.nearHeat:
                #publish to solenoid node
                time.sleep(15)
                self.visitedThermal.append((self.thermalpoint.point.x, self.thermalpoint.point.y))
                self.thermalpoint = None
                self.get_logger().info("Visited thermal point")
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
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelTask()

                    # Some navigation request change to demo preemption
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=45.0):
                        goal_pose.pose.position.x = 0.0
                        goal_pose.pose.position.y = 0.0
                        navigator.goToPose(goal_pose)
            
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                (error_code, error_msg) = navigator.getTaskError()
                print('Goal failed!{error_code}:{error_msg}')
            else:
                print('Goal has an invalid return status!')
        else:
            try:
                self.pick_direction()
                if self.laser_range.size != 0:
                    lri = (self.laser_range[FRONT_ANGLE]<float(STOP_DISTANCE)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                rclpy.spin_once(self) 
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
            finally:
                self.stopbot()  
     
    def rotate360(self):
        for i in range(0, 360, 90):
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.publisher_.publish(twist)
            time.sleep(3) #MUST CHANGE!!!!
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1)
            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(3)
            self.get_logger().info('Rotating')   
        

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
                transformed_pose = tf2_geometry_msgs.do_transform_point(input_pose.point, transform)
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
    
    def thermal(self, msg):
        self.info_msg('Received thermal point')
        self.thermalpoint = msg.point
        point_stamped = PointStamped()
        point_stamped.header = msg.header
        #setting the thermal point x and y positions with respect to the map!
        point_stamped.point.x = self.ogPose.position.x + math.hypot(self.thermalpoint.point.x, self.thermalpoint.point.y)*math.sin((math.pi/2)-self.ogyaw-self.bearing)
        point_stamped.point.x = self.ogPose.position.y + math.hypot(self.thermalpoint.point.x, self.thermalpoint.point.y)*math.cos((math.pi/2)-self.ogyaw-self.bearing)
        transformed = self.transform_pose(point_stamped, from_frame="odom", to_frame="map", pose=False)
        if transformed is not None:
            print("Transformed point successfully")
            self.thermalpoint = transformed
        else:
            self.get_logger().error("Failed to transform point")
    
    def thermal_image_callback(self, msg):
        image_data = np.frombuffer(msg.data, dtype=np.float32).reshape((8, 8))
        max_val = np.max(image_data)
        if max_val < THRESHOLD:
            self.get_logger().info('No target detected.')
            return
        
        (py,px) = np.unravel_index(np.argmax(image_data, image_data.shape))
        self.bearing = ((px-3.5)/7)*np.deg2rad(FIELD_OF_VIEW)

        point = PointStamped()
        point.header = msg.header
        point.point.x = self.x_calc()
        point.point.y = np.tan(self.bearing)*self.x_calc()

        self.thermal(point)

    def x_calc(self):
        self.nearHeat = False
        return    

    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        self.laser_range[self.laser_range==0] = np.nan    
    
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
            if auto_nav.allFrontier:
                auto_nav.randomWalk(navigator)
                auto_nav.rotate360()
            else:
                auto_nav.move(navigator)
                auto_nav.rotate360()
        except Exception as e:
            if "No frontiers found" in str(e):
                auto_nav.get_logger().info("No more frontiers available. Shutting down.")
                break
            else:
                auto_nav.get_logger().error(f'Error: {e}')
    
    time.sleep(4)
    navigator.lifecycleShutdown()
    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
