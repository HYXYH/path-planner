import rospy
import threading
from std_msgs.msg import String
from std_msgs.msg import PoseStamped
from vi_device_msgs import OdometryExtended
from motion_control_msgs import WaypointArray2, Waypoint2
from vi_pc_perception import ObstacleArray, Obstacle


def path_planner():
    publisher = rospy.Publisher('/control/global/advance_route', WaypointArray2, queue_size=10)
    planner = LocalPlanner(publisher, None)

    rospy.init_node('path_planner', anonymous=False)
    rospy.Subscriber('/map/pose', PoseStamped, planner.update_position)
    rospy.Subscriber('/odometry/extended', OdometryExtended, planner.update_speed)
    rospy.Subscriber('/vision/front/obstacle_detection', ObstacleArray, planner.update_obstacles)
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       planner.publish()
       r.sleep()


class LocalPlanner:
    position = None
    speed = None
    obstacles_data = None
    algorithm = None

    def __init__(self, publisher, algorithm):
        self.publisher = publisher
        self.algorithm = algorithm
        self.lock = threading.Lock()

    def update_position(self, pos):
        with self.lock:
            self.position = pos

    def update_speed(self, speed):
        with self.lock:
            self.speed = speed

    def update_obstacles(self, data):
        with self.lock:
            self.obstacles_data = data

    def calculate(self):
        current_data = None
        with self.lock:
            current_data = (self.position, self.speed, self.obstacles_data)
        return algorithm(current_data)

    def publish(self):
        self.publisher.publish(calculate())
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        

if __name__ == '__main__':
    path_planner()