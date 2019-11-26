import rospy
from std_msgs.msg import String
from std_msgs.msg import PoseStamped
from vi_device_msgs import OdometryExtended
from motion_control_msgs import WaypointArray2, Waypoint2
from vi_pc_perception import ObstacleArray, Obstacle


def path_planner():
    publisher = rospy.Publisher('/control/global/advance_route', WaypointArray2, queue_size=10)
    planner = LocalPlanner(publisher)

    rospy.init_node('path_planner', anonymous=False)
    rospy.Subscriber('/map/pose', PoseStamped, planner.update_position)
    rospy.Subscriber('/odometry/extended', OdometryExtended, planner.update_speed)
    rospy.Subscriber('/vision/front/obstacle_detection', ObstacleArray, planner.update_obstacles)
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
       planner.publish()
       r.sleep()


class LocalPlanner:

    def __init__(self, publisher):
        self.publisher = publisher

    def update_position(self, pos):
        pass

    def update_speed(self, speed):
        pass

    def update_obstacles(self, data):
        pass

    def calculate():
        pass

    def publish(self):
        calculate()
        self.publisher.publish("hello world")
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        

if __name__ == '__main__':
    path_planner()