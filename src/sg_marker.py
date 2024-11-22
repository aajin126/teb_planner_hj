import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

sequences = [
        # Sequence 1
        {'start': (1.824033, 6.769977, 0.096273, -1.287503), 'goal': (6.768701, 1.057576, 0.000000, 0.773270)},
        {'start': (0.985526, 6.917296, 0.096226, 0.027548), 'goal': (5.232610, 1.684851, 0.000000, 0.999823)},
        {'start': (5.269928, 1.684734, 0.096243, 0.025992), 'goal': (1.989110, 5.906961, 0.000000, 0.716123)},
        # Sequence 2
        #{'start': (6.986666, 0.376367, 0.096228, -2.253656), 'goal': (-1.831612, 1.669143, 0.000000, -0.014315)},
        {'start': (6.788350, -0.089891, 0.096250, -3.096101), 'goal': (-1.868525, 1.650294, 0.000000, -0.014738)},
        {'start': (6.805717, 0.113735, 0.096264, -2.119764), 'goal': (-0.554213, 1.652943, 0.000000, 0.014982)},
        # Sequence 3
        {'start': (-0.197335, 1.673094, 0.096252, 3.128338), 'goal': (-5.710883, -1.709810, 0.000000, -0.011583)},
        {'start': (-2.574699, 1.653359, 0.096229, 3.141128), 'goal': (-7.020387, -1.723354, 0.000000, 0.603660)},
        {'start': (-3.642075, 1.760398, 0.096239, -3.131948), 'goal': (-6.064654, -1.638782, 0.000000, -0.077751)},
        # Sequence 4
        {'start': (-5.377285, -0.750267, 0.096239, -1.742384), 'goal': (-5.645093, -5.850183, 0.000000, 1.000000)},
        {'start': (-5.485104, -4.714368, 0.096235, 1.341129), 'goal': (-2.009097, -3.007130, 0.000000, 0.999909)},
        {'start': (-5.496230, -4.258798, 0.096249, 1.746675), 'goal': (-4.039498, -3.428050, 0.000000, 0.996223)},
        {'start': (-5.487377, -5.311635, 0.096243, 1.494585), 'goal': (-3.880702, -3.380883, 0.000000, 0.999839)},
       # Sequence 6
        {'start': (-2.563111, -3.040125, 0.096228, -0.078109), 'goal': (1.376936, -6.336720, 0.000000, 0.999945)},
        {'start': (1.504560, -6.190240, 0.096256, 0.578695), 'goal': (3.139581, -2.103034, 0.000000, 0.999866)},
        {'start': (-1.074328, -6.349617, 0.096226, 0.018165), 'goal': (3.127539, -0.420866, 0.000000, 0.999835)},
        {'start': (3.177052, -0.367005, 0.096233, 0.014759), 'goal': (3.895634, -5.610425, 0.000000, 0.998171)},
        # Sequence 7
        {'start': (1.430136, -6.335843, 0.096231, 0.057171), 'goal': (2.920930, -2.147175, 0.000000, 0.999424)},
        {'start': (3.687236, -1.241461, 0.096225, -1.591248), 'goal': (5.276931, -3.293713, 0.000000, 0.711566)},
        {'start': (4.203457, -0.809600, 0.096230, 0.235379), 'goal': (6.733112, -5.549973, 0.000000, -0.369820)}
        ]

# ROS 초기화
rospy.init_node('rviz_marker_display')

# 퍼블리셔 설정
start_marker_pub = rospy.Publisher('start_marker', Marker, queue_size=10)
goal_marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)

# 마커 생성 함수
def create_marker(marker_id, position, color, shape):
    marker = Marker()
    marker.header.frame_id = "map"  # 사용 중인 frame_id 확인 후 수정 필요
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_id
    marker.type = shape
    marker.action = Marker.ADD
    marker.pose.position = Point(position[0], position[1], 0)
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    return marker

# start와 goal 위치에 마커 추가
rate = rospy.Rate(1)
marker_id = 0

while not rospy.is_shutdown():
    for sequence in sequences:
        start_pos = sequence['start']
        goal_pos = sequence['goal']
        
        # Start marker (red circle)
        start_marker = create_marker(marker_id, start_pos, (1.0, 0.0, 0.0), Marker.SPHERE)
        start_marker_pub.publish(start_marker)
        
        # Goal marker (blue square)
        goal_marker = create_marker(marker_id + 1, goal_pos, (0.0, 0.0, 1.0), Marker.CUBE)
        goal_marker_pub.publish(goal_marker)
        
        marker_id += 2  # 각 마커마다 고유 ID 필요
        rate.sleep()