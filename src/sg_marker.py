import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

sequences = [
        # Sequence 1
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (-5.376387596130371, -1.9829909801483154, 0.0025796890258789062)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (-7.038274765014648, -6.100484848022461, 0.004405975341796875)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (2.0859322547912598, -5.926054954528809, 0.004673004150390625)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (2.153135299682617, -1.359513759613037, 0.0032796859741210938)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (-3.689016342163086, -1.930342197418213, 0.00272369384765625)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (3.6924867630004883, -5.551314830780029, 0.005063056945800781)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (5.42404842376709, -1.186535358428955, 0.0034856796264648438)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (5.483254432678223, -5.260541915893555, 0.0053730010986328125)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (-5.36610221862793, -3.011920690536499, 0.0015506744384765625)},
        {'start': (0.035176217555999756, 0.0025000572204589844, 0.004619598388671875), 'goal': (-0.5889233350753784, -6.269580841064453, 0.004302024841308594)},
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