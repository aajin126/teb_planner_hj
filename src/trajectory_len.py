import rospy
from nav_msgs.msg import Odometry
import cv2
import numpy as np

# 이미지 설정
image_width = 800
image_height = 800
scale_factor = 20  # 스케일 조정
trajectory_points = []

def odometry_callback(msg):
    # 로봇의 현재 위치
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # 이미지 좌표로 변환 (이미지 중앙이 원점)
    img_x = int(x * scale_factor + image_width / 2)
    img_y = int(y * scale_factor + image_height / 2)

    # 트래젝토리 포인트를 저장
    trajectory_points.append((img_x, img_y))

def save_trajectory_image(filename):
    # 흰색 배경의 빈 이미지 생성
    image = np.ones((image_height, image_width, 3), dtype=np.uint8) * 255

    # 이동 경로 그리기
    for i in range(1, len(trajectory_points)):
        cv2.line(image, trajectory_points[i - 1], trajectory_points[i], (0, 0, 255), 2)  # 빨간색 경로

    # 시작점과 끝점 표시
    if trajectory_points:
        cv2.circle(image, trajectory_points[0], 5, (0, 255, 0), -1)  # 초록색 시작점
        cv2.circle(image, trajectory_points[-1], 5, (255, 0, 0), -1)  # 파란색 끝점

    # 이미지 파일로 저장
    cv2.imwrite(filename, image)
    rospy.loginfo(f"Trajectory image saved to {filename}")

def main():
    rospy.init_node("trajectory_saver", anonymous=True)

    # Odometry 콜백 구독
    rospy.Subscriber("/odom", Odometry, odometry_callback)

    rospy.spin()

    # 프로그램 종료 시 트래젝토리 이미지 저장
    save_trajectory_image("/home/glab/trajectory_image.png")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
