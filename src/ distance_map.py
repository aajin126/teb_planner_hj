import numpy as np
import cv2
import matplotlib.pyplot as plt

# # 1. 데이터 불러오기 (예: txt 파일 또는 직접 지정)
# # 여기선 예시로 텍스트 파일에서 불러온다고 가정
# data = np.loadtxt('/home/glab/distance_field_20250323_153208.txt')  # 100x100 형태여야 함

# # 2. 정규화: 값 범위를 0~255로 변환
# normalized = cv2.normalize(data, None, 0, 255, cv2.NORM_MINMAX)
# normalized = normalized.astype(np.uint8)

# # 3. 컬러맵 적용 (예: JET, HOT, TURBO 등 사용 가능)
# color_mapped = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

# # 4. 시각화
# cv2.imshow("Heatmap", color_mapped)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# # (선택) 저장
# cv2.imwrite("heatmap_result14.png", color_mapped)

#include <opencv2/opencv.hpp>

# import numpy as np
# import cv2

# # 1. 텍스트 파일에서 costmap 데이터를 읽어오기
# file_path = "/home/glab/costmap_data.txt"
# with open(file_path, 'r') as f:
#     lines = f.readlines()

# # 2. 문자열을 숫자로 변환하여 2D 배열 생성
# data = []
# for line in lines:
#     row = line.strip().split('\t')
#     row = [int(val) if val.isdigit() else 0 for val in row if val != '']  # 'inf' 또는 '' 제거
#     data.append(row)

# costmap_array = np.array(data, dtype=np.uint8)

# # 3. OpenCV로 시각화 (컬러맵 적용)
# colored = cv2.applyColorMap(costmap_array, cv2.COLORMAP_JET)

# # 4. 이미지 저장
# cv2.imwrite('/home/glab/costmap_visualization_python.png', colored)

# print("✅ 저장 완료: /home/glab/costmap_visualization_python.png")

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# --- 1. Distance Field 파일 읽기 ---
# 파일은 각 행이 탭으로 구분된 숫자값을 포함한다고 가정 (shape: [map_height, map_width])
distance_field_path = "/home/glab/distance_field.txt"
distance_field = np.loadtxt(distance_field_path, delimiter='\t')

# --- 2. grid_search_path 데이터 읽기 ---
# grid_search_path.txt 파일은 각 줄에 "x y" (미터 단위) 형식의 점들이 있다고 가정합니다.
grid_search_path_path = "/home/glab/grid_search_path.txt"
grid_search_path = np.loadtxt(grid_search_path_path)

# --- 3. 파라미터 설정 ---
# costmap 혹은 distance field의 해상도 (예: 0.1m per cell)
resolution = 0.5

# --- 4. 애니메이션 설정 ---
fig, ax = plt.subplots(figsize=(8, 6))
# distance_field를 시각화: origin='lower'로 설정하면 좌측 하단이 (0,0)
cax = ax.imshow(distance_field, cmap='gray', origin='lower')
ax.set_title("Signed Distance Map")
fig.colorbar(cax, ax=ax)

ax.set_xticks([])
ax.set_yticks([])

# grid_search_path에 따른 점들을 연결할 선(Line2D) 생성 (초기에는 빈 선)
(line,) = ax.plot([], [], 'w.-', linewidth=2, markersize=6)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    # i번째 프레임까지의 grid_search_path 점들을 사용
    path_so_far = grid_search_path[:i+1]
    # 좌표 변환: costmap의 셀 좌표는 (x/resolution, y/resolution)
    x_coords = path_so_far[:, 0] / resolution
    y_coords = path_so_far[:, 1] / resolution
    line.set_data(x_coords, y_coords)
    return line,

# grid_search_path의 점 개수만큼 프레임 생성 (여기서는 N = number of points)
frames = grid_search_path.shape[0]

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=frames, interval=500, blit=True)

# --- 5. GIF 저장 ---
# ImageMagick이 설치되어 있으면 'imagemagick' writer 사용, 아니면 PillowWriter도 가능
gif_path = "/home/glab/medial_axis_climb.gif"
# PillowWriter 예시:
writer = animation.PillowWriter(fps=2)
anim.save(gif_path, writer=writer)
print("GIF saved to", gif_path)

plt.show()