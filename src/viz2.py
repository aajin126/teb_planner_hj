#!/usr/bin/env python3
import re
import numpy as np
import matplotlib.pyplot as plt

# 맵 크기 (C++ 코드의 nx, ny)
NX = 310   # 가로 크기
NY = 291   # 세로 크기

# C++에서 생성된 로그 파일 이름
LOG_FILENAME = "navfn_debug.log"

# 정규표현식 패턴
# 예: 
# updateCellAstar: Cell 72689 (149, 234): left = 0.000000, right = 10000000000.000000, up = 10000000000.000000, down = 10000000000.000000
pattern = re.compile(
    r"updateCellAstar:\s*Cell\s+\d+\s+\((\d+),\s*(\d+)\):\s+left\s*=\s*([\-+Ee\d\.]+),\s*"
    r"right\s*=\s*([\-+Ee\d\.]+),\s*up\s*=\s*([\-+Ee\d\.]+),\s*down\s*=\s*([\-+Ee\d\.]+)"
)

# 빈 리스트에 좌표를 저장
current_cells = []   # 빨간색: 각 줄에서 읽은 현재 셀 (x,y)
neighbor_cells = []  # 노란색: cost가 가장 낮은 방향의 이웃 셀 (계산된 좌표)

# 파일 읽기
with open(LOG_FILENAME, "r") as f:
    lines = f.readlines()

for line in lines:
    line = line.strip()
    m = pattern.match(line)
    if m:
        # 추출: 그룹 1: x, 그룹 2: y, 그룹 3: left, 4: right, 5: up, 6: down
        x = int(m.group(1))
        y = int(m.group(2))
        cost_left  = float(m.group(3))
        cost_right = float(m.group(4))
        cost_up    = float(m.group(5))
        cost_down  = float(m.group(6))
        
        # 현재 셀 좌표 저장 (빨간색)
        current_cells.append( (x, y) )
        
        # 네 방향의 cost를 딕셔너리로 만듦
        costs = {
            'left': cost_left,
            'right': cost_right,
            'up': cost_up,
            'down': cost_down
        }
        # 가장 낮은 cost의 방향 선택
        min_dir = min(costs, key=costs.get)
        
        # 선택된 이웃의 좌표 계산 (단, 맵 경계를 벗어나지 않도록)
        if min_dir == 'left':
            nx = x - 1 if x - 1 >= 0 else x
            ny = y
        elif min_dir == 'right':
            nx = x + 1 if x + 1 < NX else x
            ny = y
        elif min_dir == 'up':
            nx = x
            ny = y - 1 if y - 1 >= 0 else y
        elif min_dir == 'down':
            nx = x
            ny = y + 1 if y + 1 < NY else y
        else:
            nx, ny = x, y  # 기본적으로 현재 셀
        
        neighbor_cells.append( (nx, ny) )
    # else: 무시

# --- 시각화 ---
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_title("로그 파일 기반 경로 시각화\n빨간: 현재 셀, 노란: 선택된(최저 cost) 이웃")
ax.set_xlim(-1, NX)
ax.set_ylim(-1, NY)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.invert_yaxis()  # (필요시, C++ 좌표계와 맞추기 위해)

# 플롯 배경에 격자 그리기
ax.set_xticks(np.arange(0, NX, 20))
ax.set_yticks(np.arange(0, NY, 20))
ax.grid(True, linestyle='--', alpha=0.5)

# 각 로그 줄마다: 현재 셀 (빨간), 선택된 이웃 (노란), 그리고 화살표 표시
for (cur, neigh) in zip(current_cells, neighbor_cells):
    # 현재 셀: 빨간색 원
    ax.plot(cur[0], cur[1], 'o', color='red', markersize=2)
    # 선택된 이웃: 노란색 원
    ax.plot(neigh[0], neigh[1], 'o', color='yellow', markersize=2)
    # 화살표: 현재 셀에서 선택된 이웃으로
    ax.arrow(cur[0], cur[1], neigh[0]-cur[0], neigh[1]-cur[1],
             head_width=1.5, head_length=2, fc='black', ec='black', length_includes_head=True)
    plt.pause(1)     # 1초 대기

plt.tight_layout()
plt.show()
