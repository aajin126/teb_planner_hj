#!/usr/bin/env python3
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation

# --- 설정 ---
LOG_FILENAME = "path_debug.log"   # 로그 파일 이름
# 표시할 iteration 범위: 0부터 151까지
min_iter = 0
max_iter = 151

# potarr 윈도우 header 패턴
window_header_pattern = re.compile(
    r"Iteration\s+(\d+):\s+potarr window around cell\s+\((\d+),(\d+)\):"
)

# --- 로그 파일에서 모든 potarr 윈도우 데이터 파싱 ---
frames_data = []  # 각 iteration의 윈도우 데이터를 저장할 리스트

with open(LOG_FILENAME, "r") as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i].strip()
    m = window_header_pattern.search(line)
    if m:
        iter_num = int(m.group(1))
        if iter_num < min_iter or iter_num > max_iter:
            i += 1
            continue

        center_x = int(m.group(2))
        center_y = int(m.group(3))
        # 고정 윈도우 크기: center를 기준으로 +/- 5 셀
        win = 5
        start_col = max(0, center_x - win)
        start_row = max(0, center_y - win)
        origin = (start_col, start_row)
        window_data = []
        i += 1  # header 다음 줄부터 데이터 시작
        while i < len(lines):
            curr_line = lines[i].strip()
            if curr_line == "" or curr_line.startswith("Iteration"):
                break
            # 각 행은 공백으로 구분된 숫자 문자열
            row_vals = [float(val) for val in curr_line.split()]
            window_data.append(row_vals)
            i += 1
        if window_data:
            data = np.array(window_data)
            frames_data.append({
                "iteration": iter_num,
                "center": (center_x, center_y),
                "origin": origin,
                "data": data
            })
        continue
    i += 1

if not frames_data:
    print("지정한 iteration 범위(0~151)의 potarr 윈도우 데이터를 찾지 못했습니다.")
    exit(1)

# 정렬: iteration 번호 순으로
frames_data = sorted(frames_data, key=lambda x: x["iteration"])
print("총 {} iteration의 potarr 윈도우 데이터가 파싱되었습니다.".format(len(frames_data)))

# 고정된 center와 origin (첫 번째 프레임의 값 사용)
fixed_center = frames_data[0]["center"]
fixed_origin = frames_data[0]["origin"]

# --- 색상 매핑 설정 ---
obstacle_val = 10000000000.0

def get_colored_data(data):
    """
    data: 2D numpy array of potarr values.
    장애물 값(obstacle_val)은 빨간색으로 표시할 마스크를 생성하고,
    나머지 영역은 낮은 값이 흰색, 높은 값이 검정색 (gray_r)으로 표시합니다.
    """
    mask_obstacle = (data >= obstacle_val)
    non_obs = data[~mask_obstacle]
    if non_obs.size == 0:
        vmin, vmax = 0, 1
    else:
        vmin, vmax = non_obs.min(), non_obs.max()
    masked_data = np.ma.masked_where(mask_obstacle, data)
    return masked_data, mask_obstacle, vmin, vmax

# --- 애니메이션 설정 ---
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.invert_yaxis()  # (0이 위쪽에 있도록)
ax.grid(True, linestyle="--", linewidth=0.5, color="lightgray")

# x, y 축 눈금 제거
ax.set_xticks([])
ax.set_yticks([])

# extent는 고정된 fixed_origin과 첫 프레임의 데이터 shape를 사용 (모든 프레임의 shape가 동일하다고 가정)
rows, cols = frames_data[0]["data"].shape
fixed_ext = [fixed_origin[0], fixed_origin[0] + cols, fixed_origin[1] + rows, fixed_origin[1]]

# 초기 이미지: 임시로 1x1 영상을 설정
im_non_obs = ax.imshow(np.zeros((1,1)), cmap="gray_r", origin="upper", extent=fixed_ext)
im_obs = ax.imshow(np.zeros((1,1)), cmap=mcolors.ListedColormap(["red"]), origin="upper", extent=fixed_ext)

# 텍스트 객체들을 저장할 리스트 (매 프레임마다 새로 생성)
pot_texts = []

# 현재 위치 marker (초록색 원) -- 고정된 위치(fixed_center)를 사용
curr_marker = ax.plot(fixed_center[0], fixed_center[1], 'o', color="green", markersize=8)[0]

# 제목 텍스트 객체 (blit 모드 업데이트를 위해 별도 생성)
title_obj = ax.text(0.5, 0.95, "", transform=ax.transAxes,
                    ha="center", va="center", color="black", fontsize=12)

def init():
    im_non_obs.set_data(np.zeros((1,1)))
    im_obs.set_data(np.zeros((1,1)))
    for txt in pot_texts:
        txt.remove()
    pot_texts.clear()
    title_obj.set_text("")
    return im_non_obs, im_obs, curr_marker, title_obj

def update(frame_index):
    global pot_texts
    # 현재 frame 데이터
    frame_data = frames_data[frame_index]
    # 여기서는 고정된 위치(fixed_origin)를 사용
    data = frame_data["data"]
    rows, cols = data.shape

    # extent는 고정된 fixed_ext 사용
    im_non_obs.set_data(np.ma.masked_where(data >= obstacle_val, data))
    mask_obstacle = (data >= obstacle_val)
    non_obs = data[~mask_obstacle]
    if non_obs.size == 0:
        vmin, vmax = 0, 1
    else:
        vmin, vmax = non_obs.min(), non_obs.max()
    im_non_obs.set_clim(vmin, vmax)
    im_non_obs.set_extent(fixed_ext)
    
    im_obs.set_data(np.ma.masked_where(~mask_obstacle, data))
    im_obs.set_extent(fixed_ext)
    
    # 이전 potential 텍스트 제거
    for txt in pot_texts:
        txt.remove()
    pot_texts.clear()
    
    # 각 셀에 potential 값 텍스트 overlay (좌표는 고정된 fixed_origin을 기준)
    for r in range(rows):
        for c in range(cols):
            val = data[r, c]
            x_coord = fixed_origin[0] + c
            y_coord = fixed_origin[1] + r
            txt_color = "white" if mask_obstacle[r, c] else "black"
            txt = ax.text(x_coord, y_coord, f"{val:.0f}",
                          ha="center", va="center", fontsize=4, color=txt_color)
            pot_texts.append(txt)
    
    # 제목 업데이트: 현재 iteration 번호를 포함
    title_obj.set_text("Iteration {}".format(frame_data["iteration"]))
    
    return im_non_obs, im_obs, curr_marker, title_obj, *pot_texts

ani = animation.FuncAnimation(fig, update, frames=len(frames_data),
                              init_func=init, interval=600, blit=False)

# GIF 파일로 저장 (fps=1 => 1초 간격)
ani.save("potarr_window.gif", writer="pillow", fps=1)
print("애니메이션이 'potarr_window.gif'로 저장되었습니다.")

plt.tight_layout()
plt.show()
