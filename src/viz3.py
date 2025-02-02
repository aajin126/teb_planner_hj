#!/usr/bin/env python3
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- 설정 ---
LOG_FILENAME = "path_debug.log"   # C++에서 생성된 로그 파일 이름
NX = 310  # grid의 가로 크기
NY = 291  # grid의 세로 크기

# 정규표현식 패턴
# 1. 셀 정보 (Iteration, cell, x, y, cost, dx, dy)
cell_pattern = re.compile(
    r"Iteration\s+(\d+):\s+cell\s+\d+\s+\(x=([\d\.]+),y=([\d\.]+)\),\s+cost=([\d\.]+),\s+dx=([\d\.-]+),\s+dy=([\d\.-]+)"
)
# 2. Gradient 정보 (Iteration, label, value1, value2)
# label는 stc, stc+1, stcnx, stcnx+1 등
grad_pattern = re.compile(
    r"Iteration\s+(\d+):\s+Gradient at\s+([\w\+\-]+)=\d+:\s+\(([\d\.-]+),([\d\.-]+)\)"
)

# 데이터를 iteration별로 저장 (키: iteration 번호)
steps_by_iter = {}

with open(LOG_FILENAME, "r") as f:
    for line in f:
        line = line.strip()
        # 셀 정보
        m_cell = cell_pattern.search(line)
        if m_cell:
            iter_num = int(m_cell.group(1))
            x = float(m_cell.group(2))
            y = float(m_cell.group(3))
            cost_val = float(m_cell.group(4))
            dx = float(m_cell.group(5))
            dy = float(m_cell.group(6))
            if iter_num not in steps_by_iter:
                steps_by_iter[iter_num] = {}
            steps_by_iter[iter_num].update({
                "iteration": iter_num,
                "x": x,
                "y": y,
                "cost": cost_val,
                "dx": dx,
                "dy": dy
            })
            continue

        # Gradient 정보
        m_grad = grad_pattern.search(line)
        if m_grad:
            iter_num = int(m_grad.group(1))
            label = m_grad.group(2)
            g1 = float(m_grad.group(3))
            g2 = float(m_grad.group(4))
            if iter_num not in steps_by_iter:
                steps_by_iter[iter_num] = {}
            # map label to key
            if label == "stc":
                key = "grad_stc"
            elif label == "stc+1":
                key = "grad_stc1"
            elif label == "stcnx":
                key = "grad_stcnx"
            elif label == "stcnx+1":
                key = "grad_stcnx1"
            else:
                key = "grad_" + label
            steps_by_iter[iter_num][key] = (g1, g2)
            continue

# 정렬된 steps 리스트 (iteration 순서대로)
steps = [steps_by_iter[k] for k in sorted(steps_by_iter.keys())]
print("총 {} iteration의 기록을 읽었습니다.".format(len(steps)))

# --- 애니메이션 설정 ---
fig, ax = plt.subplots(figsize=(10,8))
ax.set_xlim(0, NX)
ax.set_ylim(0, NY)
ax.invert_yaxis()  # (0이 위쪽에 있도록)
ax.set_title("Path Selection Animation\nred: current cell , green line: trajectory\nblue arrow: Gradient vector")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True, linestyle='--', alpha=0.5)

# 누적 경로 저장용 리스트
path_x = []
path_y = []

# 현재 셀(빨간 점)와 누적 경로(초록 점선) 및 텍스트 annotation 생성
current_dot, = ax.plot([], [], 'ro', markersize=8)
path_line, = ax.plot([], [], 'g', linewidth=2)
annotation = ax.text(5, 5, "", color='green', fontsize=10,
                      bbox=dict(facecolor='black', alpha=0.5))

# gradient 화살표들을 저장할 리스트 (매 프레임마다 새로 그림)
grad_arrows = []

# gradient 벡터의 스케일 (조정 가능)
grad_scale = 1

def init():
    path_x.clear()
    path_y.clear()
    current_dot.set_data([], [])
    path_line.set_data([], [])
    annotation.set_text("")
    # 기존 gradient 화살표 제거
    global grad_arrows
    for arrow in grad_arrows:
        arrow.remove()
    grad_arrows = []
    return current_dot, path_line, annotation

def update(frame):
    global grad_arrows
    # 만약 이전 frame의 gradient 화살표가 있다면 제거
    for arrow in grad_arrows:
        arrow.remove()
    grad_arrows = []
    
    step = steps[frame]
    # 누적 경로 업데이트
    path_x.append(step["x"])
    path_y.append(step["y"])
    
    current_dot.set_data(step["x"], step["y"])
    path_line.set_data(path_x, path_y)
    
    text_str = ("Iter: {}\nCost: {:.1f}\ndx: {:.3f}, dy: {:.3f}"
                .format(step["iteration"], step["cost"], step["dx"], step["dy"]))
    annotation.set_text(text_str)
    annotation.set_position((step["x"]+3, step["y"]+3))
    
    # Gradient 화살표 그리기 (만약 해당 정보가 있으면)
    # stc: at (x, y)
    if "grad_stc" in step:
        g = step["grad_stc"]
        arr = ax.arrow(step["x"], step["y"], grad_scale*g[0], grad_scale*g[1],
                       head_width=0.5, head_length=0.5, fc='blue', ec='blue', length_includes_head=True)
        grad_arrows.append(arr)
    # stc+1: at (x+1, y)
    if "grad_stc1" in step:
        g = step["grad_stc1"]
        arr = ax.arrow(step["x"]+1, step["y"], grad_scale*g[0], grad_scale*g[1],
                       head_width=0.5, head_length=0.5, fc='blue', ec='blue', length_includes_head=True)
        grad_arrows.append(arr)
    # stcnx: at (x, y+1)
    if "grad_stcnx" in step:
        g = step["grad_stcnx"]
        arr = ax.arrow(step["x"], step["y"]+1, grad_scale*g[0], grad_scale*g[1],
                       head_width=0.5, head_length=0.5, fc='blue', ec='blue', length_includes_head=True)
        grad_arrows.append(arr)
    # stcnx+1: at (x+1, y+1)
    if "grad_stcnx1" in step:
        g = step["grad_stcnx1"]
        arr = ax.arrow(step["x"]+1, step["y"]+1, grad_scale*g[0], grad_scale*g[1],
                       head_width=0.5, head_length=0.5, fc='blue', ec='blue', length_includes_head=True)
        grad_arrows.append(arr)
    
    return current_dot, path_line, annotation, *grad_arrows

ani = animation.FuncAnimation(fig, update, frames=len(steps),
                              init_func=init, interval=1000, blit=True)

# GIF 파일로 저장 (fps=1, 즉 1초 간격)
ani.save("path_selection.gif", writer='pillow', fps=1)
print("애니메이션이 'path_selection.gif'로 저장되었습니다.")

plt.show()
