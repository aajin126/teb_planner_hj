#!/usr/bin/env python3
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ===== 사용자에 맞게 설정 =====
NX = 310          # grid의 가로 크기 (C++ 코드의 nx와 동일)
NY = 291         # grid의 세로 크기 (C++ 코드의 ny와 동일)
POT_HIGH = 1.0e10  # C++에서 potarr 초기값 (필요에 따라 수정)

LOG_FILENAME = "navfn_debug.log"   # C++에서 생성된 로그 파일 이름

# ===== 정규표현식 패턴 =====
# Cycle 헤더: "Cycle {cycle_num}: curT = {curT}, curPe = {curPe}, nc = {nc}, nwv = {nwv}"
cycle_re = re.compile(r"^Cycle\s+(\d+):")

# 업데이트 라인: "  -> Updated cell {cell} ({x}, {y}): new pot = {new_pot} (with added heuristic {heuristic})"
update_re = re.compile(
    r"^\s*-> Updated cell\s+(\d+)\s+\((\d+),\s*(\d+)\):\s+new pot\s*=\s*([\-+eE\d\.]+)\s+\(with added heuristic\s+([\-+eE\d\.]+)\)"
)

# (옵션) priority 버퍼에 있는 셀 상태를 나타내는 라인도 있으나,
# 최종 grid state를 결정하는 데는 "-> Updated cell" 라인만 사용합니다.

# ===== 로그 파일 읽기 및 snapshot 누적 =====
# 초기 grid 상태는 모두 POT_HIGH로 설정
current_grid = np.full((NY, NX), POT_HIGH, dtype=float)
snapshots = []       # 각 cycle의 grid 상태 (누적 업데이트한 값)를 저장할 리스트
cycle_numbers = []   # cycle 번호 기록

# 파일을 한 줄씩 읽어가면서, cycle 헤더를 만나면 snapshot을 저장하고,
# "-> Updated cell" 라인에서 해당 셀의 potential 값을 업데이트하도록 합니다.
with open(LOG_FILENAME, "r") as f:
    lines = f.readlines()

for line in lines:
    # 만약 cycle 헤더 라인이라면 현재까지의 grid 상태를 snapshot으로 저장
    cycle_match = cycle_re.match(line)
    if cycle_match:
        cycle_num = int(cycle_match.group(1))
        snapshots.append(current_grid.copy())
        cycle_numbers.append(cycle_num)
        continue

    # "-> Updated cell" 라인 처리
    update_match = update_re.match(line)
    if update_match:
        # 그룹 1: cell index (사용하지 않고 좌표로 바로 사용), 그룹 2: x, 그룹 3: y, 그룹 4: new potential
        x = int(update_match.group(2))
        y = int(update_match.group(3))
        new_pot = float(update_match.group(4))
        # grid 범위 내이면 업데이트
        if 0 <= x < NX and 0 <= y < NY:
            current_grid[y, x] = new_pot
        continue

# 최종 grid 상태도 snapshot에 추가 (마지막 cycle의 결과)
snapshots.append(current_grid.copy())
if cycle_numbers:
    cycle_numbers.append(cycle_numbers[-1] + 1)
else:
    cycle_numbers.append(0)

print(f"총 {len(snapshots)} cycle snapshot이 수집되었습니다.")

# ===== 애니메이션 생성 =====
fig, ax = plt.subplots(figsize=(6, 6))
# grid를 heatmap으로 표시; origin='lower'로 (0,0)이 왼쪽 아래가 되도록 함.
im = ax.imshow(snapshots[0], cmap='viridis', origin='lower', interpolation='nearest')
cbar = fig.colorbar(im)
title_text = ax.set_title(f"Cycle {cycle_numbers[0]}")

# 만약 grid의 각 셀에 숫자도 표시하고 싶다면, 아래와 같이 텍스트를 추가할 수도 있습니다.
# (매 cycle마다 텍스트 갱신은 느릴 수 있으니, 필요에 따라 주석 해제)
#texts = [[ax.text(j, i, f"{snapshots[0][i, j]:.0f}", ha="center", va="center", color="w", fontsize=6)
#           for j in range(NX)] for i in range(NY)]

def update(frame):
    im.set_data(snapshots[frame])
    title_text.set_text(f"Cycle {cycle_numbers[frame]}")
    # (옵션) 각 셀의 숫자 텍스트 업데이트
    # for i in range(NY):
    #     for j in range(NX):
    #         texts[i][j].set_text(f"{snapshots[frame][i, j]:.0f}")
    return [im, title_text]  # + sum(texts, [])

ani = animation.FuncAnimation(fig, update, frames=len(snapshots), interval=500, blit=False)

# gif 파일로 저장 (PillowWriter 사용)
ani.save("navfn_animation.gif", writer="pillow", fps=2)
print("애니메이션이 navfn_animation.gif로 저장되었습니다.")

plt.show()