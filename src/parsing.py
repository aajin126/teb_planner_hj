def parse_manual_plan(input_poses):
    positions = []

    # 개별 Pose를 줄 단위로 분리
    lines = input_poses.strip().split("\n")
    
    for line in lines:
        try:
            # 문자열 파싱
            prefix = "Pose: [Position: ("
            suffix = "), Orientation: "
            start = line.index(prefix) + len(prefix)
            end = line.index(suffix)
            position_str = line[start:end]
            
            # 위치 좌표 (x, y, z) 추출
            x, y, z = map(float, position_str.split(", "))
            positions.append({x, y})  # z는 무시하고 x, y만 추가
        except (ValueError, IndexError) as e:
            print(f"Failed to parse pose: {line.strip()} - {e}")
    
    return positions


# 예제 입력
input_poses = """
Pose: [Position: (-0.426784, 4.42194, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.461211, 4.41846, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.48614, 4.41657, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.511011, 4.41404, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.535795, 4.41076, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.560446, 4.40659, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.584909, 4.40144, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.60911, 4.39517, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.632978, 4.38773, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.65641, 4.37902, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.679485, 4.3694, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.702521, 4.35969, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.725526, 4.3499, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.748472, 4.33998, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.771368, 4.32994, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.794212, 4.31978, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.817076, 4.30967, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.839912, 4.2995, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.862722, 4.28926, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.885549, 4.27907, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.90837, 4.26886, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.931262, 4.25881, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.954282, 4.24906, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-0.977492, 4.23977, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.00091, 4.23103, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.02457, 4.22296, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.04826, 4.21495, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.07193, 4.20693, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.09559, 4.19884, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.11923, 4.1907, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.14284, 4.18247, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.16641, 4.17416, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.18998, 4.16582, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.21354, 4.15746, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.23709, 4.14907, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.26063, 4.14064, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.28415, 4.13216, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.30764, 4.12363, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.33113, 4.11507, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.35461, 4.10648, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.3781, 4.09792, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.40155, 4.08924, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.425, 4.08059, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.44845, 4.07193, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.47192, 4.06332, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.49545, 4.05485, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.51907, 4.04666, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.54281, 4.03883, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.5667, 4.03146, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.59075, 4.02463, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.61483, 4.01791, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.63882, 4.01089, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.66268, 4.00342, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.68635, 3.99537, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.70974, 3.98656, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.73278, 3.97684, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.75561, 3.96667, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.77847, 3.95654, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.80138, 3.94653, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.82432, 3.9366, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.84763, 3.92756, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.86978, 3.91596, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.89234, 3.90519, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.91683, 3.90019, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.94171, 3.89772, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.96671, 3.89803, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-1.9875, 3.89191, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.01249, 3.88999, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.03608, 3.88888, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.05962, 3.88777, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.08307, 3.88555, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.10711, 3.88333, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.13183, 3.87999, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.15671, 3.87555, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.1817, 3.87302, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.2067, 3.87263, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.2317, 3.87275, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.25669, 3.87222, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.28167, 3.87222, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.30657, 3.87222, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.32689, 3.87222, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.35123, 3.87222, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.36351, 3.87222, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.37678, 3.87194, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.38987, 3.87333, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.40788, 3.87444, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.43001, 3.87555, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.45193, 3.87444, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.47358, 3.87333, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.48393, 3.87444, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.50747, 3.8748, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.53208, 3.87918, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.55666, 3.88372, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.5811, 3.88902, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.6047, 3.89727, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.62736, 3.90782, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.64964, 3.91916, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.67338, 3.92701, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.69682, 3.9357, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.72011, 3.94478, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.74326, 3.95422, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.7663, 3.96393, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.78945, 3.97336, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.8124, 3.98327, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.83519, 3.99354, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.85785, 4.00411, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.88042, 4.01487, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.90303, 4.02553, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.92551, 4.03648, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.94787, 4.04766, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.97014, 4.059, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-2.99235, 4.0705, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.01479, 4.08151, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.03721, 4.09256, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.05952, 4.10385, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.08179, 4.1152, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.10503, 4.12442, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.12841, 4.13329, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.15184, 4.14201, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.17532, 4.15057, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.19844, 4.16009, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.22125, 4.17033, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.24446, 4.17962, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.26769, 4.18885, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.29094, 4.19804, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.3142, 4.20721, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.33745, 4.21639, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.36073, 4.22551, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.38405, 4.23452, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.4073, 4.24372, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.4305, 4.25301, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.45364, 4.26248, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.47678, 4.27194, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.52678, 4.32194, 0), Orientation: (0, 0, 0, 1)]
Pose: [Position: (-3.49529, 4.35859, 0), Orientation: (0, 0, 0, 1)]
"""

# 결과 확인
positions = parse_manual_plan(input_poses)
print(positions)
