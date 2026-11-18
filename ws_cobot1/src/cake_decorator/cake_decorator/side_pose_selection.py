"""펜축 방향을 유지하면서 도달 가능한 옆면 자세를 조회로 선택한다.

로봇 이동은 하지 않는다. 주입한 역기구학 함수가 목표·원호 경유점·스플라인
표본을 검사한다. 이 검사는 제어기 보간 경로 전체의 충돌 검사를 대신하지 않는다.
"""

from dataclasses import replace
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .drawing_motion import DsrPos


def roll_pose(pose, degrees):
    """TCP 위치·+Z축을 유지하고 펜축 둘레의 회전만 바꾼다."""
    rotation = Rotation.from_euler("ZYZ", pose.to_list()[3:], degrees=True)
    orientation = (rotation * Rotation.from_rotvec([0, 0, math.radians(degrees)])).as_euler("ZYZ", degrees=True)
    return DsrPos(pose.x, pose.y, pose.z, *map(float, orientation))


def turn_commands(commands, degrees):
    """처음 두 수직 접근은 유지하고 옆면 자세를 모두 같은 양만큼 회전한다."""
    return [
        command if index < 2 else replace(
            command,
            pos=roll_pose(command.pos, degrees),
            via=roll_pose(command.via, degrees) if command.via else None,
            spline=[roll_pose(p, degrees) for p in command.spline] if command.spline else None,
        )
        for index, command in enumerate(commands)
    ]


def command_points(commands):
    """실행 순서대로 목표·원호 경유점·스플라인 표본을 열거한다."""
    for command in commands:
        if command.spline:
            yield from command.spline
        elif command.via:
            yield command.via
        yield command.pos


def sampled_command_points(commands):
    """명령 경계와 주요 전환을 보존하고 촘촘한 스플라인 내부 표본만 줄인다.

    시작·끝, 원호 경유점과 모든 명령 목표는 유지한다. 내부 표본은20개 간격,
    누적 이동10mm, 마지막 검사 자세와10도 차이 중 먼저 충족하는 지점에서 검사한다.
    20도 이상 꺾이는 표본도 남긴다. 미검사 구간의 도달 가능성을 보장하지 않는다.
    """
    points, required = [], set()
    for command in commands:
        if command.spline:
            required.add(len(points))
            points.extend(command.spline)
            required.add(len(points) - 1)
        elif command.via:
            required.add(len(points))
            points.append(command.via)
        required.add(len(points))
        points.append(command.pos)
    if not points:
        return []
    selected = []
    last_index = 0
    distance = 0.0
    last_rotation = Rotation.from_euler("ZYZ", points[0].to_list()[3:], degrees=True)
    for index, point in enumerate(points):
        sharp_turn = False
        if index:
            distance += math.dist(points[index-1].to_list()[:3], point.to_list()[:3])
        if 0 < index < len(points)-1:
            incoming = np.array(point.to_list()[:3]) - np.array(points[index-1].to_list()[:3])
            outgoing = np.array(points[index+1].to_list()[:3]) - np.array(point.to_list()[:3])
            length = float(np.linalg.norm(incoming) * np.linalg.norm(outgoing))
            if length > 1e-12:
                sharp_turn = float(np.dot(incoming, outgoing) / length) < math.cos(math.radians(20))
        rotation = Rotation.from_euler("ZYZ", point.to_list()[3:], degrees=True)
        angle = math.degrees((last_rotation.inv() * rotation).magnitude())
        if index in required or sharp_turn or index-last_index >= 20 or distance >= 10.0 or angle >= 10.0:
            selected.append(point)
            last_index, distance, last_rotation = index, 0.0, rotation
    return selected


def select_side_roll(commands, current_joints, inverse, cancelled=lambda: False, report=lambda _: None):
    """현재 관절 해에서 가능한 일정한 펜축 회전을 선택한다.

    inverse는 도달 가능한 목표의 관절각6개를 반환하고 불가능하면 None을 반환한다.
    후보를 시작 자세의 관절 변화량으로 정렬한 뒤 대표 검사점을 통과하는 첫 후보를 쓴다.
    주어진 관절 범위는 배포 M0609 모델의 ±360도, J3 ±150도를 따른다.
    """
    if len(commands) < 3 or commands[2].pen_down:
        raise ValueError("옆면 초기 공중 자세 전환점이 없습니다.")
    limits = [360., 360., 150., 360., 360., 360.]
    cache = {}

    def solve(pose):
        if cancelled():
            raise RuntimeError("옆면 자세 검사 취소")
        key = tuple(round(v, 7) for v in pose.to_list())
        if key not in cache:
            joints = inverse(pose)
            if joints is not None:
                joints = list(map(float, joints))
                if len(joints) != 6 or not all(math.isfinite(v) and abs(v) <= limit for v, limit in zip(joints, limits)):
                    joints = None
            cache[key] = joints
        return cache[key]

    ranked = []
    angles = [0.] + [sign * angle for angle in range(15, 180, 15) for sign in (1., -1.)] + [180.]
    for angle in angles:
        joints = solve(roll_pose(commands[2].pos, angle))
        if joints is None:
            continue
        delta = np.array(joints) - np.array(current_joints)
        # 동등한 각도로 보이더라도 ±360을 넘겨 다시 감는 동작을 가정하지 않는다.
        score = abs(delta[5]) + 0.1 * float(np.linalg.norm(delta))
        ranked.append((score, angle))
    for _, angle in sorted(ranked):
        report(f"옆면 펜축 회전 후보 {angle:+.0f}도: 대표 목표 검사 중")
        candidate = turn_commands(commands, angle)
        previous = list(current_joints)
        joint6 = [previous[5]]
        samples = sampled_command_points(candidate)
        report(f"옆면 사전 검사 축소: {sum(1 for _ in command_points(candidate))}개 → {len(samples)}개")
        for index, point in enumerate(samples):
            joints = solve(point)
            # 이웃 표본 사이에 180도 넘게 돌아야 하는 해는 연속 경로로 채택하지 않는다.
            if joints is None or max(abs(a-b) for a, b in zip(joints, previous)) > 180:
                break
            previous = joints
            joint6.append(joints[5])
            if index and index % 100 == 0:
                report(f"옆면 목표 {index}개 도달 검사 완료")
        else:
            report(f"옆면 자세 선택: 펜축 회전 {angle:+.0f}도, 검사 표본의 예상 J6 범위 {min(joint6):.1f}~{max(joint6):.1f}도")
            return candidate
    raise RuntimeError("현재 관절 해에서 옆면 대표 목표를 통과하는 자세가 없습니다. 이동하지 않습니다.")
