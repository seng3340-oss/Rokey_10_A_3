"""실제 펜촉 TCP의 측정·등록 절차. 로봇 이동·그리퍼 명령은 호출하지 않는다."""

import json
import math
import os
from pathlib import Path
import tempfile
import time
from uuid import uuid4

from scipy.spatial.transform import Rotation

if __package__:
    from .pen_tcp_calibration import fit_tcp
else:
    from pen_tcp_calibration import fit_tcp


def _save_profile(path, profile):
    """검증 완료한 실측값만 임시 파일을 거쳐 원자적으로 저장한다."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as output:
            temporary = Path(output.name)
            os.chmod(temporary, 0o600)
            json.dump(profile, output, ensure_ascii=False, indent=2)
        os.replace(temporary, path)
    finally:
        if temporary and temporary.exists():
            temporary.unlink()


def _load_profile(path):
    """저장된 측정값을 다시 계산해 임의로 수정되거나 깨진 교정값은 사용하지 않는다."""
    profile = json.loads(Path(path).read_text())
    if profile.get("version") != 1 or not isinstance(profile.get("tcp_name"), str):
        raise ValueError(
            "펜 TCP 교정 파일 형식이 올바르지 않습니다. 새 교정이 필요합니다."
        )
    limit = profile.get("max_error_mm", 0.5)
    if (
        not isinstance(limit, (int, float))
        or not math.isfinite(limit)
        or not 0.5 <= limit <= 2.0
    ):
        raise ValueError("교정 잔차 허용값은 0.5~2mm 범위여야 합니다.")
    computed = fit_tcp(profile["samples"], max_error_mm=limit)
    position = profile.get("tcp_pose_mm_deg", [])
    if len(position) != 6 or not all(math.isfinite(v) for v in position):
        raise ValueError("저장된 펜 TCP 값이 유효하지 않습니다.")
    if math.dist(position[:3], computed["tcp_pose_mm_deg"][:3]) > 1e-5:
        raise ValueError("저장 TCP와 원본 측정값이 다릅니다. 새 교정이 필요합니다.")
    actual = Rotation.from_euler("ZYZ", position[3:], degrees=True)
    expected = Rotation.from_euler("ZYZ", computed["tcp_pose_mm_deg"][3:], degrees=True)
    if math.degrees((expected.inv() * actual).magnitude()) > 1e-5:
        raise ValueError("저장 TCP 방향과 원본 측정값이 다릅니다.")
    return profile


def verify_tcp_selection(session, profile):
    """선택한 이름과 실제 TCP 변환이 계산값과 일치하는지 읽기 전용으로 확인한다."""
    transform = profile["tcp_pose_mm_deg"]
    relative = Rotation.from_euler("ZYZ", transform[3:], degrees=True)
    deadline = time.monotonic() + 2.0
    while True:
        tcp = session.pose()
        details = session.tcp_details(tcp)
        flange = details["flange_pose"]
        rotation = Rotation.from_euler("ZYZ", flange[3:], degrees=True)
        offset = rotation.apply(transform[:3])
        expected = [base + delta for base, delta in zip(flange[:3], offset)]
        actual_rotation = Rotation.from_euler("ZYZ", tcp[3:], degrees=True)
        angle = math.degrees(
            ((rotation * relative).inv() * actual_rotation).magnitude()
        )
        if (
            details["tcp_name"] == profile["tcp_name"]
            and math.dist(tcp[:3], expected) < 0.5
            and angle < 0.5
        ):
            return details
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "TCP 적용 결과가 실측 변환과 일치하지 않습니다. 웹을 시작하지 않습니다."
            )
        time.sleep(0.05)


def ensure_pen_tcp(
    session, profile_path, force=False, answer=input, output=print, trial_profile=None
):
    """플랜지 기준이면 실측 교정을 먼저 진행하고, 성공 후에만 케이크 영점으로 넘어간다.

    기존에 적용된 비영점 TCP는 자동으로 덮어쓰지 않는다. 펜을 새로 잡았거나
    그리퍼 TCP를 쓰고 있었다면 --calibrate-pen-tcp로 명시적으로 다시 교정한다.
    """
    force = force or trial_profile is not None
    details = session.tcp_details(session.pose())
    if not force and not details["tcp_at_flange"]:
        output(
            f"현재 활성 TCP: {details['tcp_name'] or '(이름 없음)'}. 실제 펜촉으로 교정된 설정인지 확인하세요."
        )
        return None
    path = Path(profile_path)
    previous = details["tcp_name"]
    profile = None
    create = True
    if trial_profile is not None:
        profile = _load_profile(trial_profile)
        profile["tcp_name"] = "CakePen_" + uuid4().hex[:12]
        output(f"수작업 측정값을 이용한 시험용 TCP: {profile['tcp_pose_mm_deg']}")
        output(
            f"측정 잔차 {profile['max_residual_mm']:.4f}mm. 실제 펜촉 정확도를 보증하는 값은 아닙니다."
        )
        if (
            answer(
                "측정 당시와 같은 펜 고정 상태이며 시험값을 적용하려면 APPLY (그 외 종료): "
            )
            .strip()
            .upper()
            != "APPLY"
        ):
            raise RuntimeError("시험용 TCP 적용을 취소했습니다.")
    if path.exists() and not force:
        saved = _load_profile(path)
        output(
            f"저장된 펜 교정: {saved['tcp_name']}, 오프셋 {saved['tcp_pose_mm_deg']}"
        )
        choice = (
            answer("펜 고정 상태가 같으면 USE, 다시 측정하려면 NEW (그 외 종료): ")
            .strip()
            .upper()
        )
        if choice == "USE":
            profile, create = saved, False
        elif choice != "NEW":
            raise RuntimeError("펜 TCP 준비를 취소했습니다.")
    if profile is None:
        output("\n실제 펜촉 TCP를 측정합니다. 프로그램은 로봇을 움직이지 않습니다.")
        output(
            "펜을 고정한 채 작업대의 단단한 기준점 하나를 정하세요. 케이크 중심 영점과는 별도 측정입니다."
        )
        instructions = (
            "펜을 수직 아래로 세워 기준점에 펜끝을 가볍게 맞추세요",
            "다른 방향으로 기울여 같은 기준점에 펜끝을 맞추세요",
            "반대 방향으로 기울여 같은 기준점에 펜끝을 맞추세요",
            "앞의 기울임과 다른 방향으로 기울여 같은 기준점에 펜끝을 맞추세요",
        )
        samples = []
        output(
            "측정 동안 수동 모드를 사용하며, 측정 종료·취소 후 기존 모드로 복원합니다."
        )
        with session.manual_tcp_mode():
            for index, instruction in enumerate(instructions, start=1):
                response = answer(
                    f"{index}/4: {instruction}. 준비 후 Enter (CANCEL 취소): "
                )
                if response.strip().upper() in {"CANCEL", "EXIT", "QUIT"}:
                    raise RuntimeError("펜 TCP 측정을 취소했습니다.")
                sample = session.flange_pose()
                samples.append(sample)
                output(f"측정 {index}: {sample}")
        profile = fit_tcp(samples)
        profile["tcp_name"] = "CakePen_" + uuid4().hex[:12]
        output(f"플랜지→펜촉 TCP [mm, deg]: {profile['tcp_pose_mm_deg']}")
        output(
            f"측정 최대 오차: {profile['max_residual_mm']:.4f}mm / 조건수: {profile['condition_number']:.2f}"
        )
        output(
            "첫 자세에서 펜이 수직 아래였는지 확인하세요. 등록 시 수동 모드로 전환하고 원래 모드로 복원합니다."
        )
        if (
            answer("이 교정값을 새 TCP로 등록·적용하려면 APPLY (그 외 종료): ")
            .strip()
            .upper()
            != "APPLY"
        ):
            raise RuntimeError(
                "펜 TCP 등록을 취소했습니다. 로봇 설정은 변경하지 않았습니다."
            )
    try:
        session.apply_pen_tcp(profile, create=create)
        verify_tcp_selection(session, profile)
    except Exception:
        # 이전 TCP 이름을 알고 있으면 선택만 복원한다. 기존 TCP 항목은 삭제하거나
        # 덮어쓰지 않는다. 통신 결과가 불명확하면 주문 처리를 시작하지 않는다.
        if previous:
            try:
                session.apply_pen_tcp({"tcp_name": previous}, create=False)
            except Exception as restore_error:
                output(f"이전 TCP 복원 결과를 확인하지 못했습니다: {restore_error}")
        raise
    _save_profile(path, profile)
    output(f"펜촉 TCP 적용 확인: {profile['tcp_name']} / 저장: {path}")
    output("이제 교정된 펜촉을 케이크 윗면 중앙으로 옮긴 뒤 다음 YES 안내를 따르세요.")
    return profile
