"""실제 펜의 동일점 접촉 자세만으로 TCP를 식별하는 순수 수학 모듈.

부모 위저드는 첫 접촉에서 실제 펜을 수직 아래로 정렬하고 모든 접촉이
같은 고정점에서 이루어졌음을 운영자에게 확인받아야 한다. 한 자세만으로
그 물리적 정렬을 인증할 수 없으며, 작은 잔차도 이 가정을 증명하지 않는다.
CAD 오프셋, 로봇 SDK, 장비 통신, 파일 저장은 이 모듈의 책임이 아니다.
"""

from numbers import Real
import warnings

import numpy as np
from scipy.spatial.transform import Rotation


# [R, -I]의 두 블록은 모두 무차원이다. 조건수 50은 회전 변화가 부족할 때
# 측정 오차가 과도하게 증폭되는 해를 거부하기 위한 보수적인 식별성 기준이다.
_MAX_CONDITION_NUMBER = 50.0
# 제어기 범위는 성분별 범위보다 보수적인 벡터 길이로 적용한다(mm).
_MIN_OFFSET_MM = 1.0
_MAX_OFFSET_MM = 10000.0
# mm 단위 1e-9는 측정 오차 허용치가 아니라 최소제곱 경계 비교의 반올림 여유다.
_OFFSET_ROUNDOFF_MM = 1e-9


def fit_tcp(samples, max_error_mm=0.5) -> dict:
    """접촉 수집 후 호출하여 JSON 직렬화 가능한 버전 1 보정값을 반환한다.

    samples는 최소 4개의 BASE 기준 플랜지 자세 [x,y,z,A,B,C] 리스트다.
    위치 단위는 mm, 각도는 도 단위 Doosan 내재적 대문자 'ZYZ'이다.
    모든 자세에서 실제 펜 끝이 동일한 고정 기준점에 접촉해야 한다.

    첫 실제 펜이 수직 아래를 향한다는 운영자 확인을 전제로, 첫 자세의
    TCP +Z를 BASE [0,0,-1]에 맞춘다. TCP X는 첫 플랜지 X의 XY 투영이며,
    투영이 사라지면 플랜지 Y를 사용한다. 이는 플랜지 Z와 펜 축이 같다는
    가정이 아니다. 알고리즘은 한 자세로 실제 펜의 수직 여부를 인증하지 못한다.

    [R_i,-I][t,c] = -p_i를 풀어 플랜지 좌표계 오프셋 t와 BASE 기준점 c를
    동시에 구한다. max_error_mm는 각 접촉점의 3차원 거리 상한이며 0 이상이다.
    잘못된 입력, rank 6 미달, 조건수 50 초과, 오프셋 길이 (1,10000] mm 이탈,
    최대 잔차 초과는 ValueError다. 길이 경계의 수치 반올림은 1e-9 mm 이내에서
    처리하며, 하한에 그만큼 근접한 해는 보수적으로 거부한다. 입력 수정이나
    외부 상태 변경은 하지 않는다.
    """
    # 문자열 변환이나 ragged 배열의 암묵적 수용을 막아 수집 오류를 조기에 알린다.
    if not isinstance(samples, list) or len(samples) < 4:
        raise ValueError("samples must be a list of at least 4 flange poses")
    try:
        raw_poses = np.asarray(samples)
    except (TypeError, ValueError) as exc:
        raise ValueError("each sample must contain six finite real numbers") from exc
    if raw_poses.shape != (len(samples), 6) or raw_poses.dtype.kind not in "fiu":
        raise ValueError("each sample must contain six finite real numbers")
    poses = raw_poses.astype(float, copy=True)
    if not np.isfinite(poses).all():
        raise ValueError("each sample must contain six finite real numbers")
    if (
        isinstance(max_error_mm, (bool, np.bool_))
        or not isinstance(max_error_mm, Real)
        or not np.isfinite(max_error_mm)
        or max_error_mm < 0
    ):
        raise ValueError("max_error_mm must be a finite nonnegative real number")

    # SciPy 소문자 'zyz'는 외재적 회전이므로 반드시 대문자 내재적 ZYZ를 지정한다.
    rotations = Rotation.from_euler("ZYZ", poses[:, 3:], degrees=True)
    rotation_matrices = rotations.as_matrix()
    design = np.concatenate(
        (rotation_matrices, np.broadcast_to(-np.eye(3), rotation_matrices.shape)),
        axis=2,
    ).reshape(-1, 6)
    right_hand_side = -poses[:, :3].reshape(-1)
    try:
        solution, _, rank, singular_values = np.linalg.lstsq(
            design, right_hand_side, rcond=None,
        )
    except np.linalg.LinAlgError as exc:
        raise ValueError("TCP least-squares solve did not converge") from exc

    # 동일 자세·단축 회전은 축 방향의 t와 c를 분리할 수 없어 잔차보다 먼저 검사한다.
    if rank != 6:
        raise ValueError("TCP calibration requires rank 6; vary flange rotations about multiple axes")
    condition_number = float(singular_values[0] / singular_values[-1])
    if not np.isfinite(condition_number) or condition_number > _MAX_CONDITION_NUMBER:
        raise ValueError("TCP calibration condition number exceeds 50; increase rotation variation")
    if not np.isfinite(solution).all():
        raise ValueError("TCP solution must contain finite values")
    offset = solution[:3]
    reference = solution[3:]
    offset_norm = float(np.hypot.reduce(offset))
    if (
        offset_norm <= _MIN_OFFSET_MM + _OFFSET_ROUNDOFF_MM
        or offset_norm > _MAX_OFFSET_MM + _OFFSET_ROUNDOFF_MM
    ):
        raise ValueError("TCP offset norm must be > 1 mm and <= 10000 mm")
    if offset_norm > _MAX_OFFSET_MM:
        # 정확히 상한인 합성 데이터도 lstsq 반올림으로 초과할 수 있다. 이 극소
        # 초과만 안쪽으로 정규화하고 아래에서 실제 반환 오프셋의 잔차를 재검증한다.
        offset *= np.nextafter(_MAX_OFFSET_MM, 0.0) / offset_norm

    # 잔차는 좌표별 오차가 아니라 BASE로 복원한 접촉점과 공통점 c 사이의 거리다.
    # 유한 입력이라도 극단적인 좌표는 산술 범위를 넘을 수 있으므로 결과도 검사한다.
    with np.errstate(over="ignore", invalid="ignore"):
        point_errors = poses[:, :3] + rotations.apply(offset) - reference
        residuals = np.hypot.reduce(point_errors, axis=1)
    if not np.isfinite(residuals).all():
        raise ValueError("contact residuals must be finite")
    max_residual = float(np.max(residuals))
    if max_residual > max_error_mm:
        raise ValueError(
            f"maximum contact residual {max_residual:.6g} mm exceeds "
            f"max_error_mm={max_error_mm:.6g}",
        )
    # 정규화 후 제곱하여 RMS 중간 연산의 overflow를 피하고, 정확한 영잔차도 보존한다.
    rms_residual = (
        float(max_residual * np.sqrt(np.mean((residuals / max_residual) ** 2)))
        if max_residual > 0
        else 0.0
    )

    # 첫 물리적 수직 정렬에서 펜 축을 정의하고, 수평 X로 축 주위의 회전 자유도를 고정한다.
    tcp_z_base = np.array([0.0, 0.0, -1.0])
    tcp_x_base = rotation_matrices[0, :, 0].copy()
    tcp_x_base[2] = 0.0
    if np.linalg.norm(tcp_x_base) < 1e-10:
        tcp_x_base = rotation_matrices[0, :, 1].copy()
        tcp_x_base[2] = 0.0
    tcp_x_base /= np.linalg.norm(tcp_x_base)
    tcp_y_base = np.cross(tcp_z_base, tcp_x_base)
    desired_base = Rotation.from_matrix(
        np.column_stack((tcp_x_base, tcp_y_base, tcp_z_base)),
    )
    relative_tcp = rotations[0].inv() * desired_base
    # ZYZ 특이점에서는 Euler 표현만 비유일하며 회전 자체는 유효하다.
    # 이 변환에서 예상한 gimbal 경고만 억제하고 호출자의 경고 설정은 복원한다.
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore", message="Gimbal lock detected.*", category=UserWarning,
        )
        tcp_angles = relative_tcp.as_euler("ZYZ", degrees=True)

    # ndarray/NumPy scalar를 노출하지 않고 원래 여섯 값의 사본을 보관한다.
    return {
        "version": 1,
        "tcp_pose_mm_deg": offset.tolist() + tcp_angles.tolist(),
        "reference_point_base_mm": reference.tolist(),
        "max_residual_mm": max_residual,
        "rms_residual_mm": rms_residual,
        "condition_number": condition_number,
        "samples": poses.tolist(),
    }
