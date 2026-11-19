"""실물 실패 로그의 수직 그리퍼 자세가 최종 두산 명령에서도 보존되는지 검사한다."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cake_decorator.bent_pen import side_gripper_quaternion
from cake_decorator.drawing_motion import quaternion_to_zyz


@pytest.mark.parametrize("angles", [
    (-48.425847821916875,180.,0.),
    (123.66,180.,123.43),
    (90.,180.,-25.),
    (-120.,180.,70.),
    (0.,0.,45.),
    (30.,50.,-60.),
])
def test_rotation_survives_actual_command_conversion(angles):
    """같은 ZYZ 표현인지가 아니라 변환 전후 실제 회전이 같은지를 비교한다."""
    expected=Rotation.from_euler("ZYZ",angles,degrees=True)
    actual=Rotation.from_euler("ZYZ",quaternion_to_zyz(*expected.as_quat()),degrees=True)
    assert (expected.inv()*actual).magnitude()<1e-9


def test_logged_side_target_points_pen_toward_center_after_conversion():
    """문제가 발생한 접근점의 최종 명령이 펜을 중심 쪽으로 향하게 한다."""
    x,y=450.13164996228505,-9.810590975651074
    cx,cy=399.2743225097656,-54.559329986572266
    quat=side_gripper_quaternion(x,y,cx,cy)
    actual=Rotation.from_euler("ZYZ",quaternion_to_zyz(*quat),degrees=True)
    # +Y 방향 45도 파지에서 확인한 TOOL 기준 펜 몸통→펜촉 방향이다.
    pen_local=Rotation.from_euler("ZYZ",[90.,135.,89.77],degrees=True).inv().apply([0.,0.,-1.])
    expected=np.array([cx-x,cy-y,0.])
    expected/=np.linalg.norm(expected)
    pen=actual.apply(pen_local)
    horizontal=pen[:2]/np.linalg.norm(pen[:2])
    np.testing.assert_allclose(horizontal,expected[:2],atol=1e-9)
    assert math.isclose(pen[2],-math.sqrt(.5),abs_tol=1e-9)
