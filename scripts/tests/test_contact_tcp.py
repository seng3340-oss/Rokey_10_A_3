"""상면 접촉 기반 TCP 추정의 좌표 변환과 잘못된 자세 거부를 검사한다."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


path = Path(__file__).parents[1] / "contact_tcp.py"
spec = importlib.util.spec_from_file_location("contact_tcp", path)
contact = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contact)


def test_tip_is_flange_relative_and_keeps_world_orientation():
    """수직 펜의 290mm 오프셋을 등록하면 BASE Z가 510→220이 된다."""
    flange = [400., -55., 510., 10., 180., 20.]
    config, tip = contact.estimate_contact_tcp(flange, flange, 220.)
    rotation = Rotation.from_euler("ZYZ", flange[3:], degrees=True)
    assert rotation.apply(config[:3]) + flange[:3] == pytest.approx(tip[:3])
    assert tip[:3] == pytest.approx([400., -55., 220.])
    assert np.linalg.norm(config[:3]) == pytest.approx(290.)
    assert (rotation * Rotation.from_euler("ZYZ", config[3:], degrees=True)).as_matrix() == pytest.approx(rotation.as_matrix())


def test_existing_tcp_is_replaced_using_flange_not_added_twice():
    """이미 100mm TCP가 있어도 새 등록값은 플랜지부터의 290mm여야 한다."""
    flange = [400., -55., 510., 0., 180., 0.]
    current = [400., -55., 410., 0., 180., 0.]
    config, _ = contact.estimate_contact_tcp(flange, current, 220.)
    assert config[:3] == pytest.approx([0., 0., 290.])


def test_tilted_contact_cannot_calibrate_from_one_height():
    """한 점 측정의 수직 가정이 깨지면 TCP를 등록하기 전에 거부한다."""
    with pytest.raises(ValueError, match="기울어"):
        contact.estimate_contact_tcp([400., 0., 500., 0., 150., 0.],
                                     [400., 0., 500., 0., 150., 0.], 220.)


def test_setting_tcp_never_sends_motion_and_restores_mode(tmp_path):
    """가짜 서비스로 등록·적용·원래 모드 복원만 수행하는지 확인한다."""
    pytest.importorskip("dsr_msgs2.srv")
    from types import SimpleNamespace

    class Session:
        def __init__(self):
            self.active = "Original"
            self.requests = []

        def pose(self):
            z = 510.0 if self.active == "Original" else 220.0
            return [400.0, 0.0, z, 0.0, 180.0, 0.0]

        def call(self, kind, path, request):
            name = kind.__name__
            self.requests.append((name, request))
            assert not name.startswith("Move")
            if name == "GetCurrentToolFlangePosx":
                return SimpleNamespace(pos=[400., 0., 510., 0., 180., 0.])
            if name == "GetCurrentTcp":
                return SimpleNamespace(info=self.active)
            if name == "GetRobotMode":
                return SimpleNamespace(robot_mode=1)
            if name == "GetCurrentPosj":
                return SimpleNamespace(pos=[0.] * 6)
            if name == "SetCurrentTcp":
                self.active = request.name
            return SimpleNamespace(success=True)

    session = Session()
    pose = contact.apply_contact_tcp(session, 220., tmp_path / "contact.json")
    assert pose[2] == 220.
    assert session.active.startswith("CakePen_")
    assert [req.robot_mode for name, req in session.requests if name == "SetRobotMode"] == [0, 1]
