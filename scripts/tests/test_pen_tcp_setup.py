"""교정 UI와 설정 적용 순서를 가짜 세션으로 검사한다. 실제 장비는 사용하지 않는다."""

import json
from contextlib import contextmanager
import warnings
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from scripts.pen_tcp_setup import ensure_pen_tcp


def samples():
    """실제 오프셋과 고정 접촉점으로부터 서로 다른 플랜지 측정값을 만든다."""
    point = np.array([400.0, 0.0, 80.0])
    offset = np.array([8.0, -4.0, 273.0])
    rotations = [Rotation.from_euler("ZYZ", [0.0, 180.0, 0.0], degrees=True)]
    for axis, angle in [("x", 25.0), ("y", -25.0), ("x", -25.0)]:
        rotations.append(Rotation.from_euler(axis, angle, degrees=True) * rotations[0])
    poses = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        for rotation in rotations:
            poses.append(
                [
                    *(point - rotation.apply(offset)),
                    *rotation.as_euler("ZYZ", degrees=True),
                ]
            )
    return poses


class FakeSession:
    """조회·설정만 제공한다. 이동 API가 없어 잘못된 자동 이동은 즉시 실패한다."""

    def __init__(self, measured=None):
        self.samples = measured or samples()
        self.index = 0
        self.current = self.samples[0]
        self.active = None
        self.calls = []

    @contextmanager
    def manual_tcp_mode(self):
        self.manual_entered = True
        try:
            yield
        finally:
            self.manual_restored = True

    def pose(self):
        if self.active is None:
            return self.current[:]
        flange = Rotation.from_euler("ZYZ", self.current[3:], degrees=True)
        transform = self.active["tcp_pose_mm_deg"]
        point = np.array(self.current[:3]) + flange.apply(transform[:3])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            angles = (
                flange * Rotation.from_euler("ZYZ", transform[3:], degrees=True)
            ).as_euler("ZYZ", degrees=True)
        return [*point, *angles]

    def tcp_details(self, pose):
        return {
            "tcp_at_flange": self.active is None,
            "tcp_name": self.active["tcp_name"] if self.active else "",
            "flange_pose": self.current[:],
        }

    def flange_pose(self):
        self.current = self.samples[self.index]
        self.index += 1
        return self.current[:]

    def apply_pen_tcp(self, profile, create=True):
        self.calls.append((create, profile.copy()))
        if create:
            self.saved = profile.copy()
        self.active = self.saved.copy()


def answer_sequence(*values):
    sequence = iter(values)
    return lambda prompt: next(sequence)


def test_missing_tcp_measures_applies_verifies_and_saves(tmp_path):
    session = FakeSession()
    path = tmp_path / "local/pen_tcp.json"
    result = ensure_pen_tcp(
        session,
        path,
        answer=answer_sequence("", "", "", "", "APPLY"),
        output=lambda _: None,
    )
    assert session.index == 4
    assert len(session.calls) == 1 and session.calls[0][0]
    assert result["tcp_name"].startswith("CakePen_")
    assert np.allclose(result["tcp_pose_mm_deg"][:3], [8.0, -4.0, 273.0])
    assert json.loads(path.read_text()) == result
    assert path.stat().st_mode & 0o777 == 0o600


def test_cancel_before_apply_never_sets_tcp(tmp_path):
    session = FakeSession()
    with pytest.raises(RuntimeError, match="취소"):
        ensure_pen_tcp(
            session,
            tmp_path / "pen.json",
            answer=answer_sequence("", "", "", "", "NO"),
            output=lambda _: None,
        )
    assert not session.calls
    assert not (tmp_path / "pen.json").exists()


def test_degenerate_measurements_never_reach_apply(tmp_path):
    session = FakeSession(measured=[samples()[0]] * 4)
    with pytest.raises(ValueError):
        ensure_pen_tcp(
            session,
            tmp_path / "pen.json",
            answer=answer_sequence("", "", "", ""),
            output=lambda _: None,
        )
    assert not session.calls
    assert not (tmp_path / "pen.json").exists()


def test_saved_profile_requires_use_and_no_new_measurements(tmp_path):
    path = tmp_path / "pen.json"
    session = FakeSession()
    profile = ensure_pen_tcp(
        session,
        path,
        answer=answer_sequence("", "", "", "", "APPLY"),
        output=lambda _: None,
    )
    session.active = None
    session.calls.clear()
    session.index = 0
    result = ensure_pen_tcp(
        session, path, answer=answer_sequence("USE"), output=lambda _: None
    )
    assert result == profile
    assert session.index == 0
    assert len(session.calls) == 1 and not session.calls[0][0]


def test_existing_tcp_is_not_overwritten_without_force(tmp_path):
    session = FakeSession()
    session.active = {
        "tcp_name": "Existing_Pen",
        "tcp_pose_mm_deg": [0.0, 0.0, 273.0, 0.0, 0.0, 0.0],
    }
    result = ensure_pen_tcp(
        session,
        tmp_path / "pen.json",
        answer=lambda _: pytest.fail("임의 교정 질문"),
        output=lambda _: None,
    )
    assert result is None
    assert not session.calls


def test_apply_acknowledgement_without_actual_selection_never_saves(
    tmp_path, monkeypatch
):
    """설정 API가 성공을 응답해도 실제 TCP가 그대로면 주문 준비를 중단한다."""
    from scripts import pen_tcp_setup as setup

    class NoSelection(FakeSession):
        def apply_pen_tcp(self, profile, create=True):
            self.calls.append((create, profile.copy()))

    robot = NoSelection()
    clock = iter([0.0, 3.0])
    monkeypatch.setattr(setup.time, "monotonic", lambda: next(clock))
    with pytest.raises(RuntimeError, match="일치하지"):
        ensure_pen_tcp(
            robot,
            tmp_path / "pen.json",
            answer=answer_sequence("", "", "", "", "APPLY"),
            output=lambda _: None,
        )
    assert not (tmp_path / "pen.json").exists()


def test_tampered_saved_measurement_is_rejected_before_any_setting(tmp_path):
    robot = FakeSession()
    path = tmp_path / "pen.json"
    ensure_pen_tcp(
        robot,
        path,
        answer=answer_sequence("", "", "", "", "APPLY"),
        output=lambda _: None,
    )
    profile = json.loads(path.read_text())
    profile["tcp_pose_mm_deg"][2] += 10
    path.write_text(json.dumps(profile))
    robot.active = None
    robot.calls.clear()
    with pytest.raises(ValueError, match="다릅니다"):
        ensure_pen_tcp(robot, path, answer=lambda _: "USE", output=lambda _: None)
    assert not robot.calls


def test_cancel_during_manual_measurement_restores_mode_without_setting(tmp_path):
    robot = FakeSession()
    with pytest.raises(RuntimeError, match="취소"):
        ensure_pen_tcp(
            robot,
            tmp_path / "pen.json",
            answer=answer_sequence("CANCEL"),
            output=lambda _: None,
        )
    assert robot.manual_entered and robot.manual_restored
    assert robot.index == 0 and not robot.calls


def test_trial_profile_uses_existing_measurements_after_explicit_apply(tmp_path):
    from scripts.pen_tcp_calibration import fit_tcp

    profile = fit_tcp(samples())
    profile.update(
        tcp_name="Trial", max_error_mm=2.0, quality="manual_trial_unverified"
    )
    source = tmp_path / "trial.json"
    source.write_text(json.dumps(profile))
    robot = FakeSession()
    result = ensure_pen_tcp(
        robot,
        tmp_path / "active.json",
        trial_profile=source,
        answer=answer_sequence("APPLY"),
        output=lambda _: None,
    )
    assert robot.index == 0
    assert len(robot.calls) == 1 and robot.calls[0][0]
    assert result["quality"] == "manual_trial_unverified"
    assert result["max_error_mm"] == 2.0


def test_trial_profile_decline_never_changes_tcp(tmp_path):
    from scripts.pen_tcp_calibration import fit_tcp

    profile = fit_tcp(samples())
    profile.update(tcp_name="Trial", max_error_mm=2.0)
    source = tmp_path / "trial.json"
    source.write_text(json.dumps(profile))
    robot = FakeSession()
    with pytest.raises(RuntimeError, match="취소"):
        ensure_pen_tcp(
            robot,
            tmp_path / "active.json",
            trial_profile=source,
            answer=answer_sequence("NO"),
            output=lambda _: None,
        )
    assert not robot.calls and robot.index == 0
