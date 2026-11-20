"""공유 설정이 확정된 원뿔대 치수인지 확인한다. 실제 DB·로봇은 사용하지 않는다."""

from pathlib import Path
import json

from app.cake import load_cake_settings


def test_shared_settings_preserve_frustum_dimensions():
    """주문 사본은 실제 윗지름·밑지름·높이를 모두 포함해야 한다."""
    cake = load_cake_settings()
    assert cake["top_diameter_mm"] == 113.5
    assert cake["bottom_diameter_mm"] == 119.5
    assert cake["height_mm"] == 79.0
    assert cake["board_width_mm"] == cake["board_depth_mm"] == 179.0
    assert cake["board_thickness_mm"] == 2.0


def test_all_registered_idl_files_exist():
    """삭제된 획 타입을 빌드 목록에 남기는 문제를 실제 소스 파일 기준으로 확인한다."""
    import re

    root = (
        Path(__file__).resolve().parents[2] / "ws_cobot1/src/cake_decorator_interfaces"
    )
    registered = re.findall(
        r'"((?:msg|srv|action)/[^\"]+)"', (root / "CMakeLists.txt").read_text()
    )
    assert all((root / path).is_file() for path in registered)
    stroke = (root / "msg/Stroke2D.msg").read_text()
    assert "uint32 stroke_id" in stroke
    assert "geometry_msgs/Point32[] points" in stroke
    request = (root / "srv/GenerateTrajectory.srv").read_text().split("---")[0]
    assert "canvas_width" in request and "canvas_height" in request
    # 최신 DP/TM 서비스의 필드명은 strokes이며 좌표 단위는 계속 픽셀이다.
    assert "Stroke2D[] strokes" in request
    assert "StrokeMM" not in request
    assert "cake_radius_top" not in request
    assert "cake_radius_bottom" not in request
