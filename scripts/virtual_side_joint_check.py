"""독립된 가상 제어기에서 실물 펜 파지 자세와 옆면 경로의 관절값을 검사한다."""

import argparse
from contextlib import redirect_stdout
from io import StringIO
import json
import os
from pathlib import Path
from types import SimpleNamespace

from real_side_air_test import TIP_MM, make_path, preflight
from real_pen_pick_45 import ROOT, check_step
from system_launcher import RobotSession, configure_discovery


def main():
    """가상 모드·localhost·독립 도메인을 확인한 뒤 가상 관절만 초기화한다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--joints", type=float, nargs=6,
        default=[43.59, .54, 69.87, .09, 109.62, 43.31],
        help="재현할 실물 시작 관절각 6개",
    )
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "187":
        raise SystemExit("이 검사는 ROS_DOMAIN_ID=187 가상 전용입니다.")
    configure_discovery("virtual", os.environ)

    from dsr_msgs2.srv import ConfigCreateTcp, SetCurrentTcp, SetRobotMode
    from cake_decorator.doosan_robot import DoosanRobotClient

    reference = sorted((ROOT / "log/system").glob("*-real-*/reference.json"))[-1]
    settings = SimpleNamespace(
        reference=reference,
        rack_config=ROOT.parent / "ws_cobot_pjt_frustum_review/ws_cobot1/src/cake_decorator/config/workspace_config.yaml",
        tilt_heading_deg=90., jaw_center_offset=[0.,0.,220.],
        height_ratio=.5, gap_mm=1., sweep_deg=360., direction="ccw",
    )
    result = {"reference":str(reference), "start_joints":args.joints, "checks":{}}
    session = RobotSession()
    try:
        session.identity("virtual", "127.0.0.1")
        prefix = "/dsr01/dsr_controller2/"
        session.call(SetRobotMode,prefix+"system/set_robot_mode",SetRobotMode.Request(robot_mode=0))
        session.call(ConfigCreateTcp,prefix+"tcp/config_create_tcp",ConfigCreateTcp.Request(name="JointCheckPen",pos=[*map(float,TIP_MM),0.,0.,0.]))
        session.call(SetCurrentTcp,prefix+"tcp/set_current_tcp",SetCurrentTcp.Request(name="JointCheckPen"))
        session.call(SetRobotMode,prefix+"system/set_robot_mode",SetRobotMode.Request(robot_mode=1))
        robot=DoosanRobotClient(session.node)
        check_step(robot.move_joint(args.joints,speed_deg_s=30.,acc_deg_s2=60.),"가상 초기 자세")
        initial=session.pose()
        result["start_tcp"]=initial
        for direction in ("cw","ccw"):
            settings.direction=direction
            entry,samples,exit_path,_,summary=make_path(settings,initial)
            output=StringIO()
            status="passed"
            try:
                with redirect_stdout(output):
                    preflight(robot,[*entry,*samples,*exit_path],initial)
            except RuntimeError as exc:
                status=str(exc)
            result["checks"][direction]={"status":status,"details":output.getvalue(),"cake":summary}
            print(f"{direction}: {status}\n{output.getvalue()}",flush=True)
    finally:
        session.close()
        folder=ROOT/"log/virtual_side_joint_check"
        folder.mkdir(parents=True,exist_ok=True)
        path=folder/"result.json"
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(f"결과 {path}")


if __name__=="__main__":
    main()
