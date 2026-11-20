"""주문 재개의 단계·표면·기준 자세 계약(v1). 로봇 명령이나 상태를 변경하지 않는다."""

import json
import math

RECOVERABLE_ERRORS = {
    'OPENER_MISSING', 'PEN_GRASP_FAILED', 'PEN_GRASP_RECOVERY_FAILED',
    'CAKE_WEIGHT_FAILED', 'RESUME_NOT_SAFE',
}


def parse_checkpoint(raw, order_id):
    """같은 주문의 안전한 단계 재개 정보만 반환하고 손상·다른 주문·좌표 오류는 거부한다.

    top_pose는 BASE 위치 m 3값과 ROS quaternion 4값, side_pose는 BASE mm·ZYZ deg
    6값이다. 획 도중의 임의 지점은 지원하지 않으며 DRAWING은 펜 파지 전부터 시작한다.
    """
    value = json.loads(raw) if isinstance(raw, str) else raw
    if (
        not isinstance(value, dict)
        or value.get('version') != 1
        or value.get('order_id') != order_id
        or value.get('stage') not in {'PREPARING', 'DRAWING', 'FINISHING'}
        or type(value.get('path_index')) is not int
        or not 0 <= value['path_index'] <= 1
    ):
        raise ValueError('재개 체크포인트의 주문·단계가 유효하지 않습니다.')
    if value['stage'] == 'DRAWING':
        if value.get('surface') not in {'TOP', 'SIDE'}:
            raise ValueError('재개할 그리기 면이 유효하지 않습니다.')
    elif value['path_index'] != 0 or value.get('surface') != '':
        raise ValueError('운반 단계의 재개 정보가 유효하지 않습니다.')
    for key, sizes in (('top_pose', {7}), ('side_pose', {0, 6})):
        pose = value.get(key)
        if not isinstance(pose, list) or len(pose) not in sizes or not all(
            type(number) in (int, float) and math.isfinite(number) for number in pose
        ):
            raise ValueError('재개 기준 자세가 유효하지 않습니다.')
    return value


def checkpoint_json(request, stage, path_index, surface, side_pose):
    """실패 직전의 공정 위치와 주문 기준 자세를 새 JSON 사본으로 고정한다."""
    center = request.cake_params.center
    orientation = request.cake_params.tool_orientation
    value = {
        'version': 1,
        'order_id': request.order_id,
        'stage': stage,
        'path_index': path_index if stage == 'DRAWING' else 0,
        'surface': surface if stage == 'DRAWING' else '',
        'top_pose': [
            center.x, center.y, center.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        ],
        'side_pose': list(side_pose),
    }
    return json.dumps(parse_checkpoint(value, request.order_id), allow_nan=False)
