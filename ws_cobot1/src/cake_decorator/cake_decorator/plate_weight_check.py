"""Plate Safe Position 하중 상태 검증.

케이크를 들고 있는 단계(both)에서만 최소 변화량(기본 75g)을 강제한다.
따개/빈 상태는 하중 서비스의 노이즈가 커서 기록만 하고 차단하지 않는다.
따개 파지 여부는 그리퍼 폭/각도 안전예외에서 확인한다.

단위는 kg이며, 실제 서비스 출력의 단위와 저중량 분해능은 실물에서
확인해야 한다.
"""

from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class WeightCheckResult:
    """한 단계 하중 검사 결과.

    cake_delta_kg/cake_min_delta_kg는 both(케이크 보유) 단계에서만 채워진다.
    """

    stage: str
    current_kg: float
    expected_kg: float
    tolerance_kg: float
    passed: bool
    cake_delta_kg: float | None = None
    cake_min_delta_kg: float | None = None


class PlateWeightChecker:
    STAGES = {
        'prepare': [
            ('prepare_opener', 'opener'),
            ('prepare_both', 'both'),
            ('prepare_opener_after_cake', 'opener'),
            ('prepare_end', 'empty'),
        ],
        'finish': [
            ('finish_opener', 'opener'),
            ('finish_both', 'both'),
            ('finish_opener_after_cake', 'opener'),
            ('finish_end', 'empty'),
        ],
    }

    def __init__(
        self,
        robot,
        opener_kg,
        cake_kg,
        tolerance_percent,
        cake_min_delta_kg,
        phase,
        reads=1,
        read_interval=0.0,
    ):
        if phase not in self.STAGES:
            raise ValueError('잘못된 공정')
        if (
            not isinstance(reads, int)
            or isinstance(reads, bool)
            or reads < 1
        ):
            raise ValueError(f'reads는 1 이상의 정수여야 합니다: {reads!r}')
        if (
            not isinstance(read_interval, (int, float))
            or isinstance(read_interval, bool)
            or not math.isfinite(read_interval)
            or read_interval < 0
        ):
            raise ValueError(
                f'read_interval은 0 이상의 유한값이어야 합니다: '
                f'{read_interval!r}'
            )
        for name, value in (
            ('opener_kg', opener_kg),
            ('cake_kg', cake_kg),
            ('tolerance_percent', tolerance_percent),
            ('cake_min_delta_kg', cake_min_delta_kg),
        ):
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(
                    f'{name} 값은 유한한 양수여야 합니다: {value!r}'
                )

        self.robot = robot
        self.opener = float(opener_kg)
        self.cake = float(cake_kg)
        self.percent = float(tolerance_percent)
        self.cake_min_delta = float(cake_min_delta_kg)
        self.phase = phase
        self.reads = int(reads)
        self.read_interval = float(read_interval)
        self.baseline = None
        self.opener_measured = None
        self.index = 0

    def _read(self):
        """지정 횟수만큼 하중을 읽고 평균을 반환한다.

        짧은 시간에 2회 읽어 평균하면 단일 샘플보다 노이즈가 줄어든다.
        개별 샘플은 로그로 남겨 실제 분포를 확인할 수 있게 한다.
        """
        samples = []
        for index in range(self.reads):
            value = self.robot.get_workpiece_weight()
            if (
                value is None
                or not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0
            ):
                raise RuntimeError('하중 측정 실패')
            samples.append(float(value))
            if index + 1 < self.reads:
                time.sleep(self.read_interval)

        if len(samples) > 1:
            logger = getattr(self.robot, '_logger', None)
            if logger is not None:
                logger.info(
                    '하중 샘플: '
                    + ', '.join(f'{value:.4f}' for value in samples)
                )
        return sum(samples) / len(samples)

    def start(self):
        self.baseline = self._read()
        self.index = 0
        return self.baseline

    def check(self, stage):
        stages = self.STAGES[self.phase]
        if (
            self.baseline is None
            or self.index >= len(stages)
            or stages[self.index][0] != stage
        ):
            raise RuntimeError(f'하중 검사 순서 오류: {stage}')

        state = stages[self.index][1]
        current = self._read()

        if state == 'opener':
            # 따개 단독 측정은 노이즈가 커서 차단하지 않고, 케이크 판정의
            # 기준값으로만 저장한다.
            self.opener_measured = current

        if state == 'both':
            if self.opener_measured is None:
                return WeightCheckResult(
                    stage, current, current, 0.0, False, None,
                    self.cake_min_delta,
                )
            cake_delta = current - self.opener_measured
            passed = cake_delta > self.cake_min_delta
            if passed:
                self.index += 1
            return WeightCheckResult(
                stage,
                current,
                self.opener_measured + self.cake_min_delta,
                0.0,
                passed,
                cake_delta,
                self.cake_min_delta,
            )

        # opener/empty 단계는 기록만 하고 통과시킨다.
        load = 0.0 if state == 'empty' else self.opener
        expected = self.baseline + load
        tolerance = (load if load else self.opener) * self.percent / 100.0
        self.index += 1
        return WeightCheckResult(stage, current, expected, tolerance, True)
