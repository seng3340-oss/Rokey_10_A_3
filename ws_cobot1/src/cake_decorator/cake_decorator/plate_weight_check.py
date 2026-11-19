"""Plate Safe Position 하중 상태 검증. 단위는 실물 서비스 출력에서 확인해야 합니다."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class WeightCheckResult:
    stage: str
    current_kg: float
    expected_kg: float
    tolerance_kg: float
    passed: bool

class PlateWeightChecker:
    STAGES = {
        'prepare': [('prepare_opener','opener'), ('prepare_both','both'),
                    ('prepare_opener_after_cake','opener'), ('prepare_end','empty')],
        'finish': [('finish_opener','opener'), ('finish_both','both'),
                   ('finish_opener_after_cake','opener'), ('finish_end','empty')],
    }
    def __init__(self, robot, opener_kg, cake_kg, tolerance_percent, phase):
        if phase not in self.STAGES:
            raise ValueError('잘못된 공정')
        for value in (opener_kg,cake_kg,tolerance_percent):
            if not isinstance(value,(int,float)) or isinstance(value,bool) or not math.isfinite(value) or value<=0:
                raise ValueError('하중/허용오차는 유한한 양수여야 합니다')
        self.robot=robot
        self.opener=float(opener_kg)
        self.cake=float(cake_kg)
        self.percent=float(tolerance_percent)
        self.phase=phase
        self.baseline=None
        self.index=0

    def _read(self):
        value=self.robot.get_workpiece_weight()
        if value is None or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
            raise RuntimeError('하중 측정 실패')
        return float(value)

    def start(self):
        self.baseline=self._read()
        self.index=0
        return self.baseline

    def check(self,stage):
        stages=self.STAGES[self.phase]
        if self.baseline is None or self.index>=len(stages) or stages[self.index][0]!=stage:
            raise RuntimeError(f'하중 검사 순서 오류: {stage}')
        state=stages[self.index][1]
        load={'empty':0.0,'opener':self.opener,'both':self.opener+self.cake}[state]
        expected=self.baseline+load
        tolerance=(load if load else self.opener)*self.percent/100.0
        current=self._read()
        passed=abs(current-expected)<=tolerance
        if passed: self.index+=1
        return WeightCheckResult(stage,current,expected,tolerance,passed)
