"""가상 펜촉 관측 JSON을 펼친 원통 옆면 비교 그림으로 저장한다."""

import argparse
import json
from pathlib import Path

import matplotlib

# GUI 창 없이도 검증 보고서 이미지를 만들 수 있도록 파일 출력 모드를 쓴다.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    """회색 원본과 빨간 관측 궤적을 같은 mm 축에서 비교한다. 누락은 채우지 않는다."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input", nargs="?", default="results/side_wave_verification.json"
    )
    parser.add_argument("--output", default="results/side_wave_verification.png")
    args = parser.parse_args()
    results = json.loads(Path(args.input).read_text())["cases"]
    figure, axes = plt.subplots(3, 3, figsize=(12, 8), constrained_layout=True)
    for axis in axes.flat:
        axis.set_visible(False)
    for result in results:
        amplitude, wavelength = result["amplitude_mm"], result["wavelength_mm"]
        row, column = (1.0, 3.0, 6.0).index(amplitude), (4.0, 8.0, 16.0).index(
            wavelength
        )
        axis = axes[row, column]
        axis.set_visible(True)
        start = 113.0 + column * 32
        base = 20.0 + row * 20
        x = np.linspace(0, 24, 1001)
        axis.plot(
            x,
            amplitude * np.sin(2 * np.pi * x / wavelength),
            color=".65",
            linewidth=2,
            label="Original",
        )
        observed = np.asarray(result.get("observed_tip_mm_quaternion", []))
        if len(observed):
            # atan2의 음수 각도를 주문의 0~2π 범위로 복원한다.
            arc = (
                np.mod(np.arctan2(observed[:, 1], observed[:, 0] - 500), 2 * np.pi) * 50
            )
            axis.plot(
                arc - start,
                observed[:, 2] - 62 - base,
                color="#cc2929",
                linewidth=0.9,
                label="Observed tip",
            )
        axis.set_title(f"A={amplitude:g} mm, wavelength={wavelength:g} mm")
        axis.set_xlabel("Unwrapped arc from start (mm)")
        axis.set_ylabel("Height from baseline (mm)")
        axis.set_xlim(-0.5, 24.5)
        axis.set_ylim(-amplitude - 1, amplitude + 1)
        axis.grid(alpha=0.2)
        axis.text(
            0.02,
            0.96,
            f"Max distance: {result['max_wave_error_mm']:.3f} mm",
            transform=axis.transAxes,
            va="top",
            fontsize=8,
        )
    axes[0, 0].legend(loc="lower right", fontsize=8)
    figure.suptitle("Virtual cylinder: planned waveform and observed pen-tip TF")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)
    print(output)


if __name__ == "__main__":
    main()
