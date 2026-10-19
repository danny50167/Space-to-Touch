"""구입 부품의 참고용 3D 모델(STEP)을 만든다.

학생이 Fusion 360에서 홀더·센서 바를 설계할 때 끼워 맞춰 보는 용도다.
실측값이 나오면 아래 숫자만 바꾸고 다시 실행한다 (CadQuery 필요).

좌표: 단위 mm. 센서 칩 윗면 중심이 원점, 센서가 보는 방향이 +Z.
"""

import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).parent

# ───────── VL53L1X 모듈 (판매처 도면) ─────────
VL_W = 20.0             # 둥근 윗부분 폭
VL_LOBE_H = 8.5         # 둥근 윗부분 높이
VL_TAIL_W = 15.0        # 핀 부분 폭
VL_TAIL_H = 3.5         # 핀 부분 높이 (전체 12 mm)
VL_HOLE_D = 2.0         # 장착 구멍 (M2)
VL_HOLE_PITCH = 16.0    # 구멍 중심 간격
VL_PIN_PITCH = 2.54
VL_PIN_ROW_FROM_BOTTOM = 1.0   # 도면 비율로 읽은 값 (실측 확인)
PCB_T = 1.6             # 기판 두께: 가정 (실측 확인)
CHIP = (4.9, 2.5, 1.56) # VL53L1X 칩 (ST 데이터시트)
CHIP_OFFSET = (0.0, 0.0)  # 칩 중심 위치: 부품 영역 가운데로 가정 (실측 확인)

# ───────── 시야 원뿔 (설계 확인용) ─────────
FOV_DEG = 27.0
FOV_LEN = 60.0

# ───────── GY-521 (MPU-6050) ─────────
GY_W, GY_H = 21.2, 16.4
GY_HOLE_D = 3.0
GY_HOLE_PITCH = 15.5         # 판매처 1곳 (실측 확인)
GY_HOLE_FROM_EDGE = 2.6      # 추정 (실측 확인)
GY_PIN_ROW_FROM_EDGE = 1.3   # 추정
GY_CHIP = (4.0, 4.0, 0.9)    # MPU-6050 QFN, 위치는 가운데로 가정


def vl53l1x_board():
    lobe = cq.Workplane("XY").slot2D(VL_W, VL_LOBE_H).extrude(-PCB_T)
    tail_cy = -(VL_LOBE_H / 2 + VL_TAIL_H / 2) + 1.0  # 1 mm 겹치게
    tail = (cq.Workplane("XY").center(0, tail_cy)
            .rect(VL_TAIL_W, VL_TAIL_H + 2.0).extrude(-PCB_T))
    board = lobe.union(tail)
    # 장착 구멍
    board = (board.faces(">Z").workplane()
             .pushPoints([(-VL_HOLE_PITCH / 2, 0), (VL_HOLE_PITCH / 2, 0)])
             .hole(VL_HOLE_D))
    # 핀 구멍 6개
    pin_y = -(VL_LOBE_H / 2 + VL_TAIL_H) + VL_PIN_ROW_FROM_BOTTOM
    pins = [(-2.5 * VL_PIN_PITCH + i * VL_PIN_PITCH, pin_y) for i in range(6)]
    board = board.faces(">Z").workplane().pushPoints(pins).hole(1.0)
    # 칩 (윗면이 Z=0이 되도록 기판을 칩 높이만큼 내린다)
    board = board.translate((0, 0, -CHIP[2]))
    chip = (cq.Workplane("XY").center(*CHIP_OFFSET)
            .rect(CHIP[0], CHIP[1]).extrude(-CHIP[2]))
    return board.union(chip)


def fov_cone():
    r = FOV_LEN * math.tan(math.radians(FOV_DEG / 2))
    solid = cq.Solid.makeCone(0.01, r, FOV_LEN)
    return cq.Workplane("XY").add(solid).translate((*CHIP_OFFSET, 0))


def gy521_board():
    board = cq.Workplane("XY").rect(GY_W, GY_H).extrude(-PCB_T)
    hole_y = GY_H / 2 - GY_HOLE_FROM_EDGE
    board = (board.faces(">Z").workplane()
             .pushPoints([(-GY_HOLE_PITCH / 2, hole_y), (GY_HOLE_PITCH / 2, hole_y)])
             .hole(GY_HOLE_D))
    pin_y = -GY_H / 2 + GY_PIN_ROW_FROM_EDGE
    pins = [(-3.5 * 2.54 + i * 2.54, pin_y) for i in range(8)]
    board = board.faces(">Z").workplane().pushPoints(pins).hole(1.0)
    board = board.translate((0, 0, -GY_CHIP[2]))
    chip = cq.Workplane("XY").rect(GY_CHIP[0], GY_CHIP[1]).extrude(-GY_CHIP[2])
    return board.union(chip)


if __name__ == "__main__":
    cq.exporters.export(vl53l1x_board(), str(OUT / "vl53l1x_module_ref.step"))
    cq.exporters.export(fov_cone(), str(OUT / "vl53l1x_fov_cone_60mm.step"))
    cq.exporters.export(gy521_board(), str(OUT / "gy521_mpu6050_ref.step"))
    print("done")
