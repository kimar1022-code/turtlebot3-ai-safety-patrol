#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KeepoutFilter 마스크 재생성 도구  (순수 파이썬, 외부 의존성 없음)

용도: 금지구역(KeepoutFilter)을 칠한 keepout_mask.pgm/yaml 을 생성한다.
원리: 55x55 그레이맵을 통행가능(254)으로 채우고, ZONES 구역만 금지(0)로 칠한다.
      mode: scale 이라 occupancy = (255 - 픽셀)/255.
      → 0 = 완전 금지(occupancy 1.0), 254 ≈ 통행가능(0.004).

★ 금지구역을 바꾸려면 아래 ZONES 딕셔너리만 수정 후 실행하면 된다.
  좌표는 픽셀 단위 (px=가로 col, py=세로 row, 둘 다 0부터, 양끝 포함).
  py(row)는 이미지 맨 위가 0.  맵 기준 좌표계는 factory_map_0621 과 동일.

검증: 이 스크립트의 기본 ZONES 는 2026-06-25 작동확인된 마스크와
      바이트 단위로 동일한 결과를 낸다. (픽셀 0 = 126개)
"""

import os
import sys

# ── 맵 스펙 (★새 맵 map.pgm 2026-06-28 14:51 좌표계) ─────────────────
W, H = 52, 52              # 가로, 세로 (픽셀)
RESOLUTION = 0.05          # m/px
ORIGIN = [-0.506, -0.607, 0.0]
FREE = 254                 # 통행가능 픽셀값
BLOCK = 0                  # 금지 픽셀값

# ── 금지구역: 이름 -> (px_min, px_max, py_min, py_max)  양끝 포함 ──────
#   ★새 맵 구조에 대응시킨 제안값 (보고 조정 예정)
ZONES = {
    "Z4": (9, 17,  8, 10),    # 좌상 벽 구조물
    "Z3": (18, 23, 18, 24),   # 중앙 블록
    "Z2": (33, 41, 18, 21),   # 우측 구조물
    "Z1": (27, 39, 31, 34),   # 우하 구조물(왼쪽 다소 넓음 — 조정 가능)
}

# ── 출력 경로 (기본: 실제 마스크 위치). 검증 시 인자로 다른 경로 전달 ──
DEFAULT_DIR = os.path.expanduser("~/team_ws/maps")


def build_mask():
    """W*H 길이의 bytearray 를 만들어 ZONES 를 금지로 칠해 반환."""
    buf = bytearray([FREE]) * (W * H)
    for name, (px0, px1, py0, py1) in ZONES.items():
        for py in range(py0, py1 + 1):
            for px in range(px0, px1 + 1):
                buf[py * W + px] = BLOCK
    return buf


def write_pgm(path, buf):
    """P5(binary) PGM 으로 저장. 헤더는 map_server 표준 포맷."""
    header = f"P5\n{W} {H}\n255\n".encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(bytes(buf))


def write_yaml(path, pgm_name):
    """KeepoutFilter 용 yaml (mode: scale) 저장."""
    text = (
        f"image: {pgm_name}\n"
        f"mode: scale\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, {ORIGIN[2]}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(path, "w") as f:
        f.write(text)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, "keepout_mask.pgm")
    yaml_path = os.path.join(out_dir, "keepout_mask.yaml")

    buf = build_mask()
    write_pgm(pgm_path, buf)
    write_yaml(yaml_path, "keepout_mask.pgm")

    blocked = sum(1 for v in buf if v == BLOCK)
    print(f"[make_keepout] 저장 완료")
    print(f"  pgm : {pgm_path}")
    print(f"  yaml: {yaml_path}")
    print(f"  금지구역 {len(ZONES)}개, 금지 픽셀 {blocked}개 / 전체 {W*H}")


if __name__ == "__main__":
    main()
