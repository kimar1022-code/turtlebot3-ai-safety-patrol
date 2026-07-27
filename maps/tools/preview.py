#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
KeepoutFilter 마스크 미리보기 도구  (순수 파이썬, 외부 의존성 없음)

용도: keepout_mask.pgm 을 터미널에 ASCII 지도로 그려, 금지구역 모양을
      RViz 없이 눈으로 확인한다.
표기: '#' = 금지(픽셀 0),  '.' = 통행가능.
      좌상단이 (px=0, py=0). 상단/좌측에 10단위 눈금 표시.

사용: python3 preview.py [마스크.pgm 경로]
      (생략 시 ~/team_ws/maps/keepout_mask.pgm)
"""

import os
import sys
import io

DEFAULT = os.path.expanduser("~/team_ws/maps/keepout_mask.pgm")
BLOCK = 0


def read_pgm(path):
    """P5 PGM 을 (w, h, bytes) 로 읽는다. (주석 라인 # 건너뜀)"""
    data = open(path, "rb").read()
    f = io.BytesIO(data)

    def tok():
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PGM 헤더 파싱 실패")
            if line.startswith(b"#"):
                continue
            parts = line.split()
            if parts:
                return parts

    magic = tok()[0]
    if magic != b"P5":
        raise ValueError(f"P5(binary PGM) 아님: {magic}")
    dims = tok()
    w, h = int(dims[0]), int(dims[1])
    _maxv = int(tok()[0])
    pix = f.read()
    return w, h, pix


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    w, h, pix = read_pgm(path)

    print(f"마스크: {path}  ({w}x{h})")
    print(f"  '#' = 금지,  '.' = 통행가능,  금지 픽셀 {pix.count(BLOCK)}개\n")

    # 상단 눈금 (10단위)
    header = "     " + "".join(str((c // 10) % 10) if c % 10 == 0 else " "
                               for c in range(w))
    print(header)
    for py in range(h):
        row = "".join("#" if pix[py * w + px] == BLOCK else "."
                      for px in range(w))
        print(f"{py:3d}  {row}")


if __name__ == "__main__":
    main()
