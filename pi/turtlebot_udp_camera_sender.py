#!/usr/bin/env python3
"""
Send TurtleBot CSI camera frames to the desktop GlobalCam UDP receiver.

Copy to TurtleBot:
  scp /home/gyul/robot_face_system/tools/turtlebot_udp_camera_sender.py tb3@<TURTLEBOT_IP>:~/

Check camera devices on TurtleBot:
  ls /dev/video*
  v4l2-ctl --list-devices
  v4l2-ctl -d /dev/video0 --list-formats-ext

Run with OpenCV V4L2 camera index (/dev/video0):
  python3 ~/turtlebot_udp_camera_sender.py \
    --backend opencv-v4l2 \
    --camera-index 0 \
    --host <DESKTOP_IP> \
    --port 5005 \
    --width 1280 \
    --height 960 \
    --fps 10 \
    --jpeg-quality 80

Run with an explicit OpenCV V4L2 device path:
  python3 ~/turtlebot_udp_camera_sender.py \
    --backend opencv-v4l2 \
    --camera-device /dev/video0 \
    --host <DESKTOP_IP> \
    --port 5005 \
    --width 1280 \
    --height 960 \
    --fps 10 \
    --jpeg-quality 80

If /dev/video0 does not support 1280x960, use a size from:
  v4l2-ctl -d /dev/video0 --list-formats-ext

For example, if only 1280x720 is available:
  python3 ~/turtlebot_udp_camera_sender.py \
    --backend gstreamer-libcamera \
    --camera-device /dev/video0 \
    --host <DESKTOP_IP> \
    --port 5005 \
    --width 1280 \
    --height 720 \
    --fps 10 \
    --jpeg-quality 80

On the desktop receiver, rx_packets, rx_frames, and input_fps should increase
when UDP sending is working.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import time
from typing import Iterable


MAGIC = b"GCM1"
VERSION = 1
HEADER_FORMAT = "!4sBHIQHHIHHB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

LIBCAMERA_ENV_MARKER = "TURTLEBOT_UDP_LIBCAMERA_ENV"
LOCAL_LIBCAMERA_LIB = "/usr/local/lib/aarch64-linux-gnu"
LOCAL_LIBCAMERA_GST = "/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0"
LOCAL_LIBCAMERA_IPA = "/usr/local/lib/aarch64-linux-gnu/libcamera/ipa"
LOCAL_LIBCAMERA_PROXY = "/usr/local/libexec/libcamera"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a TurtleBot CSI camera stream as GlobalCam UDP chunks."
    )
    parser.add_argument("--host", required=True, help="Desktop receiver IP address.")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument(
        "--extra-host",
        action="append",
        default=[],
        help="추가 수신지 IP(여러 번 지정 가능). --host와 같은 포트로 함께 전송(이중송신). "
             "예: 서버는 --host, 도킹 PC는 --extra-host 로 지정.",
    )
    parser.add_argument(
        "--backend",
        choices=("gstreamer-libcamera", "gstreamer-v4l2", "opencv-v4l2"),
        default="gstreamer-libcamera",
        help="Camera capture backend. Use gstreamer-libcamera for Raspberry Pi CSI cameras.",
    )
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--camera-device",
        default="",
        help="Open a device path directly, for example /dev/video0.",
    )
    parser.add_argument(
        "--rotate-180",
        action="store_true",
        help="캡처 프레임을 180도 회전(카메라가 물리적으로 뒤집혀 장착된 경우). "
             "옛 camera_ros의 orientation:180 대체.",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--chunk-size", type=int, default=1200)
    parser.add_argument("--frame-id", default="turtlebot_camera")
    parser.add_argument("--show-preview", action="store_true")
    parser.add_argument(
        "--no-libcamera-env",
        action="store_true",
        help="Do not auto-apply /usr/local libcamera environment before opening libcamerasrc.",
    )
    return parser.parse_args()


def clamp_int(value: int, minimum: int, maximum: int, name: str) -> int:
    if value < minimum or value > maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}, got {value}")
    return value


def prepend_env_path(env: dict[str, str], key: str, value: str) -> None:
    existing = env.get(key, "")
    parts = [part for part in existing.split(os.pathsep) if part]
    if value not in parts:
        parts.insert(0, value)
    env[key] = os.pathsep.join(parts)


def ensure_libcamera_environment(args: argparse.Namespace) -> None:
    if args.backend != "gstreamer-libcamera" or args.no_libcamera_env:
        return
    if os.environ.get(LIBCAMERA_ENV_MARKER) == "1":
        print("Using libcamera environment already applied by script", flush=True)
        print(f"LD_LIBRARY_PATH={os.environ.get('LD_LIBRARY_PATH', '')}", flush=True)
        print(f"GST_PLUGIN_PATH={os.environ.get('GST_PLUGIN_PATH', '')}", flush=True)
        print(f"LIBCAMERA_IPA_MODULE_PATH={os.environ.get('LIBCAMERA_IPA_MODULE_PATH', '')}", flush=True)
        print(f"LIBCAMERA_IPA_PROXY_PATH={os.environ.get('LIBCAMERA_IPA_PROXY_PATH', '')}", flush=True)
        return

    env = dict(os.environ)
    prepend_env_path(env, "LD_LIBRARY_PATH", LOCAL_LIBCAMERA_LIB)
    prepend_env_path(env, "GST_PLUGIN_PATH", LOCAL_LIBCAMERA_GST)
    env["LIBCAMERA_IPA_MODULE_PATH"] = LOCAL_LIBCAMERA_IPA
    env["LIBCAMERA_IPA_PROXY_PATH"] = LOCAL_LIBCAMERA_PROXY
    env[LIBCAMERA_ENV_MARKER] = "1"

    print("Re-executing with /usr/local libcamera environment for gstreamer-libcamera", flush=True)
    print(f"LD_LIBRARY_PATH={env.get('LD_LIBRARY_PATH', '')}", flush=True)
    print(f"GST_PLUGIN_PATH={env.get('GST_PLUGIN_PATH', '')}", flush=True)
    print(f"LIBCAMERA_IPA_MODULE_PATH={env.get('LIBCAMERA_IPA_MODULE_PATH', '')}", flush=True)
    print(f"LIBCAMERA_IPA_PROXY_PATH={env.get('LIBCAMERA_IPA_PROXY_PATH', '')}", flush=True)
    os.execvpe(sys.executable, [sys.executable, *sys.argv], env)


def fourcc_to_string(value: float) -> str:
    code = int(value)
    chars = [chr((code >> (8 * index)) & 0xFF) for index in range(4)]
    text = "".join(chars)
    if not text.strip("\x00"):
        return "unknown"
    return text


def try_set_fourcc(cap: cv2.VideoCapture, codes: Iterable[str]) -> str:
    for code in codes:
        fourcc = cv2.VideoWriter_fourcc(*code)
        if cap.set(cv2.CAP_PROP_FOURCC, fourcc):
            actual = fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
            print(f"Requested camera FOURCC={code}, actual={actual}", flush=True)
            return actual
    actual = fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
    print(
        f"Warning: could not request MJPG/YUYV FOURCC; continuing with actual={actual}",
        flush=True,
    )
    return actual


def camera_device_from_args(args: argparse.Namespace) -> str:
    if args.camera_device:
        return args.camera_device
    return f"/dev/video{args.camera_index}"


def gstreamer_fps(args: argparse.Namespace) -> int:
    return max(1, int(round(args.fps)))


def open_camera_gstreamer_libcamera(args: argparse.Namespace) -> cv2.VideoCapture:
    fps_num = gstreamer_fps(args)
    pipeline = (
        "libcamerasrc ! "
        f"video/x-raw,width={args.width},height={args.height},framerate={fps_num}/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )
    print("Opening camera with backend=gstreamer-libcamera", flush=True)
    print(f"GStreamer pipeline: {pipeline}", flush=True)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Failed to open camera with gstreamer-libcamera")
    return cap


def open_camera_gstreamer_v4l2(args: argparse.Namespace) -> cv2.VideoCapture:
    device = camera_device_from_args(args)
    fps_num = gstreamer_fps(args)
    pipeline = (
        f"v4l2src device={device} ! "
        f"video/x-raw,width={args.width},height={args.height},framerate={fps_num}/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )
    print("Opening camera with backend=gstreamer-v4l2", flush=True)
    print(f"GStreamer pipeline: {pipeline}", flush=True)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Failed to open camera with gstreamer-v4l2")
    return cap


def open_camera_opencv_v4l2(args: argparse.Namespace) -> cv2.VideoCapture:
    if args.camera_device:
        print(f"Opening camera device {args.camera_device}", flush=True)
        cap = cv2.VideoCapture(args.camera_device, cv2.CAP_V4L2)
    else:
        print(f"Opening camera index {args.camera_index}", flush=True)
        cap = cv2.VideoCapture(args.camera_index, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    try_set_fourcc(cap, ("MJPG", "YUYV"))

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = fourcc_to_string(cap.get(cv2.CAP_PROP_FOURCC))
    print(
        "Camera actual settings: "
        f"width={actual_width} height={actual_height} fps={actual_fps:.2f} "
        f"fourcc={actual_fourcc}",
        flush=True,
    )
    return cap


def open_camera(args: argparse.Namespace) -> cv2.VideoCapture:
    if args.backend == "gstreamer-libcamera":
        return open_camera_gstreamer_libcamera(args)
    if args.backend == "gstreamer-v4l2":
        return open_camera_gstreamer_v4l2(args)
    return open_camera_opencv_v4l2(args)


def encode_jpeg(frame, quality: int) -> bytes | None:
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), quality],
    )
    if not ok:
        return None
    return encoded.tobytes()


def send_frame(
    sock: socket.socket,
    addresses: list,
    frame_seq: int,
    timestamp_ns: int,
    width: int,
    height: int,
    frame_id_bytes: bytes,
    jpeg_bytes: bytes,
    chunk_size: int,
) -> int:
    jpeg_size = len(jpeg_bytes)
    total_chunks = math.ceil(jpeg_size / chunk_size)
    if total_chunks <= 0 or total_chunks > 0xFFFF:
        raise ValueError(f"Invalid total_chunks={total_chunks} for jpeg_size={jpeg_size}")

    header_size = HEADER_SIZE + len(frame_id_bytes)

    # GlobalCam UDP format: fixed big-endian header, UTF-8 frame_id, then JPEG chunk bytes.
    for chunk_index in range(total_chunks):
        start = chunk_index * chunk_size
        end = min(start + chunk_size, jpeg_size)
        chunk = jpeg_bytes[start:end]
        header = struct.pack(
            HEADER_FORMAT,
            MAGIC,
            VERSION,
            header_size,
            frame_seq,
            timestamp_ns,
            width,
            height,
            jpeg_size,
            total_chunks,
            chunk_index,
            len(frame_id_bytes),
        )
        packet = header + frame_id_bytes + chunk
        # 이중송신: 모든 수신지로 같은 패킷 전송
        for address in addresses:
            sock.sendto(packet, address)
    return total_chunks


def main() -> int:
    args = parse_args()
    ensure_libcamera_environment(args)

    global cv2
    import cv2

    if args.fps <= 0:
        raise ValueError("--fps must be greater than 0")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("--width and --height must be greater than 0")
    if args.chunk_size <= 0:
        raise ValueError("--chunk-size must be greater than 0")
    args.jpeg_quality = clamp_int(args.jpeg_quality, 1, 100, "--jpeg-quality")

    frame_id_bytes = args.frame_id.encode("utf-8")
    if len(frame_id_bytes) > 255:
        raise ValueError("--frame-id UTF-8 length must be <= 255 bytes")

    cap = open_camera(args)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
    # 이중송신: --host + --extra-host 전부 같은 포트로
    addresses = [(args.host, args.port)] + [(h, args.port) for h in args.extra_host]

    dests = ", ".join(f"{h}:{p}" for h, p in addresses)
    print(
        f"Sending GlobalCam UDP to [{dests}] "
        f"frame_id={args.frame_id} chunk_size={args.chunk_size}",
        flush=True,
    )
    print(
        "Desktop receiver success indicators: rx_packets, rx_frames, input_fps increase.",
        flush=True,
    )

    frame_seq = 0
    sent_frames = 0
    sent_packets = 0
    capture_failures = 0
    encode_failures = 0
    last_jpeg_size = 0
    last_total_chunks = 0
    inspected_frames = 0
    start_time = time.monotonic()
    last_stats_at = start_time
    next_frame_at = start_time
    frame_period = 1.0 / args.fps

    try:
        while True:
            now = time.monotonic()
            if now < next_frame_at:
                time.sleep(min(next_frame_at - now, 0.01))
                continue
            next_frame_at += frame_period
            if next_frame_at < now - frame_period:
                next_frame_at = now + frame_period

            ok, frame = cap.read()
            if not ok or frame is None:
                capture_failures += 1
                print("Warning: camera capture failed; retrying", flush=True)
                time.sleep(0.05)
                continue

            # 카메라 물리 장착이 뒤집힌 경우 180도 회전(서버 YOLO·ArUco bearing 정상화)
            if args.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)

            height, width = frame.shape[:2]
            if inspected_frames < 10:
                frame_min = int(frame.min())
                frame_max = int(frame.max())
                frame_mean = float(frame.mean())
                frame_std = float(frame.std())
                print(
                    "frame_check "
                    f"index={inspected_frames} "
                    f"width={width} height={height} "
                    f"min={frame_min} max={frame_max} "
                    f"mean={frame_mean:.2f} std={frame_std:.2f}",
                    flush=True,
                )
                if frame_mean < 1.0 and frame_std < 1.0:
                    print(
                        "Warning: captured frame looks black; check camera backend/pipeline",
                        flush=True,
                    )
                inspected_frames += 1

            jpeg_bytes = encode_jpeg(frame, args.jpeg_quality)
            if jpeg_bytes is None:
                encode_failures += 1
                print("Warning: JPEG encode failed; skipping frame", flush=True)
                continue

            timestamp_ns = time.time_ns()
            try:
                total_chunks = send_frame(
                    sock,
                    addresses,
                    frame_seq,
                    timestamp_ns,
                    width,
                    height,
                    frame_id_bytes,
                    jpeg_bytes,
                    args.chunk_size,
                )
            except OSError as exc:
                print(f"Warning: UDP send failed: {exc}", flush=True)
                continue

            frame_seq = (frame_seq + 1) & 0xFFFFFFFF
            sent_frames += 1
            sent_packets += total_chunks
            last_jpeg_size = len(jpeg_bytes)
            last_total_chunks = total_chunks

            if args.show_preview:
                cv2.imshow("TurtleBot UDP Camera Sender", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    print("Preview requested exit", flush=True)
                    break

            now = time.monotonic()
            if now - last_stats_at >= 1.0:
                elapsed = max(now - start_time, 1e-9)
                effective_fps = sent_frames / elapsed
                print(
                    "stats "
                    f"sent_frames={sent_frames} "
                    f"sent_packets={sent_packets} "
                    f"effective_fps={effective_fps:.2f} "
                    f"last_jpeg_size={last_jpeg_size} "
                    f"last_total_chunks={last_total_chunks} "
                    f"capture_failures={capture_failures} "
                    f"encode_failures={encode_failures}",
                    flush=True,
                )
                last_stats_at = now
    except KeyboardInterrupt:
        print("Interrupted; shutting down", flush=True)
    finally:
        cap.release()
        sock.close()
        if args.show_preview:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
