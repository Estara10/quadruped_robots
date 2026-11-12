#!/usr/bin/env python3
"""Finite read-only D435i depth-to-ABS-ray diagnostic.

The live path subscribes to an existing ROS depth topic; it never opens a
camera SDK pipeline, publishes a ray protocol frame, or accesses shared memory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

from ray_predictor import CAM_H, CAM_W
from ray_predictor import RAY_COUNT, RAY_MAX_M, RAY_MIN_M


DEFAULT_DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
DEFAULT_INFO_TOPIC = "/camera/camera/depth/camera_info"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SELF_CHECK_DIR = PROJECT_ROOT / "logs/real_go2/20261111_075040_d435i_new_position_standing"
CENTER_ROI_YX = (slice(115, 155), slice(220, 260))
RAY_ANGLES_DEG = [-45.0 + 9.0 * index for index in range(RAY_COUNT)]


def parse_args():
    parser = argparse.ArgumentParser(description="Finite, read-only depth to ABS ray diagnostics")
    parser.add_argument("--model", type=Path, required=True,
                        help="explicit TorchScript Ray-Pred model; no fallback is used")
    parser.add_argument("--output-dir", type=Path,
                        help="new output directory (required in live mode; existing paths are rejected)")
    parser.add_argument("--duration-s", type=float, default=30.0,
                        help="finite live observation duration, default 30 seconds")
    parser.add_argument("--depth-topic", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("--camera-info-topic", default=DEFAULT_INFO_TOPIC)
    parser.add_argument("--unknown-fill-m", type=float,
                        help="explicit diagnostic-only placeholder for unknown sampled cells; no default")
    parser.add_argument("--offline-self-check", action="store_true",
                        help="check vectorized sampling, finite inference and latest-only slot without ROS/DDS")
    parser.add_argument("--self-check-input-dir", type=Path, default=DEFAULT_SELF_CHECK_DIR)
    return parser.parse_args()


def validate_args(args):
    if not args.model.is_file():
        raise FileNotFoundError("explicit model does not exist: {}".format(args.model))
    if args.unknown_fill_m is not None and (
            not math.isfinite(args.unknown_fill_m) or
            not RAY_MIN_M <= args.unknown_fill_m <= RAY_MAX_M):
        raise ValueError("--unknown-fill-m must be finite and within the model range [0.1, 6.0]")
    if args.offline_self_check:
        if not args.self_check_input_dir.is_dir():
            raise FileNotFoundError("self-check input directory does not exist")
        return
    if args.output_dir is None:
        raise ValueError("--output-dir is required in live mode")
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        raise ValueError("--duration-s must be positive and finite")
    if args.output_dir.exists():
        raise FileExistsError("output directory already exists: {}".format(args.output_dir))


def vectorized_center_sample(depth_m: np.ndarray, source_valid: np.ndarray):
    """Return the existing pixel-center samples plus strict and candidate masks."""
    height, width = depth_m.shape
    if (height, width) != (270, 480):
        raise ValueError("this field observer expects the measured 480x270 profile")
    if source_valid.shape != depth_m.shape:
        raise ValueError("depth and validity mask shapes differ")
    ys = ((2 * np.arange(CAM_H, dtype=np.int64) + 1) * height) // (2 * CAM_H)
    xs = ((2 * np.arange(CAM_W, dtype=np.int64) + 1) * width) // (2 * CAM_W)
    ys = np.minimum(ys, height - 1)
    xs = np.minimum(xs, width - 1)
    sampled = depth_m[np.ix_(ys, xs)]
    candidate_valid = source_valid[np.ix_(ys, xs)]
    strict_valid = source_valid.reshape(CAM_H, 3, CAM_W, 3).all(axis=(1, 3))
    return sampled, strict_valid, candidate_valid


def prepare_input(depth_m, source_valid, unknown_fill_m):
    sampled_m, strict_valid, candidate_valid = vectorized_center_sample(depth_m, source_valid)
    if not candidate_valid.all() and unknown_fill_m is None:
        return None, strict_valid, candidate_valid
    model_m = sampled_m.copy()
    if unknown_fill_m is not None:
        model_m[~candidate_valid] = np.float32(unknown_fill_m)
    depth_log = np.log2(np.clip(model_m, RAY_MIN_M, RAY_MAX_M)).astype(np.float32)
    return depth_log, strict_valid, candidate_valid


def load_model(model_path):
    import sys

    user_torch_path = Path.home() / ".local/lib/python{}/site-packages".format(
        "{}.{}".format(*sys.version_info[:2]))
    if user_torch_path.is_dir() and str(user_torch_path) not in sys.path:
        # Keep system ROS/NumPy first; append only the already-used local Torch location.
        sys.path.append(str(user_torch_path))
    import torch

    torch.set_num_threads(1)
    model = torch.jit.load(str(model_path.resolve()), map_location="cpu").eval()
    warmup_started_ns = time.monotonic_ns()
    with torch.inference_mode():
        model(torch.zeros((1, 3, CAM_H, CAM_W), dtype=torch.float32))
    return model, (time.monotonic_ns() - warmup_started_ns) / 1e6


def infer_log_depth(model, depth_log):
    import torch

    tensor = torch.frombuffer(bytearray(depth_log.tobytes()), dtype=torch.float32)
    tensor = tensor.reshape(CAM_H, CAM_W).unsqueeze(0).unsqueeze(0).repeat(1, 3, 1, 1)
    if tuple(tensor.shape) != (1, 3, CAM_H, CAM_W):
        raise RuntimeError("model input shape must be (1, 3, 90, 160)")
    started_ns = time.monotonic_ns()
    with torch.inference_mode():
        output = model(tensor)
    ended_ns = time.monotonic_ns()
    if not isinstance(output, torch.Tensor) or tuple(output.shape) != (1, RAY_COUNT):
        raise RuntimeError("model output must be (1, 11)")
    rays_log2 = np.asarray(output[0].detach().cpu().tolist(), dtype=np.float32)
    if not np.isfinite(rays_log2).all():
        raise RuntimeError("model returned non-finite ray values")
    rays_m = np.exp2(np.clip(rays_log2, np.log2(RAY_MIN_M), np.log2(RAY_MAX_M)))
    return rays_m.astype(np.float32), (ended_ns - started_ns) / 1e6


class LatestFrameSlot:
    """One pending item; replacement counts discarded pending frames."""

    def __init__(self):
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._overwritten_count = 0

    @property
    def overwritten_count(self):
        with self._condition:
            return self._overwritten_count

    def put(self, item):
        with self._condition:
            if self._closed:
                return False
            if self._pending is not None:
                self._overwritten_count += 1
            self._pending = item
            self._condition.notify()
            return True

    def take(self):
        with self._condition:
            while self._pending is None and not self._closed:
                self._condition.wait()
            if self._pending is None:
                return None
            item = self._pending
            self._pending = None
            return item

    def pending(self):
        with self._condition:
            return self._pending is not None

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()


def _queue_self_check():
    slot = LatestFrameSlot()
    processing_started = threading.Event()
    release_processing = threading.Event()
    processed = []

    def slow_worker():
        first = slot.take()
        processed.append(first)
        processing_started.set()
        release_processing.wait(timeout=2.0)
        newest = slot.take()
        if newest is not None:
            processed.append(newest)
        slot.close()

    worker = threading.Thread(target=slow_worker, name="synthetic-slow-worker", daemon=False)
    worker.start()
    assert slot.put(1)
    assert processing_started.wait(timeout=2.0)
    assert slot.put(2)
    assert slot.put(3)
    assert slot.put(4)
    assert slot.overwritten_count == 2
    assert slot.pending()
    release_processing.set()
    worker.join(timeout=2.0)
    if worker.is_alive():
        slot.close()
        release_processing.set()
        worker.join()
        raise AssertionError("synthetic slow worker failed to exit")
    assert processed == [1, 4]
    assert slot.take() is None
    return {"processed_sequence": processed, "overwritten_pending_frames": 2,
            "repeated_old_frame": False, "pending_after_drain": False}


def offline_self_check(args):
    from realsense_depth_to_abs_rays import downsample_depth_centers

    model, warmup_ms = load_model(args.model)
    filenames = ["depth_000030_ros_mm.npy", "depth_000090_ros_mm.npy",
                 "depth_000150_ros_mm.npy", "depth_last_ros_mm.npy"]
    checks = []
    for filename in filenames:
        raw = np.load(args.self_check_input_dir / filename, allow_pickle=False)
        if raw.dtype != np.uint16 or raw.shape != (270, 480):
            raise ValueError("unexpected saved ROS frame {}: {} {}".format(filename, raw.dtype, raw.shape))
        depth_m = raw.astype(np.float32) * np.float32(0.001)
        valid = raw > 0
        center_roi_valid = valid[CENTER_ROI_YX]
        old_values, old_strict_mask = downsample_depth_centers(depth_m, valid)
        new_values, new_strict_mask, candidate_mask = vectorized_center_sample(depth_m, valid)
        if not np.array_equal(old_values, new_values):
            raise AssertionError("vectorized sampled depth differs from existing adapter: {}".format(filename))
        if not np.array_equal(old_strict_mask, new_strict_mask):
            raise AssertionError("vectorized strict mask differs from existing adapter: {}".format(filename))
        if args.unknown_fill_m is None and not candidate_mask.all():
            status = "skipped_unknown_no_fill_selected"
            rays = None
            inference_ms = None
        else:
            model_input, _, candidate_mask = prepare_input(depth_m, valid, args.unknown_fill_m)
            rays, inference_ms = infer_log_depth(model, model_input)
            status = "diagnostic_prediction_invalid_for_control"
        checks.append({
            "file": filename,
            "center_samples_identical_to_existing_adapter": True,
            "strict_mask_identical_to_existing_adapter": True,
            "raw_zero_fraction": float((~valid).mean()),
            "center_40x40_zero_fraction": float((~center_roi_valid).mean()),
            "strict_unknown_fraction": float((~old_strict_mask).mean()),
            "candidate_center_unknown_fraction": float((~candidate_mask).mean()),
            "candidate_fill_m": args.unknown_fill_m,
            "status": status,
            "inference_ms": inference_ms,
            "rays_m": None if rays is None else [float(value) for value in rays],
            "prediction_valid_for_control": False,
        })
    result = {
        "mode": "offline_self_check_no_ros_no_dds",
        "model_path": str(args.model.resolve()),
        "torch_warmup_ms": warmup_ms,
        "source_directory": str(args.self_check_input_dir.resolve()),
        "frames": checks,
        "latest_slot_check": _queue_self_check(),
        "no_image_saved": True,
        "no_protocol_frame_or_shared_memory": True,
    }
    print(json.dumps(result, indent=2))
    return 0


def _rate(timestamps_ns):
    if len(timestamps_ns) < 2 or timestamps_ns[-1] <= timestamps_ns[0]:
        return None
    return (len(timestamps_ns) - 1) / ((timestamps_ns[-1] - timestamps_ns[0]) * 1e-9)


def _max_gap_ms(timestamps_ns):
    if len(timestamps_ns) < 2:
        return None
    gaps = np.diff(np.asarray(timestamps_ns, dtype=np.int64))
    return float(np.max(gaps) * 1e-6)


def _percentiles(values):
    if not values:
        return {"min": None, "median": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {"min": float(np.min(a)), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def live_observe(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    model, warmup_ms = load_model(args.model)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    slot = LatestFrameSlot()
    rows = []
    receive_times_ns = []
    header_stamps_ns = []
    receive_lock = threading.Lock()
    camera_info_snapshot = {"message": None, "receive_monotonic_ns": None}
    processing_state = {"in_flight": False}
    receive_count = 0
    camera_info_count = 0
    errors = []
    observation_status = "duration_complete"

    def process_loop():
        while True:
            item = slot.take()
            if item is None:
                return
            msg = item["message"]
            row = {
                "receive_index": item["receive_index"],
                "header_sec": int(msg.header.stamp.sec),
                "header_nanosec": int(msg.header.stamp.nanosec),
                "header_stamp_ns": item["header_stamp_ns"],
                "receive_monotonic_ns": item["receive_monotonic_ns"],
                "image_width": int(msg.width),
                "image_height": int(msg.height),
                "image_encoding": msg.encoding,
                "frame_id": msg.header.frame_id,
                "process_start_monotonic_ns": time.monotonic_ns(),
                "process_end_monotonic_ns": None,
                "queue_wait_ms": None,
                "preprocess_ms": None,
                "inference_ms": None,
                "receive_to_completion_ms": None,
                "capture_to_pc_age_ms": None,
                "strict_unknown_fraction": None,
                "candidate_unknown_fraction": None,
                "raw_zero_fraction": None,
                "center_40x40_zero_fraction": None,
                "unknown_fill_m_diagnostic_only": args.unknown_fill_m,
                "status": None,
                "error": "",
            }
            with receive_lock:
                processing_state["in_flight"] = True
            try:
                if msg.encoding != "16UC1" or (msg.height, msg.width) != (270, 480):
                    raise ValueError("expected 16UC1 480x270 input, got {} {}x{}".format(
                        msg.encoding, msg.width, msg.height))
                if msg.step != msg.width * 2 or len(msg.data) != msg.height * msg.step:
                    raise ValueError("unexpected Image step/data length")
                prep_start_ns = time.monotonic_ns()
                dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
                raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
                depth_m = raw.astype(np.float32) * np.float32(0.001)
                source_valid = (raw != 0) & np.isfinite(depth_m)
                row["raw_zero_fraction"] = float((~source_valid).mean())
                row["center_40x40_zero_fraction"] = float((~source_valid[CENTER_ROI_YX]).mean())
                depth_log, strict_valid, candidate_valid = prepare_input(
                    depth_m, source_valid, args.unknown_fill_m)
                prep_end_ns = time.monotonic_ns()
                row["preprocess_ms"] = (prep_end_ns - prep_start_ns) / 1e6
                row["strict_unknown_fraction"] = float((~strict_valid).mean())
                row["candidate_unknown_fraction"] = float((~candidate_valid).mean())
                if depth_log is None:
                    row["status"] = "skipped_unknown_no_placeholder_selected"
                else:
                    rays, infer_ms = infer_log_depth(model, depth_log)
                    row["inference_ms"] = infer_ms
                    row.update({"ray_{:02d}_m".format(index): float(value)
                                for index, value in enumerate(rays)})
                    row["status"] = "diagnostic_prediction_invalid_for_control"
            except Exception as exc:
                row["status"] = "processing_error"
                row["error"] = "{}: {}".format(type(exc).__name__, exc)
                errors.append(row["error"])
            finally:
                row["process_end_monotonic_ns"] = time.monotonic_ns()
                row["queue_wait_ms"] = (row["process_start_monotonic_ns"] - row["receive_monotonic_ns"]) / 1e6
                row["receive_to_completion_ms"] = (row["process_end_monotonic_ns"] - row["receive_monotonic_ns"]) / 1e6
                with receive_lock:
                    rows.append(row)
                    processing_state["in_flight"] = False

    rclpy.init()
    node = rclpy.create_node("go2_depth_ray_readonly_observer")
    subscriptions = []
    observation_start_ns = time.monotonic_ns()

    def on_depth(msg):
        nonlocal receive_count
        rx_ns = time.monotonic_ns()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        with receive_lock:
            receive_count += 1
            receive_times_ns.append(rx_ns)
            header_stamps_ns.append(stamp_ns)
            index = receive_count
        slot.put({"message": msg, "receive_index": index,
                  "receive_monotonic_ns": rx_ns, "header_stamp_ns": stamp_ns})

    def on_camera_info(msg):
        nonlocal camera_info_count
        rx_ns = time.monotonic_ns()
        camera_info_count += 1
        camera_info_snapshot["receive_monotonic_ns"] = rx_ns
        camera_info_snapshot["message"] = {
            "header_stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "width": int(msg.width), "height": int(msg.height),
            "distortion_model": msg.distortion_model,
            "d": [float(value) for value in msg.d],
            "k": [float(value) for value in msg.k],
            "p": [float(value) for value in msg.p],
        }

    subscriptions.append(node.create_subscription(Image, args.depth_topic, on_depth, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(CameraInfo, args.camera_info_topic,
                                                  on_camera_info, qos_profile_sensor_data))
    worker = threading.Thread(target=process_loop, name="depth-ray-inference", daemon=False)
    worker.start()
    deadline_ns = observation_start_ns + int(args.duration_s * 1e9)
    last_print_ns = observation_start_ns
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            rclpy.spin_once(node, timeout_sec=0.1)
            now_ns = time.monotonic_ns()
            if now_ns - last_print_ns >= 5_000_000_000:
                with receive_lock:
                    rx_n, processed_n = receive_count, len(rows)
                    output_n = sum(row["status"] == "diagnostic_prediction_invalid_for_control" for row in rows)
                    in_flight = processing_state["in_flight"]
                print("[depth-ray] rx={} processed={} outputs={} overwritten={} pending={} in_flight={}".format(
                    rx_n, processed_n, output_n,
                    slot.overwritten_count, int(slot.pending()), int(in_flight)), flush=True)
                last_print_ns = now_ns
    except KeyboardInterrupt:
        interrupted = True
        observation_status = "ctrl_c"
    except ExternalShutdownException:
        # rclpy may translate SIGINT-driven context shutdown into this exception
        # instead of KeyboardInterrupt from spin_once(). The finally block below
        # still closes subscriptions, drains the worker, and writes the record.
        observation_status = "ros_context_shutdown"
    finally:
        observation_end_ns = time.monotonic_ns()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        slot.close()  # worker finishes its current inference and drains the last pending frame
        worker.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        _write_live_outputs(args, rows, receive_times_ns, header_stamps_ns,
                            receive_count, camera_info_count, camera_info_snapshot,
                            slot.overwritten_count, errors, warmup_ms,
                            observation_start_ns, observation_end_ns, observation_status, interrupted)
    return 1 if errors else 0


def _write_live_outputs(args, rows, receive_times_ns, header_stamps_ns,
                        receive_count, camera_info_count, camera_info_snapshot,
                        overwritten_count, errors, warmup_ms,
                        start_ns, end_ns, status, interrupted):
    ray_fields = ["ray_{:02d}_m".format(index) for index in range(RAY_COUNT)]
    fields = ["receive_index", "header_sec", "header_nanosec", "header_stamp_ns",
              "image_width", "image_height", "image_encoding", "frame_id",
              "receive_monotonic_ns", "process_start_monotonic_ns", "process_end_monotonic_ns",
              "queue_wait_ms", "preprocess_ms", "inference_ms", "receive_to_completion_ms",
              "capture_to_pc_age_ms", "raw_zero_fraction", "center_40x40_zero_fraction",
              "strict_unknown_fraction", "candidate_unknown_fraction",
              "unknown_fill_m_diagnostic_only", "status", "error"] + ray_fields
    csv_path = args.output_dir / "per_frame.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    outputs = [row for row in rows if row["inference_ms"] is not None]
    outputs_before_observation_end = [row for row in outputs if row["process_end_monotonic_ns"] <= end_ns]
    candidate_unknown = [row["candidate_unknown_fraction"] for row in rows
                         if row["candidate_unknown_fraction"] is not None]
    strict_unknown = [row["strict_unknown_fraction"] for row in rows
                      if row["strict_unknown_fraction"] is not None]
    raw_zero = [row["raw_zero_fraction"] for row in rows if row["raw_zero_fraction"] is not None]
    center_40x40_zero = [row["center_40x40_zero_fraction"] for row in rows
                         if row["center_40x40_zero_fraction"] is not None]
    complete_ms = [row["receive_to_completion_ms"] for row in rows]
    preprocess_ms = [row["preprocess_ms"] for row in rows if row["preprocess_ms"] is not None]
    inference_ms = [row["inference_ms"] for row in rows if row["inference_ms"] is not None]
    queue_wait_ms = [row["queue_wait_ms"] for row in rows]
    stamp_diffs = np.diff(np.asarray(header_stamps_ns, dtype=np.int64)) if len(header_stamps_ns) > 1 else []
    camera_info = camera_info_snapshot["message"]
    camera_info_receive_ns = camera_info_snapshot["receive_monotonic_ns"]
    summary = {
        "status": status,
        "interrupted_by_ctrl_c": interrupted,
        "mode": "read_only_depth_subscription_and_local_inference",
        "prediction_valid_for_control": False,
        "model_path": str(args.model.resolve()),
        "torch_warmup_ms": warmup_ms,
        "expected_input": {"encoding": "16UC1", "width": 480, "height": 270,
                           "unit": "millimeters", "meters_per_unit": 0.001},
        "model_input": {"shape_nchw": [1, 3, CAM_H, CAM_W],
                        "transform": "center sample; clip [0.1,6]m; log2; repeat 3 channels"},
        "validity_candidate": "valid iff sampled center source pixel is nonzero; unknown source centers remain masked",
        "ray_angles_deg_increasing": RAY_ANGLES_DEG,
        "ray_output_unit": "meter, clipped [0.1,6]",
        "depth_topic": args.depth_topic,
        "camera_info_topic": args.camera_info_topic,
        "duration_requested_s": args.duration_s,
        "observation_start_monotonic_ns": start_ns,
        "observation_end_monotonic_ns": end_ns,
        "observation_window_s": (end_ns - start_ns) * 1e-9,
        "depth_received_count": receive_count,
        "frames_processed_count": len(rows),
        "ray_output_count": len(outputs),
        "processing_error_count": len(errors),
        "processing_errors": errors,
        "pending_frame_overwrite_count": overwritten_count,
        "receive_hz": _rate(receive_times_ns),
        "ray_output_hz": _rate([row["process_end_monotonic_ns"] for row in outputs]),
        "max_receive_gap_ms": _max_gap_ms(receive_times_ns),
        "max_ray_output_gap_ms": _max_gap_ms([row["process_end_monotonic_ns"] for row in outputs]),
        "header_duplicate_count": int(np.count_nonzero(stamp_diffs == 0)) if len(stamp_diffs) else 0,
        "header_backward_count": int(np.count_nonzero(stamp_diffs < 0)) if len(stamp_diffs) else 0,
        "latest_receive_age_at_observation_end_ms": (
            (end_ns - receive_times_ns[-1]) * 1e-6 if receive_times_ns else None),
        "latest_output_age_at_observation_end_ms": (
            (end_ns - outputs_before_observation_end[-1]["process_end_monotonic_ns"]) * 1e-6
            if outputs_before_observation_end else None),
        "ray_outputs_completed_after_observation_end_count": len(outputs) - len(outputs_before_observation_end),
        "receive_to_completion_ms": _percentiles(complete_ms),
        "preprocess_ms": _percentiles(preprocess_ms),
        "inference_ms": _percentiles(inference_ms),
        "queue_wait_ms": _percentiles(queue_wait_ms),
        "candidate_unknown_fraction": _percentiles(candidate_unknown),
        "strict_3x3_unknown_fraction": _percentiles(strict_unknown),
        "raw_depth_zero_fraction_processed_frames": _percentiles(raw_zero),
        "center_40x40_zero_fraction_processed_frames": _percentiles(center_40x40_zero),
        "unknown_fill_m_diagnostic_only": args.unknown_fill_m,
        "source_capture_to_pc_age_ms": None,
        "source_capture_to_pc_age_note": "Orin and PC clock alignment is unverified; no subtraction of header and PC monotonic time",
        "camera_info_received_count": camera_info_count,
        "camera_info_latest_receive_age_ms": (
            (end_ns - camera_info_receive_ns) * 1e-6 if camera_info_receive_ns else None),
        "camera_info_latest": camera_info,
        "image_files_saved": 0,
        "protocol_frames_written": 0,
        "shared_memory_accessed": False,
        "control_data_published": False,
        "csv_path": str((args.output_dir / "per_frame.csv").resolve()),
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print("[depth-ray] saved {} and {}; rx={} processed={} outputs={} overwritten={}".format(
        args.output_dir / "per_frame.csv", args.output_dir / "summary.json",
        receive_count, len(rows), len(outputs), overwritten_count), flush=True)


def main():
    args = parse_args()
    validate_args(args)
    if args.offline_self_check:
        return offline_self_check(args)
    return live_observe(args)


if __name__ == "__main__":
    raise SystemExit(main())
