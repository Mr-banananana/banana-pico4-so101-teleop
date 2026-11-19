"""Offline SO101 forward-reach IK diagnostic. Never opens a serial port or commands motors."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mujoco_pick_place.so101.kinematics import Kinematics
from mujoco_pick_place.pico_teleop.real import PhysicalArm

WORK_MIN = PhysicalArm.work_min.copy()
WORK_MAX = PhysicalArm.work_max.copy()


def read_trace(path):
    status = None
    last_input = None
    with Path(path).open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("type") == "tick":
                status = event.get("status")
            elif event.get("type") == "input":
                last_input = event.get("payload")
    return status, last_input


def latest_trace_status(path=None):
    trace_dir = ROOT / "artifacts" / "pico-teleop"
    if path is not None:
        status, last_input = read_trace(path)
        candidates = [(Path(path), status, last_input)]
    else:
        paths = sorted(trace_dir.glob("trace-*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
        candidates = []
        for candidate in paths:
            status, last_input = read_trace(candidate)
            if status and (status.get("hardware") or {}).get("feedback_valid"):
                candidates.append((candidate, status, last_input))
                break
    if not candidates or not candidates[0][1] or not (candidates[0][1].get("hardware") or {}).get("feedback_valid"):
        raise ValueError("trace 中没有有效的真机编码器反馈；不会推测初始姿态")
    return candidates[0]


def solve_path(kin, q_start, step_m=0.002):
    q = np.asarray(q_start, dtype=float).copy()
    start = kin.fk(q)[0]
    safe_z = max(float(start[2]), WORK_MIN[2] + 0.08)
    safe_y = float(np.clip(start[1], WORK_MIN[1] + 0.02, WORK_MAX[1] - 0.02))
    target_x = WORK_MAX[0] - 0.02
    path = []

    def segment(goal):
        nonlocal q
        origin = kin.fk(q)[0]
        distance = float(np.linalg.norm(goal - origin))
        count = max(1, int(np.ceil(distance / step_m)))
        for i in range(1, count + 1):
            point = origin + (goal - origin) * (i / count)
            q = kin.ik_position_step(point, q)
            actual = kin.fk(q)[0]
            error = float(np.linalg.norm(actual - point))
            if error > 0.002:
                raise RuntimeError(f"IK 路径误差过大：{error:.4f} m")
            path.append((point.copy(), actual.copy(), q.copy(), error))

    segment(np.array([start[0], safe_y, safe_z]))
    segment(np.array([target_x, safe_y, safe_z]))
    return start, kin.fk(q)[0], q, path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", help="真机 trace JSONL；默认选最近的有效反馈 trace")
    parser.add_argument("--csv", help="可选：保存逐步 IK 结果 CSV")
    args = parser.parse_args()

    trace, status, last_input = latest_trace_status(args.trace)
    if len(status.get("q_rad", [])) < 5:
        raise ValueError("trace 缺少五个手臂关节角")
    kin = Kinematics()
    start, end, q_end, path = solve_path(kin, status["q_rad"][:5])
    q_start = np.asarray(status["q_rad"][:5], dtype=float)
    logged_tcp = np.asarray(status.get("tcp_m", start), dtype=float)
    print(f"Trace: {trace}")
    print(f"记录状态: {status.get('state')} / {status.get('reason')}")
    print("模型 FK 起点 (m):", np.round(start, 4).tolist())
    print("Trace TCP 起点 (m):", np.round(logged_tcp, 4).tolist())
    print("前向目标 (m):", np.round(end, 4).tolist())
    print("初始关节角 ID1-ID5 (deg):", np.round(np.degrees(q_start), 1).tolist())
    print("IK 预测关节角 ID1-ID5 (deg):", np.round(np.degrees(q_end), 1).tolist())
    print("IK 预测关节角变化 ID1-ID5 (deg):", np.round(np.degrees(q_end-q_start), 1).tolist())
    print(f"路径点: {len(path)}")
    hardware = status.get("hardware") or {}
    prep = hardware.get("workspace_preparation") or {}
    grip = None if not last_input else (last_input.get("right") or {}).get("grip")
    print(f"最新记录握持值: {grip}; 工作区准备: {prep.get('state')} / {prep.get('message')}")
    if status.get("operator_requested_m") is None:
        print("判定: trace 没有活动的手柄末端目标，无法据此判断真机力矩。")
    if not np.allclose(start, logged_tcp, atol=0.003):
        print("判定: 当前工作树模型 FK 与 trace TCP 不一致；本结果只用于软件 IK 诊断，禁止当作真机轨迹。")
    if args.csv:
        with Path(args.csv).open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.writer(stream)
            writer.writerow(["step", "target_x_m", "target_y_m", "target_z_m", "fk_x_m", "fk_y_m", "fk_z_m",
                             "id1_deg", "id2_deg", "id3_deg", "id4_deg", "id5_deg", "ik_error_m"])
            for i, (target, actual, q, error) in enumerate(path, 1):
                writer.writerow([i, *target, *actual, *np.degrees(q), error])
        print(f"CSV: {Path(args.csv).resolve()}")
    print("本诊断只计算 MuJoCo IK，不打开串口、不使能或移动电机。")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"诊断失败：{exc}", file=sys.stderr)
        raise SystemExit(2)
