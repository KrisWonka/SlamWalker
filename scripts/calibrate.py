#!/usr/bin/env python3
"""
SlamWalker 步行机构里程计标定工具（闭环 tick 版）

跟以前不一样：脚本直接控制 Arduino 驱动机器人到固定 tick 数后停车，
你只负责量实际距离 / 角度 / 横向偏移。

标定流程：
  第 1 步: 脚本前进固定 tick → 你量实际距离 → ticks_per_meter
           顺便量横向偏移 → right_encoder_scale + left/right_scale 建议
  第 2 步: 脚本原地旋转固定 tick → 你量实际角度 → effective_wheel_base

用法:
  1. ★ 先停掉 bridge（占用同一个串口）。如果用 session_manager + RViz panel
     起的，建议先 EMERGENCY STOP；或直接 pkill -f serial_bridge_node。
  2. 运行: python3 ~/walker_ws/scripts/calibrate.py
     （要"一键标定 + 刷固件 + 重启"，跑 auto_calibrate.py 而不是这个）
  3. 按提示操作；Ctrl+C 紧急停车。
"""
import datetime
import errno
import math
import os
import re
import select
import shutil
import sys
import threading
import time
from pathlib import Path

import serial

PORT = '/dev/arduino'
BAUD = 115200

# 回退默认（读不到 launch 里当前值时用）
FALLBACK_LIN_TICKS = 10000   # ≈ 1m，按理论 tpm=9970
FALLBACK_ANG_TICKS = 14000   # 仅作回退；实际会按当前 wb/tpm 算一圈
DEFAULT_LIN_SPEED = 0.075    # m/s — 匹配 Nav2 max_vel_x 上限
DEFAULT_ANG_SPEED = 0.6      # rad/s — 远低于 Nav2 上限，转得更稳
DRIVE_TIMEOUT = 60.0         # s，安全上限（大 wb + 慢速可能 30+s）
CMD_HZ = 20.0


def _ws_root():
    return Path(__file__).resolve().parent.parent


# 标定前要快照的文件（auto_calibrate.py 会改的全部）
SNAPSHOT_TARGETS = [
    'arduino/test/test.ino',
    'src/slamwalker_explore/launch/slamwalker_explore.launch.py',
    'src/slamwalker_bringup/launch/slamwalker_slam.launch.py',
    'src/slamwalker_bringup/launch/slamwalker_nav.launch.py',
]


def snapshot_state(tag='precalib'):
    """Copy 当前 test.ino + 3 个 launch 文件到带时间戳的快照。

    返回 (ok_count, total)。失败的文件只打 WARN 不抛异常 — 标定不能因为
    snapshot 出问题而中断。
    """
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    ws = _ws_root()
    ok = 0
    total = 0
    print(f"\n  [snapshot] tag={tag} ts={ts}")
    for rel in SNAPSHOT_TARGETS:
        total += 1
        src = ws / rel
        if not src.exists():
            print(f"    [SKIP] {rel}  (不存在)")
            continue
        dst = src.with_name(f'{src.name}.{tag}_{ts}')
        try:
            shutil.copy2(src, dst)
            print(f"    ✓ {rel} → {dst.name}")
            ok += 1
        except Exception as e:
            print(f"    [WARN] {rel}: {e}")
    print(f"  [snapshot] {ok}/{total} 文件已备份\n")
    return ok, total


# ── 自适应默认：从 launch 文件读当前生效的 tpm + wheel_base ───────
def read_current_calib():
    """
    扫 walker_ws 下的 launch 文件，返回 (tpm, wb, src_filename) 或 (None, None, None)。
    优先 explore（session_manager 实际用的），再退 slam，再退 nav。
    """
    ws_root = Path(__file__).resolve().parent.parent  # ~/walker_ws
    candidates = [
        ws_root / 'src/slamwalker_explore/launch/slamwalker_explore.launch.py',
        ws_root / 'src/slamwalker_bringup/launch/slamwalker_slam.launch.py',
        ws_root / 'src/slamwalker_bringup/launch/slamwalker_nav.launch.py',
    ]
    for p in candidates:
        if not p.exists():
            continue
        try:
            text = p.read_text()
        except Exception:
            continue
        tpm_m = re.search(r"'ticks_per_meter'\s*:\s*([\d.]+)", text)
        wb_m = re.search(r"'wheel_base'\s*:\s*([\d.]+)", text)
        if tpm_m and wb_m:
            return float(tpm_m.group(1)), float(wb_m.group(1)), p.name
    return None, None, None


def open_serial():
    s = serial.Serial(PORT, BAUD, timeout=0, dsrdtr=False, rtscts=False)
    time.sleep(2)
    s.reset_input_buffer()
    return s


def send_cmd(ser, lx, az):
    try:
        ser.write(f'V{lx:.4f},{az:.4f}#\n'.encode())
    except serial.SerialException as e:
        print(f"  [WARN] serial write failed: {e}")


def hard_stop(ser, n=5):
    for _ in range(n):
        send_cmd(ser, 0.0, 0.0)
        time.sleep(1.0 / CMD_HZ)


class TickRecorder:
    """后台线程读 `O<dL>,<dR>,<dt>` 累加。"""

    def __init__(self, ser):
        self.ser = ser
        self.total_L = 0
        self.total_R = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None

    def reset(self):
        with self._lock:
            self.total_L = 0
            self.total_R = 0

    def snapshot(self):
        with self._lock:
            return self.total_L, self.total_R

    def _run(self):
        buf = b''
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self.ser.fileno()], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(self.ser.fileno(), 512)
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    lb, buf = buf.split(b'\n', 1)
                    line = lb.decode(errors='replace').strip()
                    if line.startswith('O'):
                        try:
                            parts = line[1:].split(',')
                            dl = int(parts[0])
                            dr = int(parts[1])
                            with self._lock:
                                self.total_L += dl
                                self.total_R += dr
                        except (ValueError, IndexError):
                            pass
            except OSError as e:
                if e.errno == errno.EAGAIN:
                    time.sleep(0.01)
                    continue
                break


def drive_until_ticks(ser, rec, lx, az, target_ticks, metric='avg',
                      timeout=DRIVE_TIMEOUT):
    """恒速驱动到 metric 累计 ≥ target_ticks 停车。

    metric:
      'avg'  → (|tL| + |tR|) / 2，用于直线
      'diff' → |tR - tL|，用于原地旋转
    """
    rec.reset()
    t0 = time.time()
    last_cmd = 0.0
    reached = False
    try:
        while True:
            now = time.time()
            if now - t0 > timeout:
                print(f"\n  [TIMEOUT] {timeout:.1f}s 到，未达到目标 tick")
                break
            if now - last_cmd >= 1.0 / CMD_HZ:
                send_cmd(ser, lx, az)
                last_cmd = now
            time.sleep(0.01)

            tL, tR = rec.snapshot()
            m = (abs(tL) + abs(tR)) / 2.0 if metric == 'avg' else abs(tR - tL)

            if int((now - t0) * 2) % 4 == 0:
                sys.stdout.write(
                    f"\r  L={tL:+6d}  R={tR:+6d}  metric={m:6.0f}/{target_ticks}    ")
                sys.stdout.flush()

            if m >= target_ticks:
                reached = True
                break
    except KeyboardInterrupt:
        print("\n  [Ctrl+C] 紧急停车")
    finally:
        hard_stop(ser)

    elapsed = time.time() - t0
    tL, tR = rec.snapshot()
    sys.stdout.write(
        f"\r  L={tL:+6d}  R={tR:+6d}  耗时 {elapsed:.1f}s"
        f"  {'✓达标' if reached else '未达标'}        \n")
    sys.stdout.flush()
    return tL, tR, elapsed, reached


def ask_float(prompt, default=None, allow_zero=False):
    while True:
        suffix = f" [{default}]" if default is not None else ""
        raw = input(f"{prompt}{suffix}: ").strip()
        if raw == "" and default is not None:
            return float(default)
        try:
            v = float(raw)
            if v == 0 and not allow_zero:
                print("  必须 > 0")
                continue
            return v
        except ValueError:
            print("  请输入数字")


def ask_yes(prompt):
    raw = input(f"{prompt} [Y/n]: ").strip().lower()
    return raw in ('', 'y', 'yes')


def run_calibration(defaults=None):
    """跑闭环 tick 标定，返回 result dict。

    defaults: 可选 dict，含 'tpm', 'wheel_base', 'src_file' — 用来算默认 tick
              target 和提示文案。None 时自动从 launch 文件读取。
    返回字段（部分可能为 None）:
      ticks_per_meter, effective_wheel_base, right_encoder_scale,
      lateral_cm, drift_deg, right_scale_suggested, left_scale_suggested
    """
    if defaults is None:
        tpm_cur, wb_cur, src_file = read_current_calib()
    else:
        tpm_cur = defaults.get('tpm')
        wb_cur = defaults.get('wheel_base')
        src_file = defaults.get('src_file')

    lin_default = int(round(tpm_cur)) if tpm_cur else FALLBACK_LIN_TICKS
    if tpm_cur and wb_cur:
        ang_default = int(round(2 * math.pi * wb_cur * tpm_cur))  # ~360°
    else:
        ang_default = FALLBACK_ANG_TICKS

    print("=" * 58)
    print("   SlamWalker 步行机构里程计标定（闭环 tick）")
    print("=" * 58)
    print()
    print("  ★ 前提：bridge 已停掉，本脚本独占串口。")
    print("    Ctrl+C 可紧急停车。")
    if tpm_cur and wb_cur:
        print(f"\n  当前生效标定 (from {src_file}):")
        print(f"    ticks_per_meter = {tpm_cur:.1f}")
        print(f"    wheel_base      = {wb_cur:.4f} m")
    print()

    try:
        ser = open_serial()
    except Exception as e:
        print(f"  打不开 {PORT}: {e}")
        print(f"  检查：bridge 是否还在跑？sudo lsof {PORT}")
        sys.exit(1)

    rec = TickRecorder(ser)
    rec.start()
    time.sleep(0.5)
    rec.reset()

    # ── Step 1: 直线标定 ──────────────────────────────
    print("【第 1 步：直线 tick 驱动】")
    if tpm_cur:
        m1 = lin_default / tpm_cur
        print(f"  默认 {lin_default} ticks @ {DEFAULT_LIN_SPEED} m/s "
              f"（按当前 tpm 估 ≈ {m1:.2f}m，{m1/DEFAULT_LIN_SPEED:.0f}s）")
    else:
        print(f"  默认 {lin_default} ticks @ {DEFAULT_LIN_SPEED} m/s")
    print()
    lin_speed = ask_float("  线速度 (m/s, 负值倒退)", DEFAULT_LIN_SPEED,
                          allow_zero=False)
    lin_ticks = int(ask_float("  目标 tick 数", lin_default))
    print()
    print("  在地上标出起点 + 前进朝向。准备好后启动。")
    input("  按 Enter 启动直线驱动... ")

    tL, tR, t_lin, ok_lin = drive_until_ticks(
        ser, rec, lin_speed, 0.0, lin_ticks, metric='avg')

    if not ok_lin and abs(tL) + abs(tR) < 100:
        print("  编码器几乎没动 — 检查电机/编码器接线后再来。")
        rec.stop()
        ser.close()
        sys.exit(1)

    avg_ticks = (abs(tL) + abs(tR)) / 2.0
    actual_dist = ask_float("  请输入实际行走距离 (米)", None)
    ticks_per_meter = avg_ticks / actual_dist
    print(f"\n  ★ ticks_per_meter = {ticks_per_meter:.1f}")

    # 机器人确认能动 + 已有真实数据 → 在改任何文件之前快照
    snapshot_state(tag='precalib')

    # ── 走直度分析 ───────────────────────────────────
    print("\n  --- 走直度分析 ---")
    right_enc_scale = None
    if abs(tR) > 0:
        ratio = abs(tL) / abs(tR)
        print(f"  左右编码器比 tL/tR = {ratio:.4f}  (差 {(ratio-1)*100:+.2f}%)")
        if abs(ratio - 1.0) > 0.005:
            right_enc_scale = ratio
            print(f"  ★ 建议 right_encoder_scale = {right_enc_scale:.4f}")
            print("    (bridge 默认 1.041；这一项只修编码器读数差)")
        else:
            print("  编码器读数一致 (<0.5%)，无需改 right_encoder_scale")

    print()
    print("  量终点的横向偏移（站起点朝终点看：左偏为正，右偏为负）:")
    raw = input("  横向偏移 (cm，无偏移留空): ").strip()
    try:
        lateral_cm = float(raw) if raw else 0.0
    except ValueError:
        print("  非数字，按 0 处理")
        lateral_cm = 0.0

    motor_bias = None   # 速度分数 (vR - vL) / v_avg
    drift_deg = None
    if abs(lateral_cm) > 0.1:
        lateral_m = lateral_cm / 100.0
        drift_deg = math.degrees(math.atan2(lateral_m, actual_dist))
        # 末态横向偏移 ≈ b · d² / (2·wb), b = (v_R-v_L)/v_avg
        wb_for_bias = wb_cur if wb_cur else 0.40
        motor_bias = 2 * wb_for_bias * lateral_m / (actual_dist ** 2)
        print(f"\n  终点漂移角 ≈ {drift_deg:+.2f}°")
        print(f"  电机不对称估算 bias ≈ {motor_bias*100:+.2f}%  (右相对左)")
        print(f"    (基于 wheel_base={wb_for_bias:.4f}；第 2 步若有 effective_base 会重算)")
        # 粗估 PWM bias delta（用 test.ino 默认 MAX_SPEED=0.10, PWM_MIN_MOVE=230）
        pwm_delta_est = int(round(motor_bias * lin_speed / 0.10 * 25))
        if pwm_delta_est == 0 and abs(motor_bias) > 0:
            pwm_delta_est = 1 if motor_bias > 0 else -1
        if motor_bias > 0:
            print(f"  ★ 建议 RIGHT_PWM_BIAS += {pwm_delta_est} (右轮太快，PWM 降一点)")
        else:
            print(f"  ★ 建议 LEFT_PWM_BIAS += {-pwm_delta_est} (左轮太快，PWM 降一点)")

    # ── Step 2: 旋转标定 ──────────────────────────────
    print("\n" + "=" * 58)
    print("【第 2 步：原地旋转 tick 驱动】")
    if tpm_cur and wb_cur:
        t_est = ang_default / (wb_cur * DEFAULT_ANG_SPEED * tpm_cur)
        print(f"  默认 {ang_default} 差分 ticks @ {DEFAULT_ANG_SPEED} rad/s "
              f"（按当前 tpm/wb 估约一圈, {t_est:.1f}s）")
    else:
        print(f"  默认 {ang_default} 差分 ticks @ {DEFAULT_ANG_SPEED} rad/s")
    effective_base = None
    if ask_yes("  继续做旋转标定？"):
        ang_speed = ask_float(
            "  角速度 (rad/s，正=逆时针)", DEFAULT_ANG_SPEED, allow_zero=False)
        ang_ticks = int(ask_float("  目标差分 tick 数", ang_default))
        print()
        print("  原地放好，记录初始朝向（建议贴标签 / 拿手机标线）。")
        input("  按 Enter 启动旋转驱动... ")

        tL2, tR2, t_ang, ok_ang = drive_until_ticks(
            ser, rec, 0.0, ang_speed, ang_ticks, metric='diff')

        actual_angle_deg = ask_float("  请输入实际旋转角度 (度)", None)
        actual_rad = actual_angle_deg * math.pi / 180.0
        tick_diff_m = abs(tR2 - tL2) / ticks_per_meter
        effective_base = tick_diff_m / actual_rad
        print(f"\n  ★ effective_wheel_base = {effective_base:.4f} m")

        # 用真实 wheel_base 重算 motor bias
        if abs(lateral_cm) > 0.1:
            lateral_m = lateral_cm / 100.0
            motor_bias = 2 * effective_base * lateral_m / (actual_dist ** 2)
            print(f"  (用 effective_base 重算 bias ≈ {motor_bias*100:+.2f}%)")

    rec.stop()
    ser.close()

    result = {
        'ticks_per_meter': ticks_per_meter,
        'effective_wheel_base': effective_base,
        'right_encoder_scale': right_enc_scale,
        'lateral_cm': lateral_cm if abs(lateral_cm) > 0.1 else None,
        'drift_deg': drift_deg,
        'motor_bias': motor_bias,        # 速度分数 (vR-vL)/v_avg；正=右快
        'lin_speed_used': lin_speed,     # update_test_ino 用来算 PWM delta
    }

    # ── 收敛性 sanity check ──────────────────────────
    if tpm_cur and abs(ticks_per_meter - tpm_cur) / tpm_cur > 0.1:
        print(f"  ⚠ tpm 跟上次差 {(ticks_per_meter-tpm_cur)/tpm_cur*100:+.1f}%；")
        print(f"    建议重跑 1-2 次取中位数再落地。")
    if wb_cur and effective_base and abs(effective_base - wb_cur) / wb_cur > 0.1:
        print(f"  ⚠ wb 跟上次差 {(effective_base-wb_cur)/wb_cur*100:+.1f}%；")
        print(f"    步行机构旋转噪声大，建议跑 3 次取中位数。")

    # ── Summary ───────────────────────────────────────
    print("\n" + "=" * 58)
    print("   标定结果")
    print("=" * 58)
    print(f"  ticks_per_meter      = {ticks_per_meter:.1f}")
    if effective_base:
        print(f"  effective_wheel_base = {effective_base:.4f} m")
    if right_enc_scale:
        print(f"  right_encoder_scale  = {right_enc_scale:.4f}")
    if drift_deg is not None:
        print(f"  终点漂移角           = {drift_deg:+.2f}°  ({lateral_cm:+.1f} cm)")
    if motor_bias is not None:
        pwm_delta = int(round(motor_bias * lin_speed / 0.10 * 25))
        if pwm_delta == 0:
            pwm_delta = 1 if motor_bias > 0 else -1
        if motor_bias > 0:
            print(f"  PWM bias 建议        = RIGHT_PWM_BIAS += {pwm_delta}  (cumulative)")
        else:
            print(f"  PWM bias 建议        = LEFT_PWM_BIAS += {-pwm_delta}  (cumulative)")
    print()
    return result


def main():
    """独立运行模式 — 只标定不落地，打印手动启动命令。"""
    result = run_calibration()
    cmd = "ros2 run slamwalker_bridge serial_bridge_node --ros-args"
    cmd += f" -p ticks_per_meter:={result['ticks_per_meter']:.1f}"
    if result['effective_wheel_base']:
        cmd += f" -p wheel_base:={result['effective_wheel_base']:.4f}"
    if result['right_encoder_scale']:
        cmd += f" -p right_encoder_scale:={result['right_encoder_scale']:.4f}"
    print("  仅 ROS 端启动命令（不含 Arduino 端 PWM bias）:")
    print(f"    {cmd}")
    print()
    print("  要把 PWM bias 烧进 Arduino + 改 3 个 launch + 重启栈，跑:")
    print("    python3 ~/walker_ws/scripts/auto_calibrate.py")
    print()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n  已退出。")
