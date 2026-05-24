#!/usr/bin/env python3
"""
auto_calibrate.py — walker 一键标定 orchestrator

流程：
  1. 检测并 SIGINT 当前的 ros2 launch slamwalker_*（记下命令以备重启）。
     如果 session_manager_node 在跑，会警告状态会脱节。
  2. 跑闭环 tick 标定（calibrate.run_calibration），用户量数据
  3. 把建议值写入：
       arduino/test/test.ino    — LEFT/RIGHT_MOTOR_SCALE *= 建议值
                                — WHEEL_BASE = effective_wheel_base
       3 个 launch 文件         — ticks_per_meter, wheel_base, right_encoder_scale
         · src/slamwalker_explore/launch/slamwalker_explore.launch.py
         · src/slamwalker_bringup/launch/slamwalker_slam.launch.py
         · src/slamwalker_bringup/launch/slamwalker_nav.launch.py
     (原文件 .bak 备份)
  4. arduino-cli compile → upload.sh（用户按 RESET）
  5. 用原命令重启 ros2 launch

只能在 Jetson（或别的真机 host）上跑：依赖 /dev/arduino + ros2 + arduino-cli。
跑前 source 一下 ROS：
  source /opt/ros/humble/setup.bash && source ~/walker_ws/install/setup.bash
"""
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from calibrate import run_calibration, ask_yes, read_current_calib  # noqa: E402

WS_ROOT = HERE.parent  # ~/walker_ws
TEST_INO = WS_ROOT / 'arduino' / 'test' / 'test.ino'
SKETCH_DIR = WS_ROOT / 'arduino' / 'test'
UPLOAD_SH = WS_ROOT / 'arduino' / 'upload.sh'
LAUNCH_FILES = [
    WS_ROOT / 'src/slamwalker_explore/launch/slamwalker_explore.launch.py',
    WS_ROOT / 'src/slamwalker_bringup/launch/slamwalker_slam.launch.py',
    WS_ROOT / 'src/slamwalker_bringup/launch/slamwalker_nav.launch.py',
]
ARDUINO_PORT = '/dev/arduino'
LAUNCH_LOG = '/tmp/walker_launch.log'


# ── 1. 停启 launch ───────────────────────────────────
def pgrep_af(pattern):
    """Return list of (pid, cmdline) matching pattern."""
    try:
        out = subprocess.check_output(['pgrep', '-af', pattern], text=True)
    except subprocess.CalledProcessError:
        return []
    res = []
    for line in out.strip().split('\n'):
        if not line.strip():
            continue
        pid_str, _, cmdline = line.partition(' ')
        try:
            res.append((int(pid_str), cmdline.strip()))
        except ValueError:
            pass
    return res


def find_launch_process():
    """ros2 launch slamwalker_*。"""
    return pgrep_af('ros2 launch slamwalker')


def find_session_manager():
    """session_manager_node。"""
    return pgrep_af('session_manager_node')


def port_in_use(port):
    try:
        out = subprocess.check_output(['lsof', '-t', port], text=True,
                                      stderr=subprocess.DEVNULL)
        return bool(out.strip())
    except subprocess.CalledProcessError:
        return False


def stop_launch(pid):
    print(f"  → SIGINT pid {pid}")
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        return
    for _ in range(24):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.5)
    else:
        print(f"  [WARN] {pid} 未在 12s 内退出，发 SIGTERM")
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        time.sleep(2)
    for _ in range(20):
        if not port_in_use(ARDUINO_PORT):
            print(f"  ✓ {ARDUINO_PORT} 已释放")
            return
        time.sleep(0.3)
    print(f"  [WARN] {ARDUINO_PORT} 仍被占用，可能影响标定")


def restart_launch(cmdline):
    print(f"\n  重启 launch: {cmdline}")
    print(f"  (日志 → {LAUNCH_LOG}，tail -f 看)")
    log = open(LAUNCH_LOG, 'w')
    subprocess.Popen(
        cmdline, shell=True, executable='/bin/bash',
        stdout=log, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )


# ── 2. 改文件 ─────────────────────────────────────────
def backup(path):
    bak = path.with_suffix(path.suffix + '.bak')
    shutil.copy(path, bak)
    print(f"  备份 → {bak.name}")


def _ino_const(text, name, cast=float):
    """Return value of `const <type> NAME = <value>;` or None."""
    m = re.search(rf'const\s+\w+\s+{name}\s*=\s*([-\d.]+)', text)
    return cast(m.group(1)) if m else None


def update_test_ino(result):
    """
    把 motor_bias 转成 PWM 增量，cumulative 加到 LEFT/RIGHT_PWM_BIAS。
    （也兼容旧的 LEFT/RIGHT_MOTOR_SCALE 命名 — 找到哪个改哪个。）

    注意：故意不改 Arduino 的 WHEEL_BASE — 切断反馈环让标定收敛。
    Arduino WHEEL_BASE 只影响 cmd_vel → 轮速翻译；bridge wheel_base 才
    影响 odom 精度。改 launch 的 wheel_base 即可让 SLAM/Nav 看到真实值，
    Arduino 那个 cmd_vel 时 Nav2 闭环会自动补偿命令/实际的小偏差。
    """
    if not TEST_INO.exists():
        print(f"  [SKIP] 找不到 {TEST_INO}")
        return False
    text = TEST_INO.read_text()
    orig = text
    changes = []

    # Motor bias → PWM bias (cumulative)
    motor_bias = result.get('motor_bias')
    lin_speed = result.get('lin_speed_used') or 0.075
    if motor_bias is not None and abs(motor_bias) > 1e-4:
        max_speed = _ino_const(text, 'MAX_SPEED') or 0.10
        pwm_min = _ino_const(text, 'PWM_MIN_MOVE', int) or 230
        pwm_range = 255 - pwm_min
        # Δv = motor_bias · lin_speed, ΔPWM = Δv / MAX_SPEED · pwm_range
        delta_pwm = motor_bias * lin_speed / max_speed * pwm_range
        delta_int = int(round(delta_pwm))
        if delta_int == 0:
            delta_int = 1 if motor_bias > 0 else -1
        # motor_bias > 0 → 右快 → 增加 RIGHT_PWM_BIAS（PWM 被减）
        if delta_int > 0:
            target = 'RIGHT_PWM_BIAS'
            inc = delta_int
        else:
            target = 'LEFT_PWM_BIAS'
            inc = -delta_int

        m = re.search(rf'(const\s+int\s+{target}\s*=\s*)(-?\d+)', text)
        if m:
            old_val = int(m.group(2))
            new_val = old_val + inc
            text = text[:m.start(2)] + str(new_val) + text[m.end(2):]
            changes.append(f'{target}: {old_val} → {new_val}  (+{inc})')
        else:
            # 兼容旧 MOTOR_SCALE 命名
            scale_name = ('RIGHT_MOTOR_SCALE' if motor_bias > 0
                          else 'LEFT_MOTOR_SCALE')
            scale_mult = (1.0 - motor_bias) if motor_bias > 0 else (1.0 + motor_bias)
            m2 = re.search(rf'const\s+float\s+{scale_name}\s*=\s*([\d.]+)', text)
            if m2:
                old_s = float(m2.group(1))
                new_s = old_s * scale_mult
                text = text[:m2.start(1)] + f'{new_s:.4f}' + text[m2.end(1):]
                changes.append(f'{scale_name}: {old_s:.4f} → {new_s:.4f}')
            else:
                print(f"  [WARN] test.ino 里既没 {target} 也没 {scale_name}，"
                      f"PWM 校正未落地。手动加：const int {target} = {inc};")

    if text == orig:
        print(f"  [SKIP] {TEST_INO.name} 无改动")
        return False
    backup(TEST_INO)
    TEST_INO.write_text(text)
    for c in changes:
        print(f"  {TEST_INO.name}: {c}")
    return True


def update_launch_file(path, result):
    if not path.exists():
        print(f"  [SKIP] 找不到 {path}")
        return False
    text = path.read_text()
    orig = text
    changes = []

    tpm = result['ticks_per_meter']
    text, n = re.subn(
        r"('ticks_per_meter'\s*:\s*)[\d.]+",
        rf"\g<1>{tpm:.1f}", text)
    if n:
        changes.append(f'ticks_per_meter = {tpm:.1f}')

    wb = result.get('effective_wheel_base')
    if wb:
        text, n = re.subn(
            r"('wheel_base'\s*:\s*)[\d.]+",
            rf"\g<1>{wb:.4f}", text)
        if n:
            changes.append(f'wheel_base = {wb:.4f}')

    res = result.get('right_encoder_scale')
    if res:
        if "'right_encoder_scale'" in text:
            text = re.sub(
                r"('right_encoder_scale'\s*:\s*)[\d.]+",
                rf"\g<1>{res:.4f}", text)
            changes.append(f'right_encoder_scale = {res:.4f}')
        else:
            text, n = re.subn(
                r"(\n(\s+))('base_frame'\s*:)",
                rf"\n\g<2>'right_encoder_scale': {res:.4f},\g<1>\g<3>",
                text, count=1)
            if n:
                changes.append(f'right_encoder_scale = {res:.4f} (inserted)')

    if text == orig:
        print(f"  [SKIP] {path.name} 无改动")
        return False
    backup(path)
    path.write_text(text)
    for c in changes:
        print(f"  {path.name}: {c}")
    return True


# ── 3. Flash Arduino ──────────────────────────────────
def _find_arduino_cli():
    """subprocess PATH 跟 shell PATH 可能不一致；显式找一下。"""
    import shutil as _sh
    p = _sh.which('arduino-cli')
    if p:
        return p
    for cand in ['~/.local/bin/arduino-cli', '/usr/local/bin/arduino-cli',
                 '/opt/arduino-cli/arduino-cli']:
        ep = os.path.expanduser(cand)
        if os.path.isfile(ep) and os.access(ep, os.X_OK):
            return ep
    return None


def flash_arduino():
    print("\n" + "=" * 58)
    print("【刷 Arduino 固件】")
    print("=" * 58)
    cli = _find_arduino_cli()
    if not cli:
        print("  [FAIL] 找不到 arduino-cli — 装到 PATH 或 ~/.local/bin/ 下。")
        print("  配置文件已改但固件未刷，请手动：")
        print(f"    arduino-cli compile --fqbn arduino:avr:uno {SKETCH_DIR}")
        print(f"    bash {UPLOAD_SH} {ARDUINO_PORT}")
        return False
    print(f"\n  编译 {SKETCH_DIR} (using {cli})...")
    r = subprocess.run(
        [cli, 'compile', '--fqbn', 'arduino:avr:uno', str(SKETCH_DIR)])
    if r.returncode != 0:
        print("  [FAIL] 编译失败 — 配置文件已改但固件未刷。手动修后重 upload。")
        return False
    print("\n  ★ 按 Arduino RESET 按钮，然后按 upload.sh 提示回车")
    print(f"  调用 {UPLOAD_SH} {ARDUINO_PORT}")
    r = subprocess.run(['bash', str(UPLOAD_SH), ARDUINO_PORT])
    return r.returncode == 0


# ── main ─────────────────────────────────────────────
def preview(result):
    print("\n" + "=" * 58)
    print("   即将写入")
    print("=" * 58)
    print(f"  {TEST_INO.name}: (WHEEL_BASE 故意冻结，不再改)")
    mb = result.get('motor_bias')
    if mb is not None and abs(mb) > 1e-4:
        # 用真实 test.ino 常量算 delta
        try:
            ttxt = TEST_INO.read_text()
            max_s = _ino_const(ttxt, 'MAX_SPEED') or 0.10
            pwm_min = _ino_const(ttxt, 'PWM_MIN_MOVE', int) or 230
        except Exception:
            max_s, pwm_min = 0.10, 230
        lin_s = result.get('lin_speed_used') or 0.075
        delta_pwm = mb * lin_s / max_s * (255 - pwm_min)
        delta_int = int(round(delta_pwm)) or (1 if mb > 0 else -1)
        if delta_int > 0:
            print(f"    RIGHT_PWM_BIAS += {delta_int}  (cumulative)")
        else:
            print(f"    LEFT_PWM_BIAS += {-delta_int}  (cumulative)")
    print("  launch files (3 个):")
    for lf in LAUNCH_FILES:
        print(f"    - {lf.name}")
    print(f"    ticks_per_meter = {result['ticks_per_meter']:.1f}")
    if result.get('effective_wheel_base'):
        print(f"    wheel_base = {result['effective_wheel_base']:.4f}")
    if result.get('right_encoder_scale'):
        print(f"    right_encoder_scale = {result['right_encoder_scale']:.4f}")
    print()


def main():
    print("╔" + "═" * 56 + "╗")
    print("║   walker auto_calibrate — 一键标定 + 刷固件 + 重启   ║")
    print("╚" + "═" * 56 + "╝")
    print()

    # 当前标定值（仅展示）
    tpm_cur, wb_cur, src_file = read_current_calib()
    if tpm_cur and wb_cur:
        print(f"  当前 launch ({src_file}): tpm={tpm_cur:.1f}, wb={wb_cur:.4f}")
        print()

    # 1. detect launch + session_manager
    sm_procs = find_session_manager()
    if sm_procs:
        print(f"  ⚠ 检测到 session_manager_node ({len(sm_procs)} 个进程)：")
        for pid, cmd in sm_procs:
            print(f"      [{pid}] {cmd}")
        print("    本脚本会直接 kill launch；session_manager 内部 _proc/_mode 状态会脱节。")
        print("    建议你完成后从 RViz panel 重新点 'Start Bringup'（或重启 session_manager）。")
        if not ask_yes("\n  继续？"):
            print("  退出。")
            return

    procs = find_launch_process()
    launch_cmd = None
    if procs:
        print(f"\n  检测到 {len(procs)} 个 ros2 launch slamwalker_* 进程：")
        for pid, cmd in procs:
            print(f"    [{pid}] {cmd}")
        launch_cmd = procs[0][1]
        if not ask_yes("\n  停掉跑标定吗？"):
            print("  退出。")
            return
        for pid, _ in procs:
            stop_launch(pid)
    else:
        print("\n  [INFO] 没检测到 ros2 launch slamwalker_*，跳过停启。")
        if port_in_use(ARDUINO_PORT):
            print(f"  [WARN] 但 {ARDUINO_PORT} 在用。 lsof {ARDUINO_PORT}")

    # 2. calibrate（传当前值给 calibrate 算自适应默认）
    print()
    result = run_calibration(
        defaults={'tpm': tpm_cur, 'wheel_base': wb_cur, 'src_file': src_file})
    if not result or not result.get('ticks_per_meter'):
        print("  [ABORT] 标定无结果，啥也不改。")
        if launch_cmd:
            restart_launch(launch_cmd)
        return

    # 3. preview + confirm
    preview(result)
    if not ask_yes("  确认写入并刷固件？"):
        print("  取消，不写盘。")
        if launch_cmd:
            restart_launch(launch_cmd)
        return

    # 4. apply
    print("\n" + "=" * 58)
    print("【写入文件】")
    print("=" * 58)
    update_test_ino(result)
    for lf in LAUNCH_FILES:
        update_launch_file(lf, result)

    # 5. flash
    flash_ok = flash_arduino()

    # 6. restart launch
    if launch_cmd:
        if flash_ok or ask_yes("\n  刷固件失败/跳过，仍然重启 launch?"):
            restart_launch(launch_cmd)
        else:
            print("  launch 未重启。手动: ", launch_cmd)
    elif sm_procs:
        print("\n  没记到原 launch 命令。从 RViz panel 重启 bringup，或:")
        print("    ros2 launch slamwalker_explore slamwalker_explore.launch.py "
              "start_explorer:=false")
    else:
        print("\n  没有原 launch 可重启。手动起：")
        print("    ros2 launch slamwalker_explore slamwalker_explore.launch.py "
              "start_explorer:=false")

    print("\n  ✓ 完成。")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n  [Ctrl+C] 已退出。")
