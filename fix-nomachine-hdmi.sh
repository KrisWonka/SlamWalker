#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "请用 sudo 运行：sudo bash $0"
  exit 1
fi

echo "[1/6] 写入 nxserver systemd 覆盖配置..."
mkdir -p /etc/systemd/system/nxserver.service.d
cat >/etc/systemd/system/nxserver.service.d/override.conf <<'EOF'
[Unit]
After=display-manager.service gdm.service graphical.target
Wants=graphical.target

[Service]
ExecStartPre=/bin/sleep 8
EOF

echo "[2/6] 备份 NoMachine 配置..."
cp -a /usr/NX/etc/server.cfg "/usr/NX/etc/server.cfg.bak.$(date +%Y%m%d_%H%M%S)"
cp -a /usr/NX/etc/node.cfg "/usr/NX/etc/node.cfg.bak.$(date +%Y%m%d_%H%M%S)"

echo "[3/6] 设置 CreateDisplay 0（显式禁用自动创建显示）..."
if grep -qE '^#CreateDisplay 0$' /usr/NX/etc/server.cfg; then
  sed -i 's/^#CreateDisplay 0$/CreateDisplay 0/' /usr/NX/etc/server.cfg
elif grep -qE '^CreateDisplay[[:space:]]+' /usr/NX/etc/server.cfg; then
  sed -i 's/^CreateDisplay[[:space:]].*/CreateDisplay 0/' /usr/NX/etc/server.cfg
else
  printf '\nCreateDisplay 0\n' >>/usr/NX/etc/server.cfg
fi

echo "[4/6] 设置 EnableEGLCapture 0（避免捕获路径干扰）..."
if grep -qE '^EnableEGLCapture[[:space:]]+' /usr/NX/etc/node.cfg; then
  sed -i 's/^EnableEGLCapture[[:space:]].*/EnableEGLCapture 0/' /usr/NX/etc/node.cfg
else
  printf '\nEnableEGLCapture 0\n' >>/usr/NX/etc/node.cfg
fi

echo "[5/6] 确认 GDM 关闭 Wayland..."
if [[ -f /etc/gdm3/custom.conf ]]; then
  if grep -qE '^#?WaylandEnable=' /etc/gdm3/custom.conf; then
    sed -i 's/^#\?WaylandEnable=.*/WaylandEnable=false/' /etc/gdm3/custom.conf
  else
    printf '\n[daemon]\nWaylandEnable=false\n' >>/etc/gdm3/custom.conf
  fi
elif [[ -f /etc/gdm/custom.conf ]]; then
  if grep -qE '^#?WaylandEnable=' /etc/gdm/custom.conf; then
    sed -i 's/^#\?WaylandEnable=.*/WaylandEnable=false/' /etc/gdm/custom.conf
  else
    printf '\n[daemon]\nWaylandEnable=false\n' >>/etc/gdm/custom.conf
  fi
else
  echo "未找到 /etc/gdm3/custom.conf 或 /etc/gdm/custom.conf，跳过 Wayland 设置。"
fi

echo "[6/6] 重载并重启服务..."
systemctl daemon-reload
systemctl restart gdm
systemctl restart nxserver

echo
echo "修复完成。建议重启验证：sudo reboot"
